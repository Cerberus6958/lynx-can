// Two-node CAN bench test for ESP32 + SN65HVD230 (ESP-IDF, plain C)
// Flash board A with NODE=1 and board B with NODE=2.
// Each node transmits its own schedule, receives everything, and prints
// per-ID stats + controller error state once per second.
//
// Printing happens in its own low-priority task. The RX task only copies a
// snapshot of the stats, so slow UART output never delays frame timestamps.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_timer.h"
#include "can_frame.h"

// ---------------- build options ----------------
#ifndef NODE
#define NODE 1            // 1 = "sensor" node; 2 = "VCU/dash" node
#endif
#ifndef SELF_TEST
#define SELF_TEST 0       // 1 = single board; no ACK needed; receives its own frames
#endif
#ifndef STRESS_HZ
#define STRESS_HZ 0       // >0 = node 1 also sends filler ID 0x200 at this rate to raise bus load
#endif
#ifndef OPEN_DRAIN_BUS
#define OPEN_DRAIN_BUS 0   // 1 = wire-only two-board bus using internal pull-ups
#endif

#define CAN_TX_PIN      GPIO_NUM_4     // ESP32 TX -> transceiver D / CTX
#define CAN_RX_PIN      GPIO_NUM_5     // ESP32 RX <- transceiver R / CRX
#ifndef CAN_KBPS
#define CAN_KBPS 500       // 125, 250, 500 or 1000
#endif
#define BITRATE         (CAN_KBPS * 1000.0f)

#if CAN_KBPS == 1000
#define CAN_TIMING() TWAI_TIMING_CONFIG_1MBITS()
#elif CAN_KBPS == 500
#define CAN_TIMING() TWAI_TIMING_CONFIG_500KBITS()
#elif CAN_KBPS == 250
#define CAN_TIMING() TWAI_TIMING_CONFIG_250KBITS()
#elif CAN_KBPS == 125
#define CAN_TIMING() TWAI_TIMING_CONFIG_125KBITS()
#else
#error "CAN_KBPS must be 125, 250, 500 or 1000"
#endif
#define BITS_PER_FRAME  125.0f         // 8-byte std frame incl. typical stuffing + IFS
#define FILLER_ID       0x200

// ---------------- test schedule ----------------
// Mirrors the priority order in the design journal (safety first).
// IDs from both nodes interleave so the two boards genuinely arbitrate.
typedef struct {
    uint32_t id;
    uint8_t node;
    uint32_t period_ms;
    const char *name;
} Schedule;

static const Schedule kSchedule[] = {
    {0x050, 1,    5, "imu_like"},     // 200 Hz (set to 1 for the 1 kHz test)
    {0x0A0, 2,   20, "vcu_cmd"},      //  50 Hz
    {0x100, 1,   10, "wheel_spd"},    // 100 Hz
    {0x300, 1,  100, "bms_like"},     //  10 Hz
    {0x600, 2, 1000, "diag"},         //   1 Hz
};
#define N (sizeof(kSchedule) / sizeof(kSchedule[0]))

typedef struct {
    uint64_t next_us;
    uint32_t counter;
    uint32_t queue_full;
} TxState;

typedef struct {
    bool seen;
    uint32_t frames, lost, too_fast, last_counter;
    uint64_t last_us;
    uint32_t min_gap_us, max_gap_us;
} RxStats;

static TxState tx[N];
static RxStats rx[N];                            // owned by rx_task
static volatile uint32_t tx_frames_total = 0;   // written by tx_task only

// Snapshot handed from rx_task to report_task once per second
static RxStats snap[N];
static uint32_t snap_rx_window = 0;
static TaskHandle_t report_handle = NULL;

static int index_of(uint32_t id) {
    for (int i = 0; i < (int)N; i++) {
        if (kSchedule[i].id == id) return i;
    }
    return -1;
}

// ---------------- thin TWAI wrapper ----------------
static bool can_begin(void) {
    twai_mode_t mode = SELF_TEST ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL;
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, mode);
    g.tx_queue_len = 16;
    g.rx_queue_len = 32;
    g.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                       TWAI_ALERT_ERR_PASS | TWAI_ALERT_ERR_ACTIVE |
                       TWAI_ALERT_RX_QUEUE_FULL;
    twai_timing_config_t t = CAN_TIMING();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
#if OPEN_DRAIN_BUS
    gpio_od_enable(CAN_TX_PIN);    // TX can only pull low; safe to join with other TX pins
    gpio_pullup_en(CAN_TX_PIN);    // internal pull-ups hold the bus high (recessive)
    gpio_pullup_en(CAN_RX_PIN);
#endif
    return twai_start() == ESP_OK;
}

static bool can_send(const CANframe *fr, uint32_t timeout_ms) {
    twai_message_t m = {0};
    m.identifier = fr->id;
    m.data_length_code = fr->dlc;
    m.self = SELF_TEST;                        // self-reception in self-test mode
    memcpy(m.data, fr->data, fr->dlc);
    return twai_transmit(&m, pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
}

static bool can_receive(CANframe *fr, uint32_t timeout_ms) {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(timeout_ms)) != ESP_OK) return false;
    if (m.extd || m.rtr) return false;         // standard data frames only
    fr->id = m.identifier;
    fr->dlc = m.data_length_code > 8 ? 8 : m.data_length_code;
    memcpy(fr->data, m.data, fr->dlc);
    fr->timestamp_us = esp_timer_get_time();
    return true;
}

// ---------------- transmit task ----------------
// Payload: bytes 0-3 = sequence counter; bytes 4-7 = sender's micros (debug only)
static void tx_task(void *arg) {
    uint64_t now = esp_timer_get_time();
    // Stagger start times by whole 1 ms ticks so messages never pile up in the same tick.
    // (Offsets smaller than the 1 ms tick don't help: the task only checks once per tick.)
    for (int i = 0; i < (int)N; i++) tx[i].next_us = now + (uint64_t)i * 1000;
    uint64_t filler_next_us = now;
    uint32_t filler_sent = 0;
    (void)filler_next_us;
    (void)filler_sent;

    while (1) {
        now = esp_timer_get_time();

        for (int i = 0; i < (int)N; i++) {
            if (!SELF_TEST && kSchedule[i].node != NODE) continue;
            if (now < tx[i].next_us) continue;

            tx[i].next_us += kSchedule[i].period_ms * 1000ULL;
            if (tx[i].next_us < now) tx[i].next_us = now;   // fell behind; don't burst

            CANframe f = {0};
            f.id = kSchedule[i].id;
            f.dlc = 8;
            put_u32(f.data, tx[i].counter++);   // counter advances even if queue is full,
            put_u32(f.data + 4, (uint32_t)now); // so local drops show as "lost" at the receiver
            if (can_send(&f, 0)) tx_frames_total++;
            else tx[i].queue_full++;
        }

#if STRESS_HZ > 0
        // Task wakes every 1 ms, so send every filler frame that is due (rates above 1 kHz)
        while (NODE == 1 && now >= filler_next_us) {
            filler_next_us += 1000000ULL / STRESS_HZ;
            CANframe f = {0};
            f.id = FILLER_ID;
            f.dlc = 8;
            put_u32(f.data, filler_sent);
            if (can_send(&f, 0)) { filler_sent++; tx_frames_total++; }
            else { filler_next_us = now + 1000000ULL / STRESS_HZ; break; }  // queue full; don't spin
        }
#endif

        vTaskDelay(1);   // 1 ms with CONFIG_FREERTOS_HZ=1000 (see sdkconfig.defaults)
    }
}

// ---------------- error state / bus-off recovery task ----------------
static void alert_task(void *arg) {
    while (1) {
        uint32_t alerts;
        if (twai_read_alerts(&alerts, portMAX_DELAY) != ESP_OK) continue;
        if (alerts & TWAI_ALERT_ERR_PASS)      printf("!! ERROR PASSIVE (TEC or REC >= 128)\n");
        if (alerts & TWAI_ALERT_ERR_ACTIVE)    printf("!! back to ERROR ACTIVE\n");
        if (alerts & TWAI_ALERT_RX_QUEUE_FULL) printf("!! RX queue full; frames dropped\n");
        if (alerts & TWAI_ALERT_BUS_OFF) {
            printf("!! BUS OFF; starting recovery\n");
            twai_initiate_recovery();          // waits for 128 x 11 recessive bits
        }
        if (alerts & TWAI_ALERT_BUS_RECOVERED) {
            printf("!! recovered; restarting controller\n");
            twai_start();
        }
    }
}

// ---------------- report ----------------
static const char *state_name(twai_state_t s) {
    switch (s) {
        case TWAI_STATE_STOPPED:    return "STOPPED";
        case TWAI_STATE_RUNNING:    return "RUNNING";
        case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
        case TWAI_STATE_RECOVERING: return "RECOVERING";
        default:                    return "?";
    }
}

// Prints from the snapshot only; never touches the live rx[] stats
static void report(uint32_t rx_frames_window) {
    static uint32_t last_tx_total = 0;
    uint32_t tx_total = tx_frames_total;
    uint32_t tx_window = tx_total - last_tx_total;
    last_tx_total = tx_total;

    twai_status_info_t st;
    twai_get_status_info(&st);
    float load = (tx_window + rx_frames_window) * BITS_PER_FRAME / BITRATE * 100.0f;

    printf("\n[node %d] %s TEC=%" PRIu32 " REC=%" PRIu32 " txq=%" PRIu32 " rxq=%" PRIu32
           " tx_fail=%" PRIu32 " rx_missed=%" PRIu32 " arb_lost=%" PRIu32
           " bus_err=%" PRIu32 " load~%.1f%%\n",
           NODE, state_name(st.state), st.tx_error_counter, st.rx_error_counter,
           st.msgs_to_tx, st.msgs_to_rx, st.tx_failed_count, st.rx_missed_count,
           st.arb_lost_count, st.bus_error_count, load);
    printf("  id     name        rx/s  lost  fast  qfull  min_gap_us  max_gap_us\n");

    for (int i = 0; i < (int)N; i++) {
        const RxStats *s = &snap[i];
        printf("  0x%03" PRIX32 "  %-10s  %4" PRIu32 "  %4" PRIu32 "  %4" PRIu32
               "  %5" PRIu32 "  %10" PRIu32 "  %10" PRIu32 "\n",
               kSchedule[i].id, kSchedule[i].name, s->frames, s->lost, s->too_fast,
               tx[i].queue_full, s->min_gap_us == UINT32_MAX ? 0 : s->min_gap_us,
               s->max_gap_us);
    }
}

// ---------------- report task (lowest priority; does all the slow printing) ----------------
static void report_task(void *arg) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // wait for rx_task to hand over a snapshot
        report(snap_rx_window);
    }
}

// ---------------- receive task (owns all RX stats; never prints) ----------------
static void rx_task(void *arg) {
    for (int i = 0; i < (int)N; i++) rx[i].min_gap_us = UINT32_MAX;

    uint32_t rx_frames_window = 0;
    uint64_t next_report = esp_timer_get_time() + 1000000ULL;
    CANframe f;

    while (1) {
        // Blocking receive gives accurate arrival timestamps
        if (can_receive(&f, 50)) {
            rx_frames_window++;
            int i = index_of(f.id);
            if (i >= 0) {
                RxStats *s = &rx[i];
                uint32_t c = get_u32(f.data);
                if (s->seen) {
                    uint32_t gap = (uint32_t)(f.timestamp_us - s->last_us);
                    if (gap < s->min_gap_us) s->min_gap_us = gap;
                    if (gap > s->max_gap_us) s->max_gap_us = gap;
                    // Layer 3 check from the journal: arriving at < half the expected period
                    if (gap < kSchedule[i].period_ms * 500U) s->too_fast++;
                    if (c > s->last_counter + 1) s->lost += c - s->last_counter - 1;
                    // c <= last_counter means the sender rebooted; just resync
                }
                s->seen = true;
                s->last_counter = c;
                s->last_us = f.timestamp_us;
                s->frames++;
            }
        }

        if (esp_timer_get_time() >= next_report) {
            next_report += 1000000ULL;

            // Fast copy for the printer; no printf in this task
            memcpy(snap, rx, sizeof(rx));
            snap_rx_window = rx_frames_window;
            rx_frames_window = 0;

            // Reset per-window stats; keep counter tracking (lost, too_fast are cumulative)
            for (int i = 0; i < (int)N; i++) {
                rx[i].frames = 0;
                rx[i].min_gap_us = UINT32_MAX;
                rx[i].max_gap_us = 0;
            }

            xTaskNotifyGive(report_handle);        // wake the printer
        }
    }
}

// ---------------- entry point ----------------
void app_main(void) {
    printf("CAN bench test; node %d; self_test=%d; stress=%d Hz; %d kbps; open_drain=%d\n",
           NODE, SELF_TEST, STRESS_HZ, CAN_KBPS, OPEN_DRAIN_BUS);

    if (!can_begin()) {
        printf("TWAI init failed; check pins\n");
        return;
    }

    // Priorities: RX highest so timestamps stay accurate; printing lowest.
    // report_task must exist before rx_task so report_handle is valid.
    xTaskCreate(report_task, "can_report", 4096, NULL, 1, &report_handle);
    xTaskCreate(rx_task,     "can_rx",     4096, NULL, 7, NULL);
    xTaskCreate(tx_task,     "can_tx",     4096, NULL, 6, NULL);
    xTaskCreate(alert_task,  "can_alert",  3072, NULL, 5, NULL);
}