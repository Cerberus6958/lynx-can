#include <vector>
#include <cstdint>

struct CANframe {
    uint32_t id;
    std::vector<uint8_t> data;
};