#pragma once
#include <string>
#include <unordered_map>


namespace DBWConfig {
    // struct CarProfile {
    //     std::unordered_map<std::string, std::string> signals;
    // };

    struct CanSignal {
        std::string name;
        uint32_t msg_id;
        int start_bit;
        int bit_size;
        int is_intel; // 1: Intel (Little Endian), 0: Motorola (Big Endian)
        char sign;    // '+': Unsigned, '-': Signed
        double factor;
        double offset;
    };

    using CarProfile = std::unordered_map<std::string, CanSignal>;

    // Function definition
    CarProfile init_can_signal(const std::string& car_model);
    
    static constexpr const char* DEFAULT_MODEL    = "mazda_rx8";
    static constexpr const char* DBC_CONF_JSON_PATH = "../../../../config/vehicle_dbc.json";

}