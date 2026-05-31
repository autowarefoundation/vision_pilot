#include <fstream>
#include <iostream>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include "../include/can_config.hpp"

using json = nlohmann::json;

namespace DBWConfig {
    
    CarProfile init_can_signal(const std::string& car_model) {

        const std::string json_path = DBWConfig::DBC_CONF_JSON_PATH; 
        std::ifstream file(json_path);
        
        if (!file.is_open()) {
            throw std::runtime_error("DBWConfig Error: Cannot open config file: " + json_path);
        }

        json dbw_json;
        file >> dbw_json;
        
        // Check if car model exist or not 
        if (!dbw_json.contains(car_model)) {
            throw std::runtime_error("DBWConfig Error: Car model '" + car_model + "' not found in JSON data registry.");
        }

        CarProfile profile;
        const auto& car_data = dbw_json[car_model];

        // Parse DBC data
        for (auto& [generic_key, sig_data] : car_data.items()) {
            CanSignal signal;
            signal.name      = sig_data["name"].get<std::string>();
            signal.msg_id    = sig_data["msg_id"].get<unsigned int>();
            signal.start_bit = sig_data["start_bit"].get<int>();
            signal.bit_size  = sig_data["bit_size"].get<int>();
            signal.is_intel  = sig_data["is_intel"].get<int>();
            signal.sign      = sig_data["sign"].get<std::string>()[0];
            signal.factor    = sig_data["factor"].get<double>();
            signal.offset    = sig_data["offset"].get<double>();

            profile[generic_key] = signal;
        }

        return profile;
    }
}