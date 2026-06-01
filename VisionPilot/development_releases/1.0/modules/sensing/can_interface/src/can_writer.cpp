#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <cstring>
#include <algorithm>

#include <sensing/can_interface/include/can_config.hpp>
#include <sensing/can_interface/include/can_writer.hpp>


namespace can_writer {
        /**
     * @brief Constructor: Initializes vehicle profile, maps signals, 
     * registers the CAN Writer, and init the Panda hardware.
     */
    CanWriter::CanWriter(const std::string& car_model)
        : car_model(car_model.empty() ? DBWConfig::DEFAULT_MODEL : car_model),
          is_active(true), command_count(0), command_errors(0)
    
    {   
        std::cout << " CAN Writer Starts Parsing CAN data for car model: " << car_model << std::endl;
        if (!_init_signal_mapping()) {
            throw std::runtime_error("CAN Writer DBC Init Failure: check configurations for car model.");
        }
        
        // Init Panda Hardware
        try {
            panda = new Panda::Usb(Panda::MODE_ASYNCHRONOUS);
            if (!panda) {
                throw std::runtime_error("Failed to allocate Panda USB object memory");
            }
            panda->initialize();
            panda->setSafetyMode(SAFETY_ALLOUTPUT, 1);
            
            unsigned char health_ver = 0;
            unsigned char can_ver = 0;
            panda->getHealthAndCanVersions(&health_ver, &can_ver);
            can_version = can_ver;
        } catch (const std::exception& e) {
            std::cerr << "Critical: Hardware backend integration failed: " << e.what() << std::endl;
            close();
            throw;
        }
    }

    CanWriter::~CanWriter() {
        close();
    }

    /**
     * @brief Loads the car configuration and signal description maps.
    */
    bool CanWriter::_init_signal_mapping() {
        try {
            const auto profile = DBWConfig::init_can_signal(car_model);
            // Init Struct to store CAN parse
            signal_descriptors.clear();
            reverse_signal_mapping.clear();

            // Parse CAN Data Storage
            for (const auto& entry : profile) {
                const std::string& generic_key = entry.first;
                const auto& sig = entry.second;

                SignalDescriptor desc;
                desc.message_id = sig.msg_id;
                desc.start_bit = static_cast<unsigned int>(sig.start_bit);
                desc.bit_length = static_cast<unsigned int>(sig.bit_size);
                desc.big_endian = (sig.is_intel == 0);
                desc.is_signed = (sig.sign == '-');
                desc.factor = sig.factor;
                desc.offset = sig.offset;
                desc.dbc_name = sig.name;

                signal_descriptors[generic_key] = desc;
                reverse_signal_mapping[sig.name] = generic_key;
            }

            return !signal_descriptors.empty();
        } 
        catch (const std::exception& e) {
            return false;
        }
    }
    /**
     * @brief Writes a raw integer value into a specific bitfield within the 8-byte CAN data array.
     * Handles both Little Endian (Intel) and Big Endian (Motorola) alignment automatically.
     */
    void CanWriter::_set_bit(unsigned char* data, unsigned int start_bit, unsigned int bit_length, unsigned long long value, bool big_endian) const {
        if (bit_length == 0 || bit_length > 64 || !data) return;

        if (!big_endian) {
            for (unsigned int bit_index = 0; bit_index < bit_length; ++bit_index) {
                unsigned int absolute_bit = start_bit + bit_index;
                unsigned int byte_index = absolute_bit / 8;
                unsigned int bit_in_byte = absolute_bit % 8;
                
                if ((value >> bit_index) & 1) {
                    data[byte_index] |= (1u << bit_in_byte);
                } else {
                    data[byte_index] &= ~(1u << bit_in_byte);
                }
            }
        } else {
            unsigned int current_bit = start_bit;
            for (unsigned int bit_index = 0; bit_index < bit_length; ++bit_index) {
                unsigned int byte_index = current_bit / 8;
                unsigned int bit_in_byte = 7u - (current_bit % 8u);
                
                if ((value >> (bit_length - 1 - bit_index)) & 1) {
                    data[byte_index] |= (1u << bit_in_byte);
                } else {
                    data[byte_index] &= ~(1u << bit_in_byte);
                }

                if (current_bit % 8 == 0) {
                    current_bit += 15;
                } else {
                    --current_bit;
                }
            }
        }
    }
    
    /**
     * @brief Extracts a raw integer value from a specific bitfield within the 8-byte CAN data array.
     * Reconstructs the original bits based on Intel or Motorola byte-ordering.
     */
    unsigned long long CanWriter::_get_bit(const unsigned char* data, unsigned int start_bit, unsigned int bit_length, bool big_endian) const {
        if (bit_length == 0 || bit_length > 64 || !data) return 0;
        unsigned long long value = 0;

        if (!big_endian) {
            for (unsigned int bit_index = 0; bit_index < bit_length; ++bit_index) {
                unsigned int absolute_bit = start_bit + bit_index;
                unsigned int byte_index = absolute_bit / 8;
                unsigned int bit_in_byte = absolute_bit % 8;
                
                if ((data[byte_index] >> bit_in_byte) & 1) {
                    value |= (1ULL << bit_index);
                }
            }
        } else {
            unsigned int current_bit = start_bit;
            for (unsigned int bit_index = 0; bit_index < bit_length; ++bit_index) {
                unsigned int byte_index = current_bit / 8;
                unsigned int bit_in_byte = 7u - (current_bit % 8u);
                
                if ((data[byte_index] >> bit_in_byte) & 1) {
                    value |= (1ULL << (bit_length - 1 - bit_index));
                }

                if (current_bit % 8 == 0) {
                    current_bit += 15;
                } else {
                    --current_bit;
                }
            }
        }
        return value;
    }

    /**
     * @brief Converts a human-readable physical value (e.g., Speed) into raw bits and inserts it into the CAN frame.
     * Formula: Raw Value = (Physical Value - Offset) / Factor
     */
    void CanWriter::_encode_signal(unsigned char* data, const std::string& signal_name, double value) const {
        auto it = signal_descriptors.find(signal_name);
        if (it == signal_descriptors.end()) return;

        const SignalDescriptor& desc = it->second;
        double scaled_value = (value - desc.offset) / desc.factor;
        unsigned long long raw_value = static_cast<unsigned long long>(std::round(scaled_value));

        if (desc.is_signed && desc.bit_length > 0) {
            long long signed_raw = static_cast<long long>(raw_value);
            unsigned long long mask = (1ULL << desc.bit_length) - 1ULL;
            raw_value = static_cast<unsigned long long>(signed_raw) & mask;
        }

        // CAN Format
        _set_bit(data, desc.start_bit, desc.bit_length, raw_value, desc.big_endian);
    }
    
    /**
     * @brief Extracts raw bits from the CAN frame and converts them into a human-readable physical value.
     * Formula: Physical Value = (Raw Value * Factor) + Offset
     */
    double CanWriter::_decode_signal(const unsigned char* data, const std::string& signal_name) const {
        auto it = signal_descriptors.find(signal_name);
        if (it == signal_descriptors.end()) return 0.0;

        const SignalDescriptor& desc = it->second;
        unsigned long long raw_value = _get_bit(data, desc.start_bit, desc.bit_length, desc.big_endian);

        if (desc.is_signed && desc.bit_length > 0) {
            unsigned long long sign_bit = 1ULL << (desc.bit_length - 1);
            if (raw_value & sign_bit) {
                long long mask = (1ULL << desc.bit_length) - 1;
                raw_value = raw_value | ~mask;
            }
        }

        return (static_cast<double>(static_cast<long long>(raw_value)) * desc.factor) + desc.offset;
    }

    /**
     * @brief Merges new signal values into the message's historical byte buffer 
     * and packs them into a 8-byte CAN frame.
     */    
    bool CanWriter::_pack_frame(Panda::CanFrame& frame, unsigned int message_id, const std::map<std::string, double>& signals_for_frame) {
        frame.messageID = message_id;
        frame.bus = 0;
        frame.dataLength = 8;

        if (message_state_buffers.find(message_id) == message_state_buffers.end()) {
            message_state_buffers[message_id] = std::vector<unsigned char>(8, 0);
        }
        std::memcpy(frame.data, message_state_buffers[message_id].data(), 8);

        for (const auto& signal_pair : signals_for_frame) {
            _encode_signal(frame.data, signal_pair.first, signal_pair.second);
        }

        std::memcpy(message_state_buffers[message_id].data(), frame.data, 8);
        return true;
    }

    /**
     * @brief Converts a CAN frame into raw bytes and transmits it via Panda, 
     * retrying up to 3 times (5ms intervals) if it fails.
     */
    bool CanWriter::_send_frame_with_retry(const Panda::CanFrame& frame) {
        if (!panda) return false;
        
        // Prepare data to send
        unsigned char buffer[64] = {0};
        Panda::CanFrame mutable_frame = frame;
        int buffer_len = Panda::canFrameToBuffer(mutable_frame, buffer, static_cast<int>(can_version));
        if (buffer_len <= 0) return false;

        // Start send CAN
        for (int attempt = 0; attempt < 3; ++attempt) {
            if (panda->sendCanData(buffer, buffer_len) == 0) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return false;
    }

    /**
     * @brief Groups multiple input signals by their target Message IDs, 
     * locks the hardware interface, and safely transmits the packed frames.
     */
    bool CanWriter::send_signals_by_id(const std::map<std::string, double>& signals) {
        if (signals.empty() || !panda || !is_active) return false;

        // Prepare Data to Send
        std::map<unsigned int, std::map<std::string, double>> dynamic_frames;

        for (const auto& [signal_name, signal_value] : signals) {
            auto desc_it = signal_descriptors.find(signal_name);
            if (desc_it == signal_descriptors.end()) {
                std::cerr << "Warning: System signal '" << signal_name << "' isn't mapped inside the loaded configurations." << std::endl;
                continue;
            }
            unsigned int msg_id = desc_it->second.message_id;
            dynamic_frames[msg_id][signal_name] = signal_value;
        }

        // Open Thread
        bool all_success = true;
        std::lock_guard<std::mutex> lock(panda_mutex);
        
        // Start send CAN
        for (const auto& [message_id, signals_for_frame] : dynamic_frames) {
            Panda::CanFrame frame;
            if (!_pack_frame(frame, message_id, signals_for_frame)) {
                command_errors++;
                all_success = false;
                continue;
            }

            if (_send_frame_with_retry(frame)) {
                command_count++;
            } else {
                command_errors++;
                all_success = false;
            }
        }

        return all_success;
    }

    /**
     * @brief Decodes an incoming CAN frame by matching its ID with configured signals 
     * and extracts their raw data into physical values.
     */
    bool CanWriter::parse_can_frame(const Panda::CanFrame& frame, std::map<std::string, double>& extracted_signals) const {
        bool signal_found = false;
        for (const auto& [canonical_name, descriptor] : signal_descriptors) {
            if (descriptor.message_id == frame.messageID) {
                extracted_signals[canonical_name] = _decode_signal(frame.data, canonical_name);
                signal_found = true;
            }
        }
        return signal_found;
    }

    /**
     * @brief Primary API to control the vehicle. Validates, clamps inputs 
     * (Steering, Accel, Brake, Speed) to safe ranges, and triggers transmission.
     */
    bool CanWriter::send_command(double steering_angle, double accelerator_pos, bool brake,
                                  double vehicle_speed, double avg_wheel_speed) {
        if (!_validate_command(steering_angle, accelerator_pos)) {
            command_errors++;
            return false;
        }

        std::map<std::string, double> signals;
        
        signals["steering_angle"] = _clamp(steering_angle, MIN_STEERING_ANGLE, MAX_STEERING_ANGLE);
        signals["vehicle_speed"] = _clamp(vehicle_speed, MIN_VEHICLE_SPEED, MAX_VEHICLE_SPEED);
        signals["accelerator_pos"] = _clamp(accelerator_pos, MIN_ACCEL_POS, MAX_ACCEL_POS);
        signals["brake"] = brake ? 1.0 : 0.0;
        signals["avg_wheel_speed"] = _clamp(avg_wheel_speed, MIN_WHEEL_SPEED, MAX_WHEEL_SPEED);

        return send_signals_by_id(signals);
    }

    /**
     * @brief Emergency safety. 
     */
    bool CanWriter::send_safe_stop() {
        std::map<std::string, double> safe_signals;
        safe_signals["vehicle_speed"] = 0.0;
        safe_signals["steering_cmd"] = 0.0;
        safe_signals["accelerator_pos"] = 0.0;
        safe_signals["brake_cmd"] = 1.0;
        safe_signals["avg_wheel_speed"] = 0.0;
        return send_signals_by_id(safe_signals);
    }

    /**
     * @brief get_stats: Returns historical telemetry counters for sent frames and errors.
     * @brief close: Deactivates the system loop and safely deallocates the Panda hardware pointer.
     * @brief _validate_command: Verifies that input values are legitimate, finite numbers (not NaN/Inf).
     * @brief _clamp: Constrains an input value strictly between predefined minimum and maximum boundaries.
     */
    std::map<std::string, int> CanWriter::get_stats() const {
        return {{"command_count", command_count}, {"command_errors", command_errors}};
    }

    void CanWriter::close() {
        is_active = false;
        if (panda) {
            delete panda;
            panda = nullptr;
        }
    }

    bool CanWriter::_validate_command(double steering_angle, double accelerator_pos) const {
        return std::isfinite(steering_angle) && std::isfinite(accelerator_pos);
    }

    double CanWriter::_clamp(double value, double min_val, double max_val) const {
        return std::max(min_val, std::min(max_val, value));
    }
}