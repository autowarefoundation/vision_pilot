#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

#include <sensing/can_interface/include/can_config.hpp>
#include <sensing/can_interface/include/can_reader.hpp>

namespace can_reader {
    /**
     * @brief Constructor: Initializes vehicle profile, maps signals, 
     * registers the CAN observer, and init the Panda hardware.
     */
    CanReader::CanReader(const std::string& car_model)
        : car_model(car_model.empty() ? DBWConfig::DEFAULT_MODEL : car_model),
          read_count(0), read_errors(0)
    {
        std::cout << " CAN Reader Starts Parsing CAN data for car model: " << car_model << std::endl;
        if (!_init_signal_mapping()) {
            throw std::runtime_error("CAN Reader DBC Init Failure: check configurations for car model.");
        }else{
            std::cout << "CanReader Init successful for car model: " << car_model << std::endl;
        }

        // Init Panda Hardware
        pandaHandler.addCanObserver(*this);                                 
        pandaHandler.initialize();
        pandaHandler.getUsb().setOperatingMode(Panda::MODE_ASYNCHRONOUS);
        pandaHandler.getCan().saveToCsvFile((this->car_model + "_rx.csv").c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    CanReader::~CanReader() {
        close();
    }

    /**
     * @brief Loads the car configuration and signal description maps.
     */
    bool CanReader::_init_signal_mapping() {
        try {
            const auto profile = DBWConfig::init_can_signal(car_model);
            // Init Struct to store CAN parse
            signal_descriptors.clear();
            reverse_signal_mapping.clear();
            
            // Parse CAN Data Storage
            for (const auto& entry : profile) {
                const std::string& generic_key = entry.first;
                const auto& sig = entry.second;

                SignalDescriptor desc{
                    sig.msg_id,
                    static_cast<unsigned int>(sig.start_bit),
                    static_cast<unsigned int>(sig.bit_size),
                    sig.sign == '-',
                    sig.is_intel == 1,
                    sig.factor,
                    sig.offset,
                    sig.name
                };

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
     * @brief Low-level bit-banging utility to extract an unaligned sequence 
     * of bits across byte boundaries supporting Intel/Motorola layouts.
     */
    unsigned long long CanReader::_extract_bits(const unsigned char* data, unsigned int start_bit, unsigned int length, bool big_endian) const {
        if (length == 0 || length > 64 || !data) return 0;
        unsigned long long value = 0;

        if (!big_endian) {
            for (unsigned int bit_index = 0; bit_index < length; ++bit_index) {
                unsigned int absolute_bit = start_bit + bit_index;
                if (data[absolute_bit / 8] & (1u << (absolute_bit % 8))) {
                    value |= (1ULL << bit_index);
                }
            }
        } else {
            unsigned int current_bit = start_bit;
            for (unsigned int bit_index = 0; bit_index < length; ++bit_index) {
                if (data[current_bit / 8] & (1u << (7u - (current_bit % 8u)))) {
                    value |= (1ULL << (length - 1 - bit_index));
                }
                current_bit = (current_bit % 8 == 0) ? (current_bit + 15) : (current_bit - 1);
            }
        }
        return value;
    }

    /**
     * @brief Performs raw bit extraction, applies sign-extension for signed data, 
     * and converts the raw data into physical data. Physical Value = (Raw Value * Factor) + Offset
     */
    double CanReader::_decode_signal(const Panda::CanFrame& frame, const SignalDescriptor& descriptor) const {
        unsigned long long raw = _extract_bits(frame.data, descriptor.start_bit, descriptor.bit_length, descriptor.big_endian);
        if (descriptor.is_signed && descriptor.bit_length > 0) {
            unsigned long long sign_bit = 1ULL << (descriptor.bit_length - 1);
            if (raw & sign_bit) {
                unsigned long long mask = ((1ULL << descriptor.bit_length) - 1ULL);
                return static_cast<double>(static_cast<long long>(raw | (~mask))) * descriptor.factor + descriptor.offset;
            }
        }
        return static_cast<double>(raw) * descriptor.factor + descriptor.offset;
    }

    /**
     * @brief Iterates through registered descriptors to match the incoming 
     * message ID and saves decoded engineering units to the registry.
     */
    void CanReader::_process_frame(const Panda::CanFrame& frame) {
        for (const auto& [canonical_name, desc] : signal_descriptors) {
            if (desc.message_id == frame.messageID) {
                global_signal_registry[canonical_name] = _decode_signal(frame, desc);
            }
        }
    }

    /**
     * @brief Core cyclic execution pipeline. Flushes queued CAN frames, 
     * triggers decoding, computes wheel averages, and evaluates safety limits.
     */
    VehicleState CanReader::read_once() {
        // Thread Safe Data Swap
        std::vector<Panda::CanFrame> local_frames;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            if (!pending_frames.empty()) {
                local_frames.swap(pending_frames);
            }
        }

        std::lock_guard<std::mutex> state_lock(state_mutex);
        accumulated_state.timestamp = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        accumulated_state.car_model = car_model;

        // If CAN Frame detected
        if (local_frames.empty()) {
            return accumulated_state; 
        }

        // Process new CAN frame
        for (const auto& frame : local_frames) {
            _process_frame(frame);
        }

        if (global_signal_registry.count("steering_angle"))   accumulated_state.steering_angle = global_signal_registry["steering_angle"];
        if (global_signal_registry.count("vehicle_speed"))    accumulated_state.vehicle_speed = global_signal_registry["vehicle_speed"];
        if (global_signal_registry.count("accelerator_pos"))  accumulated_state.accelerator_pos = global_signal_registry["accelerator_pos"];
        if (global_signal_registry.count("brake"))            accumulated_state.brake = (global_signal_registry["brake"] != 0.0);

        double wfl = global_signal_registry.count("wheel_speed_fl") ? global_signal_registry["wheel_speed_fl"] : 0.0;
        double wfr = global_signal_registry.count("wheel_speed_fr") ? global_signal_registry["wheel_speed_fr"] : 0.0;
        double wrl = global_signal_registry.count("wheel_speed_rl") ? global_signal_registry["wheel_speed_rl"] : 0.0;
        double wrr = global_signal_registry.count("wheel_speed_rr") ? global_signal_registry["wheel_speed_rr"] : 0.0;
        
        if (global_signal_registry.count("avg_wheel_speed")) {
            accumulated_state.avg_wheel_speed = global_signal_registry["avg_wheel_speed"];
        } else if (wfl + wfr + wrl + wrr > 0.0) {
            accumulated_state.avg_wheel_speed = (wfl + wfr + wrl + wrr) / 4.0;
        }

        // Checking for safety
        bool bounds_ok = (accumulated_state.steering_angle >= MIN_STEERING_ANGLE && accumulated_state.steering_angle <= MAX_STEERING_ANGLE) &&
                         (accumulated_state.vehicle_speed >= MIN_VEHICLE_SPEED && accumulated_state.vehicle_speed <= MAX_VEHICLE_SPEED) &&
                         (accumulated_state.accelerator_pos >= MIN_ACCEL_POS && accumulated_state.accelerator_pos <= MAX_ACCEL_POS);

        double steer_delta = std::abs(accumulated_state.steering_angle - safety_state.last_steering_angle);
        double speed_delta = std::abs(accumulated_state.vehicle_speed - safety_state.last_vehicle_speed);

        bool safety_ok = (read_count == 0) || (steer_delta <= safety_state.steering_rate_limit && speed_delta <= safety_state.speed_rate_limit);

        if (bounds_ok && safety_ok) {
            accumulated_state.valid = true;
            accumulated_state.error.clear();
            safety_state.last_steering_angle = accumulated_state.steering_angle;
            safety_state.last_vehicle_speed = accumulated_state.vehicle_speed;
            read_count++;
        } else {
            accumulated_state.valid = false;
            accumulated_state.error = !bounds_ok ? "Signal Out of Bounds" : "Dynamic Rate Delta Fault";
            read_errors++;
        }

        return accumulated_state;
    }
    
    /**
     * @brief Thread-safe getter to fetch the latest fully verified vehicle state.
     */
    VehicleState CanReader::get_latest_state() const {
        std::lock_guard<std::mutex> lock(state_mutex);
        return accumulated_state;
    }
    
    /**
     * @brief Thread-safe getter to dump the complete key-value map of decoded metrics.
     */
    std::map<std::string, double> CanReader::get_signal_values() const {
        std::lock_guard<std::mutex> lock(data_mutex);
        return global_signal_registry;
    }

    /**
     * @brief Event callback listener triggered asynchronously by libpanda 
     * hardware thread whenever a new CAN frame arrives.
     */
    void CanReader::newDataNotification(Panda::CanFrame* canFrame) {
        if (!canFrame) return;
        std::lock_guard<std::mutex> lock(data_mutex);
        pending_frames.push_back(*canFrame);
        if (pending_frames.size() > 1000) { 
            pending_frames.erase(pending_frames.begin(), pending_frames.begin() + 200);
        }
    }

    /**
     * @brief Halts the hardware communications interface and flushes temporary buffers.
     */
    void CanReader::close() {
        pandaHandler.stop();
        std::lock_guard<std::mutex> lock(data_mutex);
        pending_frames.clear();
    }
}

