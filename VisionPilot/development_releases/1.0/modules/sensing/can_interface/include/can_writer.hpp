#ifndef VISIONPILOT_CAN_WRITER_HPP
#define VISIONPILOT_CAN_WRITER_HPP

#include <string>
#include <map>
#include <unordered_map>
#include <vector>
#include <mutex>
#include "../libpanda/include/panda.h"
#include "can_config.hpp"

namespace can_writer {

    struct SignalDescriptor {
        unsigned int message_id = 0;
        unsigned int start_bit = 0;
        unsigned int bit_length = 0;
        bool is_signed = false;
        bool big_endian = false;
        double factor = 1.0;
        double offset = 0.0;
        std::string dbc_name;
    };

    class CanWriter {
    public:
        static constexpr double MAX_STEERING_CMD = 450.0;
        static constexpr double MIN_STEERING_CMD = -450.0;

        static constexpr double MAX_STEERING_ANGLE = 540.0;
        static constexpr double MIN_STEERING_ANGLE = -540.0;
        static constexpr double MAX_VEHICLE_SPEED = 240.0;
        static constexpr double MIN_VEHICLE_SPEED = 0.0;
        static constexpr double MAX_ACCEL_POS = 100.0;
        static constexpr double MIN_ACCEL_POS = 0.0;
        static constexpr double MAX_WHEEL_SPEED = 240.0;
        static constexpr double MIN_WHEEL_SPEED = 0.0;

        CanWriter(const std::string& car_model = DBWConfig::DEFAULT_MODEL);
        ~CanWriter();

        bool send_signals_by_id(const std::map<std::string, double>& signals);
        bool parse_can_frame(const Panda::CanFrame& frame, std::map<std::string, double>& extracted_signals) const;
        
        bool send_command(double steering_angle,
                          double accelerator_pos,
                          bool brake,
                          double vehicle_speed,
                          double avg_wheel_speed);

        bool send_safe_stop();
        std::map<std::string, int> get_stats() const;
        void close();

    private:
        bool _init_signal_mapping();
        bool _parse_dbc_file(const std::string& dbc_path);
        bool _validate_command(double steering_angle, double accelerator_pos) const;
        double _clamp(double value, double min_val, double max_val) const;

        bool _pack_frame(Panda::CanFrame& frame,
                        unsigned int message_id,
                        const std::map<std::string, double>& signals_for_frame);

        void _encode_signal(unsigned char* data, const std::string& signal_name, double value) const;
        double _decode_signal(const unsigned char* data, const std::string& signal_name) const;

        void _set_bit(unsigned char* data, unsigned int start_bit, unsigned int bit_length, unsigned long long value, bool big_endian) const;
        unsigned long long _get_bit(const unsigned char* data, unsigned int start_bit, unsigned int bit_length, bool big_endian) const;

        bool _send_frame_with_retry(const Panda::CanFrame& frame);

        std::string car_model = DBWConfig::DEFAULT_MODEL;
        // std::string dbc_file  = DBWConfig::DEFAULT_DBC_PATH;

        Panda::Usb* panda;
        unsigned char can_version;
        int command_count;
        int command_errors;
        bool is_active;

        std::mutex panda_mutex;
        std::unordered_map<std::string, SignalDescriptor> signal_descriptors;
        std::unordered_map<std::string, std::string> reverse_signal_mapping;
        std::unordered_map<unsigned int, std::vector<unsigned char>> message_state_buffers;
    };
}

#endif