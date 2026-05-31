#ifndef INC_VISIONPILOT_CAN_READER_HPP
#define INC_VISIONPILOT_CAN_READER_HPP

#include <string>
#include <map>
#include <vector>
#include <mutex>
#include "../libpanda/include/panda.h"
#include "can_config.hpp"

namespace can_reader {

    struct VehicleState {
        double steering_angle;      // Degrees
        double vehicle_speed;       // km/h
        double accelerator_pos;     // %
        bool brake;                 // ON/OFF
        double avg_wheel_speed;     // km/h
        double timestamp;
        bool valid;
        std::string error;
        std::string car_model;

        VehicleState() : steering_angle(0.0), vehicle_speed(0.0),
                         accelerator_pos(0.0), brake(false), avg_wheel_speed(0.0),
                         timestamp(0.0), valid(false), error(""), car_model("") {}
    };

    struct SafetyState {
        double last_steering_angle;
        double last_vehicle_speed;
        double steering_rate_limit;     // Max change per loop cycle
        double speed_rate_limit;        // Max change per loop cycle
        
        SafetyState() : last_steering_angle(0.0), last_vehicle_speed(0.0),
                        steering_rate_limit(25.0), speed_rate_limit(5.0) {}
    };

    struct SignalDescriptor {
        unsigned int message_id;
        unsigned int start_bit;
        unsigned int bit_length;
        bool is_signed;
        bool big_endian;
        double factor;
        double offset;
        std::string dbc_name;

        // Explicit constructor to bypass aggregate initialization rules in older C++
        SignalDescriptor(unsigned int id = 0, unsigned int sb = 0, unsigned int bl = 0,
                         bool iss = false, bool be = false, double fac = 1.0, 
                         double off = 0.0, const std::string& name = "")
            : message_id(id), start_bit(sb), bit_length(bl), is_signed(iss),
              big_endian(be), factor(fac), offset(off), dbc_name(name) {}
    };

    class CanReader : public Panda::CanListener {
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


        CanReader(const std::string& car_model = DBWConfig::DEFAULT_MODEL);
        ~CanReader();

        VehicleState read_once();
        VehicleState get_latest_state() const;
        std::map<std::string, double> get_signal_values() const;
        void close();

    //private:
        bool _init_signal_mapping();
        bool _parse_dbc_file(const std::string& dbc_path);
        void _process_frame(const Panda::CanFrame& frame);
        
        unsigned long long _extract_bits(const unsigned char* data, unsigned int start_bit, 
                                         unsigned int length, bool big_endian) const;
        double _decode_signal(const Panda::CanFrame& frame, const SignalDescriptor& descriptor) const;
        
        void newDataNotification(Panda::CanFrame* canFrame) override; 

        std::string car_model = DBWConfig::DEFAULT_MODEL;
        // std::string dbc_file  = DBWConfig::DEFAULT_DBC_PATH;

        Panda::Handler pandaHandler;
        std::vector<Panda::CanFrame> pending_frames;
        
        // Mappings matching the source file execution expectations
        std::map<std::string, std::string> reverse_signal_mapping; 
        std::map<std::string, SignalDescriptor> signal_descriptors;
        std::map<std::string, double> global_signal_registry;

        VehicleState accumulated_state;
        SafetyState safety_state;
        int read_count;
        int read_errors;

        mutable std::mutex data_mutex;
        mutable std::mutex state_mutex;
    };
}

#endif