#pragma once

namespace visionpilot::tracking {

// 1-D Kalman filter — state: [position, velocity]
// Measurement model: position-only (H = [1, 0])
// Dynamics: constant-velocity (Φ = [[1, T],[0, 1]])
class KalmanFilter {
public:
    KalmanFilter(float q_pos = 1.0f, float q_vel = 1.0f, float r_meas = 1.0f);

    void initialize(float initial_position);
    void predict(float dt);
    void update(float measured_position);
    void reset();

    float position() const { return x_[0]; }
    float velocity() const { return x_[1]; }
    bool  is_initialized() const { return initialized_; }

private:
    float x_[2];       // [position, velocity]
    float P_[2][2];    // covariance
    float Q_[2][2];    // process noise
    float R_;          // measurement noise
    bool  initialized_ = false;
};

}  // namespace visionpilot::tracking
