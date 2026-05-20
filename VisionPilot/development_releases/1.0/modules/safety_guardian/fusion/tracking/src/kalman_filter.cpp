#include <tracking/kalman_filter.hpp>

namespace visionpilot::tracking {

KalmanFilter::KalmanFilter(float q_pos, float q_vel, float r_meas)
    : R_(r_meas)
{
    x_[0] = x_[1] = 0.f;
    P_[0][0] = P_[1][1] = 1000.f;
    P_[0][1] = P_[1][0] = 0.f;
    Q_[0][0] = q_pos; Q_[1][1] = q_vel;
    Q_[0][1] = Q_[1][0] = 0.f;
}

void KalmanFilter::initialize(float p0)
{
    x_[0] = p0; x_[1] = 0.f;
    P_[0][0] = 10.f;  P_[0][1] = 0.f;
    P_[1][0] = 0.f;   P_[1][1] = 100.f;
    initialized_ = true;
}

void KalmanFilter::predict(float dt)
{
    if (!initialized_) return;

    // State: x_pred = Φ * x
    float xp0 = x_[0] + dt * x_[1];
    float xp1 = x_[1];

    // Covariance: P_pred = Φ * P * Φ^T + Q
    float phiP[2][2];
    phiP[0][0] = P_[0][0] + dt * P_[1][0];
    phiP[0][1] = P_[0][1] + dt * P_[1][1];
    phiP[1][0] = P_[1][0];
    phiP[1][1] = P_[1][1];

    P_[0][0] = phiP[0][0] + dt * phiP[0][1] + Q_[0][0];
    P_[0][1] = phiP[0][1]                    + Q_[0][1];
    P_[1][0] = phiP[1][0] + dt * phiP[1][1] + Q_[1][0];
    P_[1][1] = phiP[1][1]                    + Q_[1][1];

    x_[0] = xp0;
    x_[1] = xp1;
}

void KalmanFilter::update(float z)
{
    if (!initialized_) { initialize(z); return; }

    // Innovation
    float y  = z - x_[0];
    float S  = P_[0][0] + R_;

    // Kalman gain: K = P * H^T / S,  H = [1, 0]
    float K0 = P_[0][0] / S;
    float K1 = P_[1][0] / S;

    // State update
    x_[0] += K0 * y;
    x_[1] += K1 * y;

    // Covariance update: P = (I - K*H) * P
    float P00 = (1.f - K0) * P_[0][0];
    float P01 = (1.f - K0) * P_[0][1];
    float P10 = -K1 * P_[0][0] + P_[1][0];
    float P11 = -K1 * P_[0][1] + P_[1][1];
    P_[0][0] = P00; P_[0][1] = P01;
    P_[1][0] = P10; P_[1][1] = P11;
}

void KalmanFilter::reset()
{
    initialized_ = false;
    x_[0] = x_[1] = 0.f;
    P_[0][0] = P_[1][1] = 1000.f;
    P_[0][1] = P_[1][0] = 0.f;
}

}  // namespace visionpilot::tracking
