#pragma once

#include <models/auto_drive.hpp>
#include <models/auto_speed.hpp>
#include <opencv2/core.hpp>

#include <random>
#include <vector>

namespace visionpilot::fusion {

// ─── Output ────────────────────────────────────────────────────────────────────
struct CIPOFusionEstimate {
    bool  valid             = false;

    // Particle-filter fused posterior
    float distance_m        = 0.f;
    float velocity_ms       = 0.f;   // negative = approaching; from particle ensemble
    float distance_stddev_m = 0.f;

    // Raw CIPO distance from AutoSpeed bboxes via homography (no tracking state)
    bool  cipo_raw_found    = false;
    float cipo_raw_dist_m   = 0.f;
    bool  cut_in_detected   = false; // Level 2 is closer than Level 1
};

// ─── LongitudinalFusion ────────────────────────────────────────────────────────
//
//  Per-frame CIPO longitudinal estimation:
//    1. Project AutoSpeed Level 1 / Level 2 bbox bottom-centres through H.
//    2. Particle filter [distance_m, velocity_ms] fuses AutoDrive + CIPO raw.
//       Log-weight accumulation (MRPT style), velocity from weighted particle mean.
//    3. Innovation gate resets the filter on genuine cut-in / cut-out events.
//
class LongitudinalFusion {
public:
    struct Config {
        int   n_particles          = 500;
        float d_max_m              = 150.f;
        float dt_s                 = 0.10f;   // nominal dt; overridden per-call
        // Keep small (MRPT uses 0.03 m). Large values cause particle drift → bad velocity.
        float process_noise_dist_m = 0.05f;
        float process_noise_vel_ms = 0.20f;   // matches MRPT TRANSITION_MODEL_STD_VXY
        float autodrive_noise_m    = 15.f;
        float cipo_noise_m         = 5.f;
        // Reinitialise filter when a reliable measurement jumps this far from
        // the particle cloud — handles genuine cut-in / cut-out events.
        float reset_gate_m         = 25.f;
        bool  debug                = false;
    };

    LongitudinalFusion();
    explicit LongitudinalFusion(Config cfg);

    CIPOFusionEstimate update(
        const models::AutoDriveOutput& autodrive,
        const models::AutoSpeedOutput& autospeed,
        const cv::Mat& preprocessed_frame,
        float dt_s = 0.f);

    void reset();
    const Config& config() const { return cfg_; }

    // Override the internal H matrix used to project AutoSpeed bboxes to world
    // space.  Call once when AutoSpeed runs on a non-BEV image.
    void set_H(const cv::Mat& H) { H_ = H.clone(); }

private:
    struct Particle { float distance_m, velocity_ms, log_w; };
    struct Meas     { float distance_m = 0.f; float stddev_m = 15.f; bool valid = false; };

    struct CIPOSelection { Meas meas; bool cut_in = false; };
    CIPOSelection select_cipo(const std::vector<models::Detection>& dets) const;
    static float project_dist(const cv::Mat& H, float ux, float uy);

    void  init_from(float dist_m, float stddev_m);
    void  predict(float dt_s);
    void  weight_update(const Meas& ad, const Meas& cipo_raw);
    std::vector<float> linear_weights() const;
    float effective_n() const;
    void  resample();
    static float gaussian_loglik(float z, float mean, float sigma);

    Config cfg_;
    std::vector<Particle> particles_;
    bool   initialised_ = false;
    std::mt19937 rng_;
    // DO NOT MODIFY! VisionPilot model-view homography (1024x512 pixel -> world). Zenseact Open Dataset
    cv::Mat H_ = (cv::Mat_<double>(3, 3) <<
                       0.00209514907, -0.000941721466, -9.24906396,
                       0.00662758637, -0.000352940531, -3.33396502,
                       0.000120077371, -0.00411343505, 1.0
         );
};

}  // namespace visionpilot::fusion
