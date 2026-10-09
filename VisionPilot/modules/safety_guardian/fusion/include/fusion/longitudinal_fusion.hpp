#pragma once

#include <models/auto_drive.hpp>
#include <models/auto_speed.hpp>
#include <opencv2/core.hpp>

#include <random>
#include <vector>
#include <common/types.hpp>

namespace visionpilot::fusion {

struct PathPoly {  // y = a·x² + b·x + c, same frame as radar
    bool  valid = false;
    float a = 0.f, b = 0.f, c = 0.f;
    float x_max_m = 0.f;
};

// ─── Output ────────────────────────────────────────────────────────────────────
enum class RadarHit { None, Fov, Path, L3 };

// Snapshot for --debug-viz BEV. in_match / match_* are the association result
// itself, so the panel never has to re-derive (and disagree about) the answer.
struct RadarAssocDebug {
    bool enabled = false;
    std::vector<RadarPoint> points;
    std::vector<uint8_t> in_match;   // per point: belongs to the selected cluster
    PathPoly path;
    bool  fov_valid      = false;
    float fov_az_rad     = 0.f;
    RadarHit hit         = RadarHit::None;
    int   match_i        = -1;
    float match_range_m  = 0.f;      // exactly what fusion reported
    float match_rate_ms  = 0.f;
};

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

    // Per-source longitudinal measurements (debug + particle-filter inputs)
    bool  ad_meas_valid     = false;
    float ad_dist_m         = 0.f;
    bool  as_h_valid        = false;
    float as_h_dist_m       = 0.f;
    bool  radar_meas_valid  = false;
    float radar_dist_m      = 0.f;
    float radar_vel_ms      = 0.f;
    int   radar_scenario    = 0;  // 0 none, 1 L1/L2 present, 3 in-path (no CIPO box)

    RadarAssocDebug radar;
};

// ─── LongitudinalFusion ────────────────────────────────────────────────────────
//
//  Per-frame CIPO longitudinal estimation:
//    1. Project AutoSpeed Level 1 / Level 2 bbox bottom-centres through H.
//    2. Associate an in-path radar cluster when radar is enabled.
//    3. Particle filter [distance_m, velocity_ms]:
//         • radar match  → track radar only (camera does not reweight)
//         • radar miss   → AS L1/L2 (+ AD if present); else AD if flag ≥ 0.40
//         • source switch radar↔camera → reset and re-init
//
class LongitudinalFusion {
public:
    struct Config {
        int   n_particles          = 500;
        float d_max_m              = 150.f;
        float dt_s                 = 0.10f;   // nominal dt; overridden per-call
        // Keep small (MRPT uses 0.03 m). Large values cause particle drift → bad velocity.
        float process_noise_dist_m  = 2.0f;
        float process_noise_vel_ms  = 0.50f;
        // AD noise is scaled by confidence: tight when flag_prob≈1, loose when flag_prob≈threshold.
        // at p=1.00 → stddev = autodrive_noise_min_m  (AD dominates)
        // at p=0.40 → stddev = autodrive_noise_m       (CIPO can dominate)
        float autodrive_noise_min_m = 1.5f;   // noise floor when AD is fully confident
        float autodrive_noise_m     = 8.f;    // noise ceiling at minimum AD confidence
        float cipo_noise_m          = 3.f;    // CIPO bbox / radar range noise
        float cipo_vel_noise_ms     = 1.5f;   // radar range-rate noise
        // Range at which cipo_noise_m describes the bbox homography. Its error
        // grows with distance², so the noise is scaled by (d / this)².
        float homography_ref_range_m = 30.f;
        // Reinitialise filter when a measurement jumps this far from the
        // particle cloud (genuine cut-in / cut-out only).
        float reset_gate_m          = 25.f;
        bool  debug                = false;

        bool  radar_enabled        = false;
        float radar_hfov_deg       = 50.f;   // AutoSpeed crop HFOV (not ZOD's ~120° full cam)
        float radar_lat_buffer_m   = 0.5f;
        // Half a lane either side of the fused path. Search along the poly
        // out to radar_max_range_m, not only as far as the waypoint fit.
        float radar_path_buffer_m  = 1.8f;
        float radar_max_range_m    = 150.f;
        // T_sensor→ego 4×4. Ego = ISO-8855 (X fwd, Y left, Z up). Same as ZOD.
        // Default cam: OpenCV optical (X right, Y down, Z fwd) → ego. Default radar: identity.
        cv::Matx44d cam_T{0, 0, 1, 0,
                         -1, 0, 0, 0,
                          0,-1, 0, 0,
                          0, 0, 0, 1};
        cv::Matx44d radar_T{1, 0, 0, 0,
                            0, 1, 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1};
    };

    LongitudinalFusion();
    explicit LongitudinalFusion(Config cfg);

    CIPOFusionEstimate update(
        const models::AutoDriveOutput& autodrive,
        const models::AutoSpeedOutput& autospeed,
        const cv::Mat& preprocessed_frame,
        float dt_s = 0.f,
        const std::vector<RadarPoint>* radar = nullptr,
        const PathPoly* path = nullptr,
        const float* ego_speed_ms = nullptr);

    void reset();
    const Config& config() const { return cfg_; }

    // Override the internal H matrix used to project AutoSpeed bboxes to world
    // space.  Call once when AutoSpeed runs on a non-BEV image.
    void set_H(const cv::Mat& H) { H_ = H.clone(); }

private:
    struct Particle { float distance_m, velocity_ms, log_w; };
    struct Meas {
        float distance_m = 0.f;
        float velocity_ms = 0.f;
        float stddev_m = 15.f;
        float stddev_v = 1.5f;
        bool  valid = false;
        bool  has_velocity = false;
    };

    struct CIPOSelection {
        Meas meas;
        bool cut_in = false;
        bool from_path = false;
        bool fov_valid = false;
        float fov_az_rad = 0.f;
        int match_i = -1;
        std::vector<int> members;
        int scenario = 0;
        RadarHit hit = RadarHit::None;
    };
    CIPOSelection select_cipo(const std::vector<models::Detection>& dets) const;
    CIPOSelection select_cipo_radar(const std::vector<models::Detection>& dets,
                                    const std::vector<RadarPoint>& radar,
                                    const PathPoly* path,
                                    const float* ego_speed_ms,
                                    const float* range_prior_m = nullptr) const;
    static float project_dist(const cv::Mat& H, float ux, float uy);

    void  init_from(float dist_m, float stddev_m, float vel_ms = 0.f, float vel_std = 2.f);
    void  predict(float dt_s);
    void  weight_update(const Meas& ad, const Meas& as_h, const Meas& radar);
    std::vector<float> linear_weights() const;
    float effective_n() const;
    void  resample();
    static float gaussian_loglik(float z, float mean, float sigma);

    enum class TrackSrc { None, Radar, Camera };

    Config cfg_;
    std::vector<Particle> particles_;
    bool   initialised_ = false;
    bool   prev_cut_in_ = false;
    TrackSrc track_src_ = TrackSrc::None;
    std::mt19937 rng_;
    // DO NOT MODIFY! VisionPilot model-view homography (1024x512 pixel -> world). Zenseact Open Dataset
    cv::Mat H_ = (cv::Mat_<double>(3, 3) <<
                       0.00209514907, -0.000941721466, -9.24906396,
                       0.00662758637, -0.000352940531, -3.33396502,
                       0.000120077371, -0.00411343505, 1.0
         );
};

}  // namespace visionpilot::fusion
