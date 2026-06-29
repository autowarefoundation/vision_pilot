#pragma once

#include <models/auto_drive.hpp>
#include <models/auto_steer.hpp>
#include <opencv2/core.hpp>
#include <random>
#include <string>
#include <vector>

namespace visionpilot::fusion {

// ─── Output struct ────────────────────────────────────────────────────────────
//
//  Axis convention throughout:
//    x = forward  [m]
//    y = lateral  [m, positive = left]
//
struct LateralFusionEstimate {
    bool valid = false;

    // ── Particle-filter tracked outputs (ready for planning) ──────────────────
    float cte_m          = 0.f;   // cross-track error [m]; +ve = ego right of path
    float yaw_rad        = 0.f;   // yaw error [rad];  +ve = path heading left
    float cte_stddev_m   = 0.f;
    float yaw_stddev_rad = 0.f;

    float curvature      = 0.f;   // fused curvature [1/m]; +ve = left turn
    float curv_stddev    = 0.f;

    // ── Raw intermediates for debug / downstream use ───────────────────────────
    bool  path_valid         = false;  // RANSAC polynomial fit succeeded
    float raw_cte_m          = 0.f;   // CTE direct from polynomial (= c-coeff)
    float raw_yaw_rad        = 0.f;   // yaw direct from polynomial (= atan(b))
    float raw_path_curvature = 0.f;   // κ sampled along fitted path (median)
    float raw_ad_curvature   = 0.f;   // curvature_raw from AutoDrive (scaled)
    int   path_inliers       = 0;     // RANSAC inlier count
    int   path_points        = 0;     // world points projected from waypoints
    // Fitted polynomial y = path_a·x² + path_b·x + path_c  (world frame)
    float path_a = 0.f, path_b = 0.f, path_c = 0.f;
    // Forward extent of RANSAC inliers [m] — cap path visualization / MPC samples
    float path_x_min_m = 0.f;
    float path_x_max_m = 0.f;
};

// ─── LateralFusion ────────────────────────────────────────────────────────────
//
//  Per-frame lateral pipeline:
//    1. Project AutoSteer waypoints to world via H (matches video_visualization.py):
//       xp is (1,64): col = fixed image row (y = linspace).
//       u = xp[i] * 1024, v = image row i; mask with h_vector >= 0.5.
//    2. 2nd-order polynomial RANSAC on world points:
//         y_lateral = a·x² + b·x + c
//       → CTE = c, Yaw = atan(b), Curvature = median κ(x) at waypoint x in [curv_x_min, curv_x_max]
//    3. CTE/Yaw particle filter: state [cte, yaw], random-walk process,
//       Gaussian update from step-2 when path is valid.
//    4. Curvature particle filter: state [curv], fuses
//       a) path curvature from step-2  (when path_valid)
//       b) AutoDrive curvature_raw * ad_curvature_scale  (when drive.valid)
//
class LateralFusion {
public:
    struct Config {
        int   n_particles            = 300;
        float dt_s                   = 0.10f;   // nominal dt; overridden per-call

        // Process noise (random-walk, scaled by dt internally)
        float proc_noise_cte_m       = 0.05f;   // allows ~0.05m/s RMS drift for smooth tracking
        float proc_noise_yaw_rad     = 0.01f;
        float proc_noise_curv        = 0.002f;

        // Camera mounting offset compensation.  Set to the observed steady-state
        // CTE on a straight road (camera not at vehicle centreline).
        // Subtracted from raw polynomial CTE before it enters the particle filter.
        float cte_bias_m             = 0.0f;


        // Measurement 1-sigma
        float meas_noise_cte_m       = 0.15f;   // tighter now that CTE is at x_min (not extrapolated)
        float meas_noise_yaw_rad     = 0.05f;
        float meas_noise_curv_path   = 0.005f;  // AutoSteer polynomial curvature
        float meas_noise_curv_ad     = 0.010f;  // AutoDrive curvature (scaled)

        // RANSAC polynomial fit
        float ransac_thresh_m        = 0.20f;   // lateral inlier tolerance [m]
        int   ransac_iters           = 50;
        int   ransac_min_pts         = 5;       // min projected points to attempt fit

        // curvature_raw × scale → physical κ [1/m] (CURV_SCALE = 0.21 in training).
        float ad_curvature_scale     = 0.21f;
        int   ransac_min_inliers     = 20;      // reject path fit if fewer inliers
        float max_abs_cte_m          = 1.2f;    // reject RANSAC fits extrapolating > 1.2m at ego
        float curv_x_min_m           = 3.f;     // κ sampled only at waypoint x ≥ this
        float curv_x_max_m           = 25.f;    // κ sampled only at waypoint x ≤ this

        // Same YAML used by LongitudinalFusion (shared config field).
        bool debug = false;
    };

    LateralFusion();
    explicit LateralFusion(Config cfg);

    LateralFusionEstimate update(
        const models::AutoSteerOutput& steer,
        const models::AutoDriveOutput& drive,
        float dt_s = -1.f
    );

    void reset();

    // Override the internal H matrix used to project AutoSteer waypoints to
    // world space.  Call once after construction when the network runs on a
    // non-BEV image (e.g. plain-resized frame) that requires a different H.
    void set_H(const cv::Mat& H) { H_ = H.clone(); }

private:
    // ── Path processing ───────────────────────────────────────────────────────
    struct WorldPt { float x, y; };   // x = forward [m], y = lateral [m, +left]

    struct PathParams {
        bool  valid     = false;
        float a         = 0.f;        // y = a·x² + b·x + c
        float b         = 0.f;
        float c         = 0.f;
        float cte_m     = 0.f;        // = c
        float yaw_rad   = 0.f;        // = atan(b)
        float curvature = 0.f;        // median κ(x) along path
        int   inliers   = 0;
        float x_min_m   = 0.f;        // min forward x among fit points (inliers)
        float x_max_m   = 0.f;        // max forward x among fit points (inliers)
    };

    std::vector<WorldPt> project_waypoints(const models::AutoSteerOutput& steer) const;
    PathParams           fit_ransac(const std::vector<WorldPt>& pts) const;
    static PathParams    fit_quadratic(const std::vector<WorldPt>& pts,
                                       float curv_x_min_m, float curv_x_max_m);
    static float         eval_quad(float a, float b, float c, float x);
    static float         curvature_at_x(float a, float b, float x);
    static float         sample_path_curvature(const std::vector<WorldPt>& pts,
                                               float a, float b,
                                               float x_min_m, float x_max_m);

    // Project a single image point (u,v) → (x_forward [m], y_lateral [m])
    static std::pair<float, float> project_world(const cv::Mat& H, float u, float v);

    // ── CTE / Yaw particle filter  [state: cte, yaw] ─────────────────────────
    struct CYParticle { float cte, yaw, log_w; };
    void  cy_predict(float dt);
    void  cy_update(float meas_cte, float noise_cte,
                    float meas_yaw, float noise_yaw);
    void  cy_resample();
    float cy_eff_n() const;
    void  cy_init(float cte, float yaw);

    // ── Curvature particle filter  [state: curvature] ────────────────────────
    struct KParticle { float curv, log_w; };
    void  k_predict(float dt);
    void  k_update(float meas, float noise);
    void  k_resample();
    float k_eff_n() const;
    void  k_init(float curv);

    // ── Shared helpers ────────────────────────────────────────────────────────
    static float gauss_loglik(float z, float mean, float sigma);

    template<typename P>
    std::vector<float> linear_weights(const std::vector<P>& ps) const;

    template<typename P>
    void systematic_resample(std::vector<P>& particles);

    Config cfg_;

    std::vector<CYParticle> cy_;
    bool                    cy_init_ = false;

    std::vector<KParticle>  k_;
    bool                    k_init_  = false;

    std::mt19937 rng_;
    // DO NOT MODIFY! VisionPilot model-view homography (1024x512 pixel -> world). Zenseact Open Dataset
    cv::Mat H_ = (cv::Mat_<double>(3, 3) <<
                      0.00209514907, -0.000941721466, -9.24906396,
                      0.00662758637, -0.000352940531, -3.33396502,
                      0.000120077371, -0.00411343505, 1.0
        );
};

}  // namespace visionpilot::fusion
