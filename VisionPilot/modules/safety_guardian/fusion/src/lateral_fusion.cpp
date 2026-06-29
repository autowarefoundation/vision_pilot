#include <fusion/lateral_fusion.hpp>
#include <logging/logger.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace visionpilot::fusion {

// ─── Construction / reset ────────────────────────────────────────────────────

LateralFusion::LateralFusion()
    : LateralFusion(Config{})
{}

LateralFusion::LateralFusion(Config cfg)
    : cfg_(cfg)
    , rng_(std::random_device{}())
{
    if (cfg_.n_particles < 10)
        throw std::invalid_argument("LateralFusion: n_particles must be >= 10");
    cy_.reserve(static_cast<std::size_t>(cfg_.n_particles));
    k_.reserve(static_cast<std::size_t>(cfg_.n_particles));
}

void LateralFusion::reset()
{
    cy_.clear();  cy_init_ = false;
    k_.clear();   k_init_  = false;
}

// ─── Public update ────────────────────────────────────────────────────────────

LateralFusionEstimate LateralFusion::update(
    const models::AutoSteerOutput& steer,
    const models::AutoDriveOutput& drive,
    float dt_s)
{
    LateralFusionEstimate est;
    const float dt = (dt_s > 1e-6f) ? dt_s : cfg_.dt_s;

    // ── Step 2: project AutoSteer waypoints → world points ───────────────────
    PathParams path;
    if (steer.valid) {
        const auto pts = project_waypoints(steer);
        est.path_points = static_cast<int>(pts.size());
        if (!pts.empty())
            path = fit_ransac(pts);
    }

        est.path_valid         = path.valid;
        est.raw_cte_m          = path.cte_m;
        est.raw_yaw_rad        = path.yaw_rad;
        est.raw_path_curvature = path.curvature;
        est.path_inliers       = path.inliers;
        est.path_a             = path.a;
        est.path_b             = path.b;
        est.path_c             = path.c;
        est.path_x_min_m       = path.x_min_m;
        est.path_x_max_m       = path.x_max_m;

    // ── Step 3: AutoDrive curvature ───────────────────────────────────────────
    float ad_curv = 0.f;
    bool  ad_curv_valid = false;
    if (drive.valid) {
        ad_curv       = drive.curvature_raw * cfg_.ad_curvature_scale;
        ad_curv_valid = true;
        est.raw_ad_curvature = ad_curv;
    }

    // ── Step 4: CTE / Yaw particle filter ────────────────────────────────────
    if (path.valid) {
        // Apply camera mounting offset correction before feeding the filter.
        const float corrected_cte = path.cte_m - cfg_.cte_bias_m;
        if (!cy_init_) {
            cy_init(corrected_cte, path.yaw_rad);
        } else {
            cy_predict(dt);
            cy_update(corrected_cte, cfg_.meas_noise_cte_m,
                      path.yaw_rad, cfg_.meas_noise_yaw_rad);
            if (cy_eff_n() < 0.5f * static_cast<float>(cfg_.n_particles))
                cy_resample();
        }
    }

    // ── Step 5: Curvature particle filter ────────────────────────────────────
    // Initialise from whichever source is available first.
    if (!k_init_) {
        if      (path.valid)    k_init(path.curvature);
        else if (ad_curv_valid) k_init(ad_curv);
    } else {
        k_predict(dt);
        if (path.valid)    k_update(path.curvature, cfg_.meas_noise_curv_path);
        if (ad_curv_valid) k_update(ad_curv,        cfg_.meas_noise_curv_ad);
        if (k_eff_n() < 0.5f * static_cast<float>(cfg_.n_particles))
            k_resample();
    }

    // ── Step 6: Extract posterior means ──────────────────────────────────────
    if (cy_init_ && !cy_.empty()) {
        const auto w = linear_weights(cy_);
        float mean_cte = 0.f, mean_yaw = 0.f;
        for (std::size_t i = 0; i < cy_.size(); ++i) {
            mean_cte += w[i] * cy_[i].cte;
            mean_yaw += w[i] * cy_[i].yaw;
        }
        float var_cte = 0.f, var_yaw = 0.f;
        for (std::size_t i = 0; i < cy_.size(); ++i) {
            const float dc = cy_[i].cte - mean_cte;
            const float dy = cy_[i].yaw - mean_yaw;
            var_cte += w[i] * dc * dc;
            var_yaw += w[i] * dy * dy;
        }
        est.cte_m          = mean_cte;
        est.yaw_rad        = mean_yaw;
        est.cte_stddev_m   = std::sqrt(std::max(0.f, var_cte));
        est.yaw_stddev_rad = std::sqrt(std::max(0.f, var_yaw));
        est.valid          = true;
    }

    if (k_init_ && !k_.empty()) {
        const auto w = linear_weights(k_);
        float mean_k = 0.f;
        for (std::size_t i = 0; i < k_.size(); ++i)
            mean_k += w[i] * k_[i].curv;
        float var_k = 0.f;
        for (std::size_t i = 0; i < k_.size(); ++i) {
            const float dk = k_[i].curv - mean_k;
            var_k += w[i] * dk * dk;
        }
        est.curvature   = mean_k;
        est.curv_stddev = std::sqrt(std::max(0.f, var_k));
        est.valid       = true;
    }

    // ── Step 7: Debug log ─────────────────────────────────────────────────────
    if (cfg_.debug) {
        char path_buf[96], ad_buf[32];
        if (path.valid)
            std::snprintf(path_buf, sizeof(path_buf),
                          "raw_CTE=%.2fm  bias=%.2fm  corr=%.2fm  yaw=%.3frad  κ=%.4f  (%d pts, xmin=%.1fm)",
                          path.cte_m, cfg_.cte_bias_m, path.cte_m - cfg_.cte_bias_m,
                          path.yaw_rad, path.curvature, path.inliers, path.x_min_m);
        else
            std::snprintf(path_buf, sizeof(path_buf), "(no path fit, %d pts)", est.path_points);

        if (ad_curv_valid)
            std::snprintf(ad_buf, sizeof(ad_buf), "%.4f (raw=%.4f)",
                          ad_curv, drive.curvature_raw);
        else
            std::snprintf(ad_buf, sizeof(ad_buf), "(none)");

        VP_INFO("[Lateral] Path: %s | AD-κ=%s | Fused CTE=%.2fm yaw=%.3frad κ=%.4f",
                path_buf, ad_buf,
                est.cte_m, est.yaw_rad, est.curvature);
    }

    return est;
}

// ─── Waypoint projection ──────────────────────────────────────────────────────
//
//  Matches Models/visualizations/AutoSteer/video_visualization.py:
//    yp[i] = linspace(0, NET_H-1, 64)
//    u = xp[i] * NET_W  (masked by h_vector[i] >= 0.5)
//
std::vector<LateralFusion::WorldPt>
LateralFusion::project_waypoints(const models::AutoSteerOutput& steer) const
{
    static constexpr int N_WP  = 64;
    static constexpr float NET_W = 1024.f;
    static constexpr float NET_H = 512.f;

    std::vector<WorldPt> out;
    out.reserve(N_WP);

    for (int i = 0; i < N_WP; ++i) {
        const float v_px = (N_WP <= 1) ? 0.f
            : static_cast<float>(i) * (NET_H - 1.f) / static_cast<float>(N_WP - 1);

        if (steer.h_vector[i] < 0.5f) continue;

        const float u_px = steer.xp[i] * NET_W;
        auto [xw, yw] = project_world(H_, u_px, v_px);
        // Keep points in a sensible forward range for the polynomial fit
        if (xw < 0.5f || xw > 120.f) continue;
        out.push_back({xw, yw});
    }
    return out;
}

// Project image (u,v) → world (x_forward [m], y_lateral [m, +left])
std::pair<float, float>
LateralFusion::project_world(const cv::Mat& H, float u, float v)
{
    std::vector<cv::Point2f> src = {cv::Point2f(u, v)}, dst;
    cv::perspectiveTransform(src, dst, H);
    return {dst[0].x, dst[0].y};
}

// ─── Polynomial RANSAC ────────────────────────────────────────────────────────
//
//  Fits: y_lateral = a·x_forward² + b·x_forward + c
//  RANSAC selects the best inlier set, then refits with least squares.
//
LateralFusion::PathParams
LateralFusion::fit_ransac(const std::vector<WorldPt>& pts) const
{
    if (static_cast<int>(pts.size()) < 3) return {};

    PathParams best;
    best.inliers = 0;
    std::vector<WorldPt> best_inliers_pts;

    // Fallback: all points
    best_inliers_pts = pts;

    if (static_cast<int>(pts.size()) > 3) {
        std::uniform_int_distribution<int> pick(0, static_cast<int>(pts.size()) - 1);

        for (int iter = 0; iter < cfg_.ransac_iters; ++iter) {
            // Sample 3 distinct points
            int ia = pick(const_cast<std::mt19937&>(rng_));
            int ib, ic;
            do { ib = pick(const_cast<std::mt19937&>(rng_)); } while (ib == ia);
            do { ic = pick(const_cast<std::mt19937&>(rng_)); } while (ic == ia || ic == ib);

            const std::vector<WorldPt> sample = {pts[ia], pts[ib], pts[ic]};
            const PathParams model = fit_quadratic(sample, cfg_.curv_x_min_m, cfg_.curv_x_max_m);
            if (!model.valid) continue;

            // Count inliers
            std::vector<WorldPt> inl;
            for (const auto& p : pts) {
                const float y_pred = eval_quad(model.a, model.b, model.c, p.x);
                if (std::abs(y_pred - p.y) < cfg_.ransac_thresh_m)
                    inl.push_back(p);
            }
            if (static_cast<int>(inl.size()) > best.inliers) {
                best.inliers     = static_cast<int>(inl.size());
                best_inliers_pts = inl;
            }
        }
    }

    // Final refit on inlier set
    if (static_cast<int>(best_inliers_pts.size()) >= 3) {
        PathParams final_fit = fit_quadratic(best_inliers_pts, cfg_.curv_x_min_m, cfg_.curv_x_max_m);
        final_fit.inliers    = static_cast<int>(best_inliers_pts.size());
        if (final_fit.inliers < cfg_.ransac_min_inliers) return {};
        if (std::abs(final_fit.cte_m) > cfg_.max_abs_cte_m) return {};
        return final_fit;
    }
    return {};
}

// κ(x) for y = a·x² + b·x + c  →  κ = 2a / (1 + (2ax + b)²)^(3/2)
float LateralFusion::curvature_at_x(float a, float b, float x)
{
    const float dydx = 2.f * a * x + b;
    const float denom = std::pow(1.f + dydx * dydx, 1.5f);
    if (denom < 1e-6f) return 0.f;
    return (2.f * a) / denom;
}

// Median κ(x) on the fitted curve at each waypoint x inside the lookahead window.
float LateralFusion::sample_path_curvature(const std::vector<WorldPt>& pts,
                                           float a, float b,
                                           float x_min_m, float x_max_m)
{
    if (x_min_m > x_max_m) std::swap(x_min_m, x_max_m);

    std::vector<float> kappas;
    kappas.reserve(pts.size());
    for (const auto& p : pts) {
        if (p.x < x_min_m || p.x > x_max_m) continue;
        kappas.push_back(curvature_at_x(a, b, p.x));
    }

    if (kappas.empty())
        return curvature_at_x(a, b, x_min_m);

    const auto mid = kappas.begin() + kappas.size() / 2;
    std::nth_element(kappas.begin(), mid, kappas.end());
    return *mid;
}

// Least-squares quadratic fit: y = a·x² + b·x + c
LateralFusion::PathParams
LateralFusion::fit_quadratic(const std::vector<WorldPt>& pts,
                             float curv_x_min_m, float curv_x_max_m)
{
    const int n = static_cast<int>(pts.size());
    if (n < 3) return {};

    cv::Mat A(n, 3, CV_32F);
    cv::Mat B(n, 1, CV_32F);
    for (int i = 0; i < n; ++i) {
        const float x = pts[i].x;
        A.at<float>(i, 0) = x * x;
        A.at<float>(i, 1) = x;
        A.at<float>(i, 2) = 1.f;
        B.at<float>(i, 0) = pts[i].y;
    }

    cv::Mat sol;
    if (!cv::solve(A, B, sol, cv::DECOMP_SVD)) return {};

    PathParams p;
    p.valid = true;
    p.a = sol.at<float>(0);
    p.b = sol.at<float>(1);
    p.c = sol.at<float>(2);

    // Find x range of actual data first so CTE is evaluated there, not extrapolated to x=0.
    p.x_min_m = pts[0].x;
    p.x_max_m = pts[0].x;
    for (const auto& pt : pts) {
        p.x_min_m = std::min(p.x_min_m, pt.x);
        p.x_max_m = std::max(p.x_max_m, pt.x);
    }

    // Evaluate CTE and yaw at x_min_m (closest observed waypoint) — avoids
    // extrapolating back to x=0 where there is no actual data.
    const float x_ref   = p.x_min_m;
    const float dydx    = 2.f * p.a * x_ref + p.b;   // polynomial slope at x_ref
    p.cte_m   = eval_quad(p.a, p.b, p.c, x_ref);
    p.yaw_rad = std::atan(dydx);
    p.curvature = sample_path_curvature(pts, p.a, p.b, curv_x_min_m, curv_x_max_m);

    return p;
}

float LateralFusion::eval_quad(float a, float b, float c, float x)
{
    return a * x * x + b * x + c;
}

// ─── CTE / Yaw particle filter ────────────────────────────────────────────────

void LateralFusion::cy_init(float cte, float yaw)
{
    const int N = cfg_.n_particles;
    cy_.resize(static_cast<std::size_t>(N));
    std::normal_distribution<float> nc(cte, cfg_.meas_noise_cte_m);
    std::normal_distribution<float> ny(yaw, cfg_.meas_noise_yaw_rad);
    for (auto& p : cy_) { p.cte = nc(rng_); p.yaw = ny(rng_); p.log_w = 0.f; }
    cy_init_ = true;
}

void LateralFusion::cy_predict(float dt)
{
    std::normal_distribution<float> nc(0.f, cfg_.proc_noise_cte_m  * std::sqrt(dt / cfg_.dt_s));
    std::normal_distribution<float> ny(0.f, cfg_.proc_noise_yaw_rad * std::sqrt(dt / cfg_.dt_s));
    for (auto& p : cy_) { p.cte += nc(rng_); p.yaw += ny(rng_); }
}

void LateralFusion::cy_update(float meas_cte, float noise_cte,
                               float meas_yaw, float noise_yaw)
{
    for (auto& p : cy_) {
        p.log_w += gauss_loglik(meas_cte, p.cte, noise_cte);
        p.log_w += gauss_loglik(meas_yaw, p.yaw, noise_yaw);
    }
}

float LateralFusion::cy_eff_n() const
{
    const auto w = linear_weights(cy_);
    float ss = 0.f;
    for (auto wi : w) ss += wi * wi;
    return 1.f / (ss + 1e-12f);
}

void LateralFusion::cy_resample() { systematic_resample(cy_); }

// ─── Curvature particle filter ────────────────────────────────────────────────

void LateralFusion::k_init(float curv)
{
    const int N = cfg_.n_particles;
    k_.resize(static_cast<std::size_t>(N));
    std::normal_distribution<float> nd(curv, cfg_.meas_noise_curv_path);
    for (auto& p : k_) { p.curv = nd(rng_); p.log_w = 0.f; }
    k_init_ = true;
}

void LateralFusion::k_predict(float dt)
{
    std::normal_distribution<float> nd(0.f, cfg_.proc_noise_curv * std::sqrt(dt / cfg_.dt_s));
    for (auto& p : k_) p.curv += nd(rng_);
}

void LateralFusion::k_update(float meas, float noise)
{
    for (auto& p : k_)
        p.log_w += gauss_loglik(meas, p.curv, noise);
}

float LateralFusion::k_eff_n() const
{
    const auto w = linear_weights(k_);
    float ss = 0.f;
    for (auto wi : w) ss += wi * wi;
    return 1.f / (ss + 1e-12f);
}

void LateralFusion::k_resample() { systematic_resample(k_); }

// ─── Shared helpers ────────────────────────────────────────────────────────────

float LateralFusion::gauss_loglik(float z, float mean, float sigma)
{
    const float d = z - mean;
    return -0.5f * (d / sigma) * (d / sigma);
}

// Log-sum-exp stable weight normalisation
template<typename P>
std::vector<float> LateralFusion::linear_weights(const std::vector<P>& ps) const
{
    const std::size_t N = ps.size();
    float max_lw = ps[0].log_w;
    for (const auto& p : ps) max_lw = std::max(max_lw, p.log_w);

    std::vector<float> w(N);
    float sum = 0.f;
    for (std::size_t i = 0; i < N; ++i) {
        w[i] = std::exp(ps[i].log_w - max_lw);
        sum  += w[i];
    }
    if (sum < 1e-12f) {
        const float w0 = 1.f / static_cast<float>(N);
        for (auto& wi : w) wi = w0;
    } else {
        for (auto& wi : w) wi /= sum;
    }
    return w;
}

// Systematic resampling — preserves diversity, O(N)
template<typename P>
void LateralFusion::systematic_resample(std::vector<P>& particles)
{
    const int N = static_cast<int>(particles.size());
    if (N == 0) return;

    const auto w = linear_weights(particles);
    std::vector<float> cs(static_cast<std::size_t>(N));
    cs[0] = w[0];
    for (int i = 1; i < N; ++i)
        cs[static_cast<std::size_t>(i)] = cs[static_cast<std::size_t>(i-1)] +
                                          w[static_cast<std::size_t>(i)];

    std::vector<P> np;
    np.reserve(static_cast<std::size_t>(N));
    std::uniform_real_distribution<float> u(0.f, 1.f / static_cast<float>(N));
    const float u0 = u(rng_);
    int j = 0;
    for (int i = 0; i < N; ++i) {
        const float thr = u0 + static_cast<float>(i) / static_cast<float>(N);
        while (j < N-1 && cs[static_cast<std::size_t>(j)] < thr) ++j;
        P pnew = particles[static_cast<std::size_t>(j)];
        pnew.log_w = 0.f;
        np.push_back(pnew);
    }
    particles = std::move(np);
}

}  // namespace visionpilot::fusion
