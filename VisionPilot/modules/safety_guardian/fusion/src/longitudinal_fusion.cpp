#include <fusion/longitudinal_fusion.hpp>
#include <logging/console.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace visionpilot::fusion {

namespace {

// AutoSpeed class 1 (CIPO Level 1) or 2 (CIPO Level 2).
bool is_cipo_box(const models::Detection& d)
{
    return (d.class_id == 1) || (d.class_id == 2);
}

}  // namespace

// ─── Construction ─────────────────────────────────────────────────────────────

LongitudinalFusion::LongitudinalFusion()
    : LongitudinalFusion(Config{})
{}

LongitudinalFusion::LongitudinalFusion(Config cfg)
    : cfg_(cfg)
    , rng_(static_cast<std::mt19937::result_type>(std::random_device{}()))
{
    if (cfg_.n_particles < 10)
        throw std::invalid_argument("LongitudinalFusion: n_particles must be >= 10");
    particles_.reserve(static_cast<std::size_t>(cfg_.n_particles));
}

// ─── Public API ───────────────────────────────────────────────────────────────

void LongitudinalFusion::reset()
{
    particles_.clear();
    initialised_ = false;
    prev_cut_in_ = false;
    track_src_   = TrackSrc::None;
    pending_hits_ = 0;
    pending_dist_m_ = 0.f;
}

CIPOFusionEstimate LongitudinalFusion::update(
    const models::AutoDriveOutput& autodrive,
    const models::AutoSpeedOutput& autospeed,
    const cv::Mat& /*preprocessed_frame*/,
    float dt_s,
    const std::vector<RadarPoint>* radar,
    const PathPoly* path,
    const float* ego_speed_ms)
{
    CIPOFusionEstimate est;
    est.radar.enabled = cfg_.radar_enabled;
    if (radar != nullptr) est.radar.points = *radar;
    if (path != nullptr)  est.radar.path   = *path;

    Meas as_h;
    float best_as_score = 0.f;
    bool  as_cut_in     = false;
    if (autospeed.valid) {
        const auto sel_h = select_cipo(autospeed.detections);
        as_h = sel_h.meas;
        as_cut_in = sel_h.cut_in;  // L2 closer in range than L1 → CIPO changed
        if (as_h.valid) {
            est.as_h_valid  = true;
            est.as_h_dist_m = as_h.distance_m;
        }
        for (const auto& d : autospeed.detections) {
            if (is_cipo_box(d)) {
                best_as_score = std::max(best_as_score, d.score);
            }
        }
    }

    static constexpr float CIPO_PROB_MIN = 0.40f;  // AD prior + AD-only camera fallback
    static constexpr float D_MAX         = 150.f;

    // Soft AD confirm still feeds the radar static range prior. Particle mean
    // must not confirm a static rail at whatever the track already believes.
    const bool as_cipo_box =
        autospeed.valid &&
        std::any_of(autospeed.detections.begin(), autospeed.detections.end(), is_cipo_box);
    const bool ad_for_assoc =
        autodrive.valid && (as_cipo_box || (autodrive.flag_prob >= CIPO_PROB_MIN));

    const float* ad_ptr = nullptr;
    float ad_dist = 0.f;
    if (ad_for_assoc) {
        ad_dist = cfg_.d_max_m * (1.f - autodrive.dist_normalized);
        if (ad_dist > 1.f) ad_ptr = &ad_dist;
    }

    Meas radar_meas;
    std::vector<std::int32_t> match_members;
    if (cfg_.radar_enabled && (radar != nullptr)) {
        const std::vector<models::Detection> no_dets;
        const auto& dets = autospeed.valid ? autospeed.detections : no_dets;
        const auto sel = select_cipo_radar(dets, *radar, path, ego_speed_ms, ad_ptr);
        match_members        = sel.members;
        radar_meas           = sel.meas;
        est.radar.fov_valid      = sel.fov_valid;
        est.radar.fov_az_rad     = sel.fov_az_rad;
        est.radar.search_x_max_m = sel.search_x_max_m;
        est.radar.match_i    = sel.match_i;
        est.radar.hit        = sel.hit;
        est.radar_scenario   = sel.scenario;
        if (radar_meas.valid) {
            est.radar_meas_valid    = true;
            est.radar_dist_m        = radar_meas.distance_m;
            est.radar_vel_ms        = radar_meas.velocity_ms;
            est.radar.match_range_m = radar_meas.distance_m;
            est.radar.match_rate_ms = radar_meas.velocity_ms;
            est.radar.in_match.assign(radar->size(), 0);
            for (const std::int32_t i : match_members) {
                if ((i >= 0) && (i < static_cast<std::int32_t>(radar->size()))) {
                    est.radar.in_match[static_cast<std::size_t>(i)] = 1;
                }
            }
            est.cipo_raw_found  = true;
            est.cipo_raw_dist_m = radar_meas.distance_m;
        }
        // L2 closer in range (AS) or the associated CIPO box is L2.
        est.cut_in_detected = as_cut_in || sel.cut_in;
    } else {
        est.cut_in_detected = as_cut_in;
    }

    // HUD always shows available camera distances; the filter may ignore them.
    Meas ad_meas;
    if (autodrive.valid && (as_cipo_box || (autodrive.flag_prob >= CIPO_PROB_MIN))) {
        ad_meas.distance_m = D_MAX * (1.f - autodrive.dist_normalized);
        const float p      = std::clamp(autodrive.flag_prob, 0.f, 1.f);
        ad_meas.stddev_m   = cfg_.autodrive_noise_min_m +
                             ((cfg_.autodrive_noise_m - cfg_.autodrive_noise_min_m) * (1.f - p));
        ad_meas.valid      = (ad_meas.distance_m > 1.f);
        if (ad_meas.valid) {
            est.ad_meas_valid = true;
            est.ad_dist_m     = ad_meas.distance_m;
        }
    }

    const bool radar_ok = cfg_.radar_enabled && radar_meas.valid;
    // Radar miss → camera: prefer AS L1/L2 (+ AD if present); else AD if p≥40%.
    const bool as_l12   = as_h.valid;
    const bool ad_only  = !as_l12 && ad_meas.valid &&
                          autodrive.valid && (autodrive.flag_prob >= CIPO_PROB_MIN);
    const bool camera_ok = !radar_ok && (as_l12 || ad_only);
    const TrackSrc src = radar_ok ? TrackSrc::Radar
                       : camera_ok ? TrackSrc::Camera
                                   : TrackSrc::None;

    if (!cfg_.radar_enabled) {
        // Legacy camera-only path (radar off).
        const bool ad_cipo_confirmed =
            autodrive.valid && (as_cipo_box || (autodrive.flag_prob >= CIPO_PROB_MIN));
        if (!ad_cipo_confirmed && !as_h.valid) {
            if (initialised_) {
                std::ostringstream os;
                os << "[Fusion] No CIPO confirmed (AD="
                   << ::logging::fmt_fixed(autodrive.valid ? (autodrive.flag_prob * 100.f) : 0.f, 0)
                   << "%  AS=none) — reset to " << ::logging::fmt_fixed(D_MAX, 0) << " m";
                ::logging::log_info(os.str());
                reset();
            }
            // A gap frame breaks the confirmation streak. Otherwise a phantom,
            // a miss, and a second phantom within the gate would count as two hits.
            pending_hits_ = 0;
            pending_dist_m_ = 0.f;
            est.valid      = true;
            est.distance_m = D_MAX;
            return est;
        }
        if (as_h.valid) {
            est.cipo_raw_found  = true;
            est.cipo_raw_dist_m = as_h.distance_m;
        }
        if (!ad_cipo_confirmed) {
            ad_meas = {};
        } else if (!est.ad_meas_valid && ad_meas.valid) {
            est.ad_meas_valid = true;
            est.ad_dist_m     = ad_meas.distance_m;
        } else {
            // AD confirmed and already reported on the HUD.
        }
    } else if (src == TrackSrc::None) {
        if (initialised_ || (track_src_ != TrackSrc::None)) {
            ::logging::log_info("[Fusion] No relevant radar / camera CIPO — reset to "
                             + ::logging::fmt_fixed(D_MAX, 0) + " m");
            reset();
        }
        pending_hits_ = 0;
        pending_dist_m_ = 0.f;
        est.valid      = true;
        est.distance_m = D_MAX;
        return est;
    } else if ((src == TrackSrc::Camera) && as_h.valid) {
        est.cipo_raw_found  = true;
        est.cipo_raw_dist_m = as_h.distance_m;
    } else {
        // Radar track, or AD-only camera track: raw CIPO already set above.
    }

    // Active filter inputs: radar alone, or camera (AS L1/L2 ± AD, else AD).
    Meas ad_f;
    Meas as_f;
    Meas radar_f;
    if (cfg_.radar_enabled) {
        if (src == TrackSrc::Radar) {
            radar_f = radar_meas;
        } else if (as_l12) {
            as_f = as_h;
            if (ad_meas.valid) ad_f = ad_meas;
        } else {
            ad_f = ad_meas;
        }
    } else {
        ad_f = ad_meas;
        as_f = as_h;
    }

    // ── Particle filter ───────────────────────────────────────────────────────
    const float dt = (dt_s > 1e-6f) ? dt_s : cfg_.dt_s;

    const bool cut_in_edge = est.cut_in_detected && !prev_cut_in_;
    prev_cut_in_ = est.cut_in_detected;

    const bool have_meas = radar_f.valid || as_f.valid || ad_f.valid;
    const float meas_d = radar_f.valid ? radar_f.distance_m
                       : as_f.valid    ? as_f.distance_m
                                       : ad_f.distance_m;

    // Predict before the confirmation test so a real lead moving at the
    // tracked velocity still agrees. A brand-new range has no cloud yet.
    float cloud_mean = 0.f;
    if (initialised_) {
        predict(dt);
        for (const auto& p : particles_) {
            cloud_mean += p.distance_m;
        }
        if (!particles_.empty()) {
            cloud_mean /= static_cast<float>(particles_.size());
        }
    }

    // New track, or a range that cannot be the same object: do not seed and
    // do not let this frame's likelihood pull the cloud. A second agreeing
    // frame accepts it. A measurement that still matches the cloud updates now.
    bool accept = true;
    if (have_meas && (cfg_.confirm_frames > 1)) {
        const bool disagrees = !initialised_ || !same_candidate(meas_d, cloud_mean);
        if (disagrees)
            accept = note_candidate(meas_d);
        else
            pending_hits_ = 0;
    }

    const bool src_switch =
        cfg_.radar_enabled && (track_src_ != TrackSrc::None) && (track_src_ != src);
    if (src_switch && accept) {
        const bool cut = prev_cut_in_;
        std::ostringstream os;
        os << "[Fusion] Source switch " << ((track_src_ == TrackSrc::Radar) ? "radar" : "camera")
           << "→" << ((src == TrackSrc::Radar) ? "radar" : "camera") << " — reset filter";
        ::logging::log_info(os.str());
        reset();
        prev_cut_in_ = cut;  // reset() drops the latch; this frame already consumed the edge
    }

    if (!accept) {
        if (!initialised_) {
            // No confirmed lead. Free-road sentinel: planner treats D_MAX as no CIPO.
            est.valid      = true;
            est.distance_m = cfg_.d_max_m;
            return est;
        }
        // Keep the confirmed lead. Drop this frame's measurement so a
        // one-frame closer phantom cannot reweight or reseed the cloud.
        ad_f = {};
        as_f = {};
        radar_f = {};
    }

    auto seed_from_active = [this, &radar_f, &as_f, &ad_f](float mean_m, const char* reason) {
        using ::logging::fmt_fixed;
        std::ostringstream os;
        os << "[Fusion] " << reason << " — reinit " << fmt_fixed(mean_m, 1) << "→";
        if (radar_f.valid) {
            os << fmt_fixed(radar_f.distance_m, 1) << " m v="
               << fmt_fixed(radar_f.has_velocity ? radar_f.velocity_ms : 0.f, 2) << " (radar)";
            ::logging::log_info(os.str());
            init_from(radar_f.distance_m, cfg_.cipo_noise_m,
                      radar_f.has_velocity ? radar_f.velocity_ms : 0.f,
                      radar_f.has_velocity ? radar_f.stddev_v : 2.f);
        } else if (as_f.valid) {
            os << fmt_fixed(as_f.distance_m, 1) << " m (AS+H)";
            ::logging::log_info(os.str());
            init_from(as_f.distance_m, as_f.stddev_m);
        } else if (ad_f.valid) {
            os << fmt_fixed(ad_f.distance_m, 1) << " m (AD)";
            ::logging::log_info(os.str());
            init_from(ad_f.distance_m, ad_f.stddev_m);
        } else {
            // No active measurement: keep the current cloud.
        }
    };

    if (!initialised_) {
        if (radar_f.valid) {
            init_from(radar_f.distance_m, cfg_.cipo_noise_m,
                      radar_f.has_velocity ? radar_f.velocity_ms : 0.f,
                      radar_f.has_velocity ? radar_f.stddev_v : 2.f);
        } else if (ad_f.valid) {
            init_from(ad_f.distance_m, ad_f.stddev_m);
        } else if (as_f.valid) {
            init_from(as_f.distance_m, cfg_.cipo_noise_m);
        } else {
            return est;
        }
        initialised_ = true;
    } else {
        // predict() already ran. Reseed only a jump that has now been confirmed.
        const float gate = cfg_.reset_gate_m;
        if (cut_in_edge) {
            // New closer CIPO (L2 cut-in). Drop the old cloud; prefer radar
            // cluster range + Doppler when association has a match.
            seed_from_active(cloud_mean, "Cut-in");
        } else if (radar_f.valid &&
                   (std::abs(radar_f.distance_m - cloud_mean) > gate)) {
            seed_from_active(cloud_mean, "Target change");
        } else if (!radar_f.valid && ad_f.valid &&
                   (std::abs(ad_f.distance_m - cloud_mean) > gate)) {
            // Two-sided: cut-out (farther) and sudden drop (closer) on camera.
            seed_from_active(cloud_mean, "Camera jump");
        } else {
            // Same target: the weight update below refines the cloud.
        }
    }

    weight_update(ad_f, as_f, radar_f);
    if (effective_n() < (0.5f * static_cast<float>(cfg_.n_particles))) resample();

    if (cfg_.radar_enabled && accept)
        track_src_ = src;

    const auto w = linear_weights();
    const auto N = particles_.size();
    float mean_d = 0.f;
    float mean_v = 0.f;
    for (std::size_t i = 0; i < N; ++i) {
        mean_d += w[i] * particles_[i].distance_m;
        mean_v += w[i] * particles_[i].velocity_ms;
    }
    float var_d = 0.f;
    for (std::size_t i = 0; i < N; ++i) {
        const float dd = particles_[i].distance_m - mean_d;
        var_d += w[i] * dd * dd;
    }

    est.valid             = true;
    est.distance_m        = mean_d;
    est.velocity_ms       = mean_v;
    est.distance_stddev_m = std::sqrt(std::max(0.f, var_d));

    if (cfg_.debug) {
        using ::logging::fmt_fixed;
        const float ad_pct = autodrive.valid ? (autodrive.flag_prob * 100.f) : 0.f;
        const char* src_txt = !cfg_.radar_enabled ? "cam"
                            : (src == TrackSrc::Radar)  ? "radar"
                            : (src == TrackSrc::Camera) ? "camera"
                                                      : "none";
        std::ostringstream os;
        os << "[Fusion] src=" << src_txt << " | AD=";
        if (est.ad_meas_valid) {
            os << fmt_fixed(est.ad_dist_m, 1) << " m (p=" << fmt_fixed(ad_pct, 0) << "%)"
               << (ad_f.valid ? "" : " [idle]");
        } else {
            os << "(p=" << fmt_fixed(ad_pct, 0) << "%)";
        }
        os << " | AS+H=";
        if (est.as_h_valid) {
            os << fmt_fixed(est.as_h_dist_m, 1) << " m (s=" << fmt_fixed(best_as_score * 100.f, 0)
               << "%)" << (as_f.valid ? "" : " [idle]");
        } else {
            os << "(none)";
        }
        os << " | Radar=";
        if (radar_ok) {
            os << fmt_fixed(radar_meas.distance_m, 1) << " m v=" << fmt_fixed(radar_meas.velocity_ms, 2)
               << " case " << static_cast<int>(est.radar_scenario);
        } else {
            os << "(none)";
        }
        os << (est.cut_in_detected ? " [CUT-IN]" : "")
           << " | Fused=" << fmt_fixed(est.distance_m, 1) << " m  v=" << fmt_fixed(est.velocity_ms, 2)
           << " m/s  ±" << fmt_fixed(est.distance_stddev_m, 1) << " m";
        ::logging::log_info(os.str());
    }

    return est;
}

// ─── Particle filter internals ────────────────────────────────────────────────

bool LongitudinalFusion::same_candidate(float a, float b) const
{
    // A non-finite range is never the same object (and must not reach the gate).
    if (!std::isfinite(a) || !std::isfinite(b)) {
        return false;
    }
    const float ref = std::max(std::abs(a), std::abs(b));
    const float scale = ref / std::max(cfg_.homography_ref_range_m, 1.f);
    // 8 m covers one highway frame (~3 m at 30 m/s and 10 Hz) plus radar noise.
    // The range² term matches homography error, which is several metres nearby
    // and tens of metres at the far end of the 150 m scale.
    const float gate = std::max(8.f, (cfg_.cipo_noise_m * scale * scale));
    return std::abs(a - b) <= gate;
}

bool LongitudinalFusion::note_candidate(float dist_m)
{
    if ((pending_hits_ > 0) && same_candidate(dist_m, pending_dist_m_)) {
        ++pending_hits_;
    } else {
        pending_hits_ = 1;
        pending_dist_m_ = dist_m;
        std::ostringstream os;
        os << "[Fusion] CIPO candidate " << ::logging::fmt_fixed(dist_m, 1)
           << " m — hold braking until " << static_cast<int>(cfg_.confirm_frames) << " consecutive frames";
        ::logging::log_info(os.str());
    }
    if (pending_hits_ < cfg_.confirm_frames)
        return false;
    std::ostringstream os;
    os << "[Fusion] CIPO confirmed " << ::logging::fmt_fixed(dist_m, 1)
       << " m across " << static_cast<int>(pending_hits_) << " frames";
    ::logging::log_info(os.str());
    pending_hits_ = 0;
    return true;
}

void LongitudinalFusion::init_from(float dist_m, float stddev_m, float vel_ms, float vel_std)
{
    particles_.resize(static_cast<std::size_t>(cfg_.n_particles));
    std::normal_distribution<float> nd(dist_m, stddev_m);
    std::normal_distribution<float> nv(vel_ms, vel_std);
    for (auto& p : particles_) {
        p.distance_m  = std::clamp(nd(rng_), 0.f, cfg_.d_max_m);
        p.velocity_ms = nv(rng_);
        p.log_w       = 0.f;
    }
}

void LongitudinalFusion::predict(float dt_s)
{
    std::normal_distribution<float> nd(0.f, cfg_.process_noise_dist_m);
    std::normal_distribution<float> nv(0.f, cfg_.process_noise_vel_ms);
    for (auto& p : particles_) {
        p.distance_m  = std::clamp(p.distance_m + (p.velocity_ms * dt_s) + nd(rng_), 0.f, cfg_.d_max_m);
        p.velocity_ms = p.velocity_ms + nv(rng_);
    }
}

float LongitudinalFusion::gaussian_loglik(float z, float mean, float sigma)
{
    const float d = z - mean;
    return -0.5f * (d / sigma) * (d / sigma);
}

void LongitudinalFusion::weight_update(const Meas& ad, const Meas& as_h, const Meas& radar)
{
    for (auto& p : particles_) {
        if (ad.valid)    p.log_w += gaussian_loglik(ad.distance_m,    p.distance_m, ad.stddev_m);
        if (as_h.valid)  p.log_w += gaussian_loglik(as_h.distance_m,  p.distance_m, as_h.stddev_m);
        if (radar.valid) p.log_w += gaussian_loglik(radar.distance_m, p.distance_m, radar.stddev_m);
        if (radar.valid && radar.has_velocity)
            p.log_w += gaussian_loglik(radar.velocity_ms, p.velocity_ms, radar.stddev_v);
    }
}

std::vector<float> LongitudinalFusion::linear_weights() const
{
    const std::size_t N = particles_.size();
    float max_lw = particles_[0].log_w;
    for (const auto& p : particles_) {
        max_lw = std::max(max_lw, p.log_w);
    }

    std::vector<float> w(N);
    float sum = 0.f;
    for (std::size_t i = 0; i < N; ++i) {
        w[i] = std::exp(particles_[i].log_w - max_lw);
        sum  += w[i];
    }
    if (sum < 1e-12f) {
        const float w0 = 1.f / static_cast<float>(N);
        for (auto& wi : w) {
            wi = w0;
        }
    } else {
        for (auto& wi : w) {
            wi /= sum;
        }
    }
    return w;
}

float LongitudinalFusion::effective_n() const
{
    const auto w = linear_weights();
    float ss = 0.f;
    for (auto wi : w) {
        ss += wi * wi;
    }
    return 1.f / (ss + 1e-12f);
}

void LongitudinalFusion::resample()
{
    const std::size_t N = particles_.size();
    if (N == 0U) return;

    const auto w = linear_weights();
    std::vector<float> cs(N);
    cs[0] = w[0];
    for (std::size_t i = 1U; i < N; ++i) {
        cs[i] = cs[i - 1U] + w[i];
    }

    std::vector<Particle> np;
    np.reserve(N);
    std::uniform_real_distribution<float> u(0.f, 1.f / static_cast<float>(N));
    const float u0 = u(rng_);
    std::size_t j = 0U;
    for (std::size_t i = 0U; i < N; ++i) {
        const float thr = u0 + (static_cast<float>(i) / static_cast<float>(N));
        while (((j + 1U) < N) && (cs[j] < thr)) {
            ++j;
        }
        np.push_back({particles_[j].distance_m, particles_[j].velocity_ms, 0.f});
    }
    particles_ = std::move(np);
}

}  // namespace visionpilot::fusion
