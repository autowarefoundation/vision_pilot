#include <debug/debug_draw.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <filesystem>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace visionpilot::debug {

namespace fs = std::filesystem;

// ─── Layout (matches AutoSteer / AutoDrive Python visualizations) ─────────────
static constexpr int   kNetW      = 1024;
static constexpr int   kNetH      = 512;
static constexpr int   kPathPts   = 64;
static constexpr int   kWheelPx   = 92;

// AutoSpeed bbox colors (BGR)
static const cv::Scalar kClrL1        {  0,  0, 220};
static const cv::Scalar kClrL2        {  0, 210, 255};
static const cv::Scalar kClrL3        {220, 200,   0};
static const cv::Scalar kClrLOther    { 80, 200,  80};

static const cv::Scalar kClrEgoPath   {  0, 255,   0};   // green — AutoSteer
static const cv::Scalar kClrHudBg     {  0,   0,   0};
static const cv::Scalar kClrHudLine   { 70,  70,  70};
static const cv::Scalar kClrHeader    {220, 220, 220};
static const cv::Scalar kClrNormal    {200, 200, 200};
static const cv::Scalar kClrFusedLong {  0, 220,   0};
static const cv::Scalar kClrFusedLat  {  0, 220, 255};  // yellow/cyan — fused path
static const cv::Scalar kClrTopBar    {200, 200, 200};
static const cv::Scalar kClrSteerLbl  {120, 240, 120};
static const cv::Scalar kClrAdWheel   {240, 240, 240};

// Layout (1024×512): corners reserved so paths/HUD do not overlap
static constexpr int kTopBarH   = 22;
static constexpr int kHudH      = 128;
static constexpr int kLegendW   = 212;
static constexpr int kLegendH   = 100;
static constexpr int kBevW      = 200;
static constexpr int kBevH      = 148;

static constexpr int    kFont  = cv::FONT_HERSHEY_SIMPLEX;
static constexpr double kSmall = 0.38;
static constexpr double kNorm  = 0.42;
static constexpr int    kThin  = 1;
static constexpr int    kBold  = 2;

// ─── Wheel assets (lazy load) ─────────────────────────────────────────────────
static std::mutex              g_wheel_mu;
static cv::Mat                 g_wheel_white;   // BGRA
static cv::Mat                 g_wheel_green;
static bool                    g_wheels_tried  = false;

static std::mutex              g_homo_mu;
static cv::Mat                 g_H_world_to_img;
static bool                    g_homo_tried    = false;

static inline cv::Scalar det_color(int class_id) {
    switch (class_id) {
        case 1: return kClrL1;
        case 2: return kClrL2;
        case 3: return kClrL3;
        default: return kClrLOther;
    }
}

static inline std::string fd(float v, int d) {
    char b[24];
    std::snprintf(b, sizeof(b), "%.*f", d, static_cast<double>(v));
    return b;
}

static void fill_rect(cv::Mat& img, cv::Rect r, cv::Scalar color, double alpha)
{
    const cv::Rect clip = r & cv::Rect(0, 0, img.cols, img.rows);
    if (clip.width <= 0 || clip.height <= 0) return;
    cv::Mat roi = img(clip);
    cv::Mat block(roi.size(), roi.type(), color);
    cv::addWeighted(block, alpha, roi, 1.0 - alpha, 0.0, roi);
}

struct OverlayLayout {
    int hud_y = 0;
    cv::Rect legend{};
    cv::Rect bev{};
};

static OverlayLayout layout_for(const cv::Mat& img, bool radar_bev = false)
{
    OverlayLayout L;
    L.hud_y = img.rows - kHudH;
    L.legend = cv::Rect(6, kTopBarH + 4, kLegendW, radar_bev ? 148 : kLegendH);
    const int bw = radar_bev ? 340 : kBevW;
    const int bh = radar_bev ? 260 : kBevH;
    L.bev    = cv::Rect(img.cols - bw - 8, L.hud_y - bh - 6, bw, bh);
    return L;
}

static void draw_panel(cv::Mat& img, cv::Rect r, const char* title, cv::Scalar accent)
{
    const cv::Rect clip = r & cv::Rect(0, 0, img.cols, img.rows);
    if (clip.width <= 0 || clip.height <= 0) return;
    fill_rect(img, clip, kClrHudBg, 0.82);
    cv::rectangle(img, clip, accent, 1, cv::LINE_AA);
    if (title && title[0])
        cv::putText(img, title, cv::Point(clip.x + 6, clip.y + 14),
                    kFont, 0.40, accent, 1, cv::LINE_AA);
}

static void draw_tag(cv::Mat& img, cv::Point anchor, const char* label, cv::Scalar color)
{
    int baseline = 0;
    const cv::Size sz = cv::getTextSize(label, kFont, 0.44, 1, &baseline);
    const cv::Rect bg(anchor.x, anchor.y - sz.height - 6,
                      sz.width + 10, sz.height + 10);
    fill_rect(img, bg & cv::Rect(0, 0, img.cols, img.rows), kClrHudBg, 0.88);
    cv::rectangle(img, bg, color, 1, cv::LINE_AA);
    cv::putText(img, label, cv::Point(anchor.x + 5, anchor.y - 5),
                kFont, 0.44, color, 1, cv::LINE_AA);
}

// κ [1/m] → road-wheel angle [deg] (image_visualization.py)
static float curvature_to_steer_deg(float curv_1pm,
                                    float wheelbase_m,
                                    float steer_ratio)
{
    return static_cast<float>(
        std::atan(curv_1pm * wheelbase_m) * steer_ratio * (180.0 / M_PI));
}

static void draw_legend(cv::Mat& img, const OverlayLayout& L,
                        const DebugView& v)
{
    draw_panel(img, L.legend, "OVERLAY KEY", kClrHeader);

    const int x0 = L.legend.x + 8;
    int y = L.legend.y + 28;
    const int dy = 16;

    auto swatch = [&](int yl, cv::Scalar c, const char* txt) {
        cv::rectangle(img, cv::Rect(x0, yl - 9, 14, 4), c, -1, cv::LINE_AA);
        cv::putText(img, txt, cv::Point(x0 + 20, yl), kFont, 0.38, kClrNormal, 1, cv::LINE_AA);
    };

    swatch(y, kClrEgoPath, "Green  AutoSteer waypoints"); y += dy;
    swatch(y, kClrFusedLat, "Yellow Path fit (inliers)"); y += dy;

    if (v.cipo.radar.enabled) {
        swatch(y, cv::Scalar(200, 90, 40), "Teal   Search +/-1.8m to 150m"); y += dy;
        swatch(y, cv::Scalar(0, 255, 100), "Green  CIPO cluster"); y += dy;
        swatch(y, cv::Scalar(0, 140, 255), "Orange Mover in-path"); y += dy;
        swatch(y, cv::Scalar(200, 80, 200), "Magenta Static in-path"); y += dy;
        swatch(y, cv::Scalar(90, 90, 90), "Grey   Out of search zone"); y += dy;
        return;
    }

    if (v.lateral.valid) {
        const float st = curvature_to_steer_deg(
            v.lateral.curvature, v.vehicle.wheelbase_m, v.vehicle.steer_ratio);
        char buf[48];
        std::snprintf(buf, sizeof(buf), "Fused steer %.1f deg", static_cast<double>(st));
        cv::putText(img, buf, cv::Point(x0, y), kFont, 0.36, kClrSteerLbl, 1, cv::LINE_AA);
        y += dy;
    }
    if (v.auto_drive.valid) {
        const float curv = v.auto_drive.curvature_raw * v.vehicle.curv_scale;
        const float st = curvature_to_steer_deg(
            curv, v.vehicle.wheelbase_m, v.vehicle.steer_ratio);
        char buf[48];
        std::snprintf(buf, sizeof(buf), "AD steer %.1f deg (white wheel)", static_cast<double>(st));
        cv::putText(img, buf, cv::Point(x0, y), kFont, 0.36, kClrAdWheel, 1, cv::LINE_AA);
    }
}

static cv::Mat load_wheel_rgba(const fs::path& path, int size_px)
{
    cv::Mat img = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    if (img.empty()) return {};
    cv::resize(img, img, cv::Size(size_px, size_px), 0, 0, cv::INTER_LINEAR);
    if (img.channels() == 3)
        cv::cvtColor(img, img, cv::COLOR_BGR2BGRA);
    return img;
}

static fs::path resolve_wheel_dir(const std::string& requested)
{
    if (!requested.empty() && fs::is_directory(requested))
        return requested;

    const fs::path cwd = fs::current_path();
    const std::vector<fs::path> candidates = {
        cwd / ".." / "0.9" / "images",
        cwd / ".." / ".." / "development_releases" / "0.9" / "images",
        cwd / "VisionPilot" / "development_releases" / "0.9" / "images",
        cwd / "VisionPilot" / "production_release" / "images",
        fs::path("VisionPilot/development_releases/0.9/images"),
        fs::path("VisionPilot/production_release/images"),
    };
    for (const auto& c : candidates) {
        if (fs::is_directory(c))
            return c;
    }
    return {};
}

static cv::Mat load_homography_inv()
{
    cv::Mat H64 = (cv::Mat_<double>(3, 3) <<
                      0.00209514907, -0.000941721466, -9.24906396,
                      0.00662758637, -0.000352940531, -3.33396502,
                      0.000120077371, -0.00411343505, 1.0
        );
    cv::Mat H32;
    H64.convertTo(H32, CV_32F);
    return H32.inv();
}

void init_homography()
{
    std::lock_guard<std::mutex> lock(g_homo_mu);
    if (g_homo_tried) return;
    g_homo_tried = true;
    g_H_world_to_img = load_homography_inv();
}

void init_wheel_assets(const std::string& wheel_dir)
{
    std::lock_guard<std::mutex> lock(g_wheel_mu);
    if (g_wheels_tried) return;
    g_wheels_tried = true;

    const fs::path dir = resolve_wheel_dir(wheel_dir);
    if (dir.empty()) return;

    g_wheel_white = load_wheel_rgba(dir / "wheel_white.png", kWheelPx);
    g_wheel_green = load_wheel_rgba(dir / "wheel_green.png", kWheelPx);
}

static cv::Mat rotate_wheel_rgba(const cv::Mat& src, float angle_deg)
{
    if (src.empty()) return {};
    cv::Point2f center(src.cols / 2.f, src.rows / 2.f);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle_deg, 1.0);
    cv::Mat out;
    cv::warpAffine(src, out, rot, src.size(), cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));
    return out;
}

static void paste_rgba(cv::Mat& base, const cv::Mat& overlay, int x, int y)
{
    if (overlay.empty() || base.empty()) return;
    const int w = overlay.cols, h = overlay.rows;
    const int x1 = std::max(x, 0), y1 = std::max(y, 0);
    const int x2 = std::min(x + w, base.cols), y2 = std::min(y + h, base.rows);
    if (x2 <= x1 || y2 <= y1) return;

    cv::Mat roi = base(cv::Rect(x1, y1, x2 - x1, y2 - y1));
    cv::Mat src = overlay(cv::Rect(x1 - x, y1 - y, x2 - x1, y2 - y1));

    if (src.channels() == 4) {
        std::vector<cv::Mat> ch;
        cv::split(src, ch);
        cv::Mat rgb, alpha;
        cv::merge(std::vector<cv::Mat>{ch[0], ch[1], ch[2]}, rgb);
        alpha = ch[3];
        cv::Mat rgb_f, roi_f, alpha_f;
        rgb.convertTo(rgb_f, CV_32FC3, 1.0 / 255.0);
        roi.convertTo(roi_f, CV_32FC3, 1.0 / 255.0);
        alpha.convertTo(alpha_f, CV_32FC1, 1.0 / 255.0);
        cv::Mat alpha3;
        cv::cvtColor(alpha_f, alpha3, cv::COLOR_GRAY2BGR);
        cv::Mat blended = rgb_f.mul(alpha3) + roi_f.mul(cv::Scalar(1.f, 1.f, 1.f) - alpha3);
        blended.convertTo(roi, CV_8UC3, 255.0);
    } else {
        src.copyTo(roi);
    }
}

// ─── AutoSpeed detections ─────────────────────────────────────────────────────

static void draw_autospeed_detections(cv::Mat& img,
                                       const models::AutoSpeedOutput& speed)
{
    if (!speed.valid) return;
    for (const auto& d : speed.detections) {
        const cv::Scalar clr = det_color(d.class_id);
        const cv::Point  tl(static_cast<int>(d.x1), static_cast<int>(d.y1));
        const cv::Point  br(static_cast<int>(d.x2), static_cast<int>(d.y2));
        cv::rectangle(img, tl, br, clr, 2, cv::LINE_AA);
        char lbl[32];
        std::snprintf(lbl, sizeof(lbl), "L%d %.0f%%", d.class_id, d.score * 100.f);
        cv::putText(img, lbl, cv::Point(tl.x + 2, std::max(tl.y - 4, 10)),
                    kFont, 0.40, clr, 1, cv::LINE_AA);
    }
}

// ─── AutoSteer ego path (video_visualization.py) ─────────────────────────────
// xp/h_vector are (1, 64): lateral samples at fixed image rows.
// y = linspace(0, H-1, 64); u = xp[i] * W; mask with h_vector >= 0.5

// World (x=forward, y=lateral) → BEV inset pixels. Ego at bottom-centre, forward up.
static cv::Point world_to_bev_px(float x_fwd, float y_lat,
                                 int panel_w, int panel_h, float px_per_m)
{
    const int cx = panel_w / 2;
    const int cy = panel_h - 12;
    return cv::Point(
        static_cast<int>(std::lround(cx - y_lat * px_per_m)),
        static_cast<int>(std::lround(cy - x_fwd * px_per_m)));
}

// A static return's range-rate is -v_ego·cos(azimuth), and on a highway most
// returns are static world (ground, barriers, signs).  The median of the
// de-projected rate is therefore -v_ego, robust to the few moving targets.
static float ego_speed_from_radar(const std::vector<RadarPoint>& pts)
{
    std::vector<float> rr;
    rr.reserve(pts.size());
    for (const auto& p : pts) {
        const float ca = std::cos(p.azimuth_rad);
        if (p.range_m > 2.f && ca > 0.2f) rr.push_back(p.range_rate / ca);
    }
    if (rr.size() < 8) return 0.f;
    std::nth_element(rr.begin(), rr.begin() + static_cast<long>(rr.size() / 2), rr.end());
    return -rr[rr.size() / 2];
}

// Radar BEV. Forward and lateral use *different* scales: 0-150 m of range has to
// share a 260 px panel, so a 1.8 m association gate is a few pixels tall.
// Stretching the lateral axis is the only way the gates are visible at all.
//
// Cluster membership is not recomputed here — it arrives in RadarAssocDebug so
// the panel can only ever show the association fusion actually made. The path
// corridor is drawn the way fusion searches: fitted poly out to x_max, then
// the same poly extrapolated to radar_max_range (150 m) with a +/-1.8 m tube.
static void draw_radar_bev_panel(cv::Mat& img, const DebugView& view,
                                 const OverlayLayout& L)
{
    const auto& lat   = view.lateral;
    const auto& radar = view.cipo.radar;

    const cv::Rect panel_rect = L.bev & cv::Rect(0, 0, img.cols, img.rows);
    if (panel_rect.width < 80 || panel_rect.height < 80) return;
    draw_panel(img, panel_rect, "RADAR BEV", kClrFusedLat);

    const int pw = panel_rect.width;
    const int ph = panel_rect.height;
    cv::Mat panel = img(panel_rect);

    static constexpr float kFwdMax = 150.f;   // m of range shown (in-path search)
    static constexpr float kLatMax = 14.f;    // m either side shown
    static constexpr float kPathZoneM = 1.8f; // same lateral gate as fusion
    const int   y_ego  = ph - 16;
    const int   cx     = pw / 2;
    const float ppm_x  = static_cast<float>(y_ego - 30) / kFwdMax;
    const float ppm_y  = (static_cast<float>(pw) * 0.5f - 6.f) / kLatMax;

    auto to_px = [&](float x, float y) {
        return cv::Point(cx - static_cast<int>(std::lround(y * ppm_y)),
                         y_ego - static_cast<int>(std::lround(x * ppm_x)));
    };
    auto in_panel = [&](const cv::Point& p) {
        return p.x >= 1 && p.x < pw - 1 && p.y >= 22 && p.y < ph - 1;
    };

    cv::putText(panel, "x 0-150m  y +/-14m  zone +/-1.8m", cv::Point(6, 27),
                kFont, 0.28, cv::Scalar(120, 120, 120), 1, cv::LINE_AA);

    // Range rings every 20 m, lane-ish lateral guides at +/-2 and +/-4 m.
    for (int r = 20; r <= static_cast<int>(kFwdMax); r += 20) {
        const int y = to_px(static_cast<float>(r), 0.f).y;
        if (y < 30) continue;
        cv::line(panel, cv::Point(2, y), cv::Point(pw - 3, y),
                 cv::Scalar(45, 45, 45), 1, cv::LINE_AA);
        cv::putText(panel, std::to_string(r), cv::Point(3, y - 2),
                    kFont, 0.28, cv::Scalar(90, 90, 90), 1, cv::LINE_AA);
    }
    for (float y_off : {-12.f, -8.f, -4.f, 0.f, 4.f, 8.f, 12.f}) {
        const int x = to_px(0.f, y_off).x;
        if (x < 2 || x >= pw - 2) continue;
        const cv::Scalar c = (y_off == 0.f) ? cv::Scalar(60, 60, 60)
                                            : cv::Scalar(40, 40, 40);
        for (int y = 30; y < y_ego; y += 6)
            cv::line(panel, cv::Point(x, y), cv::Point(x, y + 3), c, 1);
    }

    // Fusion snapshot path if present (same poly association used), else lateral.
    const bool have_path = radar.path.valid || lat.path_valid;
    const float pa = radar.path.valid ? radar.path.a : lat.path_a;
    const float pb = radar.path.valid ? radar.path.b : lat.path_b;
    const float pc = radar.path.valid ? radar.path.c : lat.path_c;
    const float x_fit_src = radar.path.valid ? radar.path.x_max_m : lat.path_x_max_m;
    const cv::Scalar zone_clr(200, 90, 40);      // teal: +/-1.8 m search tube
    const cv::Scalar ext_clr(180, 200, 80);      // cyan: extrapolated centreline
    const cv::Scalar cipo_clr(0, 255, 100);
    const cv::Scalar mov_in_clr(0, 140, 255);    // orange
    const cv::Scalar mov_out_clr(0, 70, 140);
    const cv::Scalar stat_in_clr(200, 80, 200);  // magenta
    const cv::Scalar stat_out_clr(90, 90, 90);

    // Fitted path (solid yellow) vs extended in-path search to 150 m
    // (dashed cyan) with the +/-1.8 m association corridor (teal).
    if (have_path) {
        const float x_fit = (x_fit_src > 1.f) ? std::min(x_fit_src, kFwdMax) : 0.f;
        std::vector<cv::Point> mid_fit, mid_ext, lo, hi;
        for (float x = 0.f; x <= kFwdMax; x += 1.f) {
            const float y = pa * x * x + pb * x + pc;
            if (std::abs(y) > kLatMax) continue;
            const cv::Point pm = to_px(x, y);
            if (!in_panel(pm)) continue;
            if (x <= x_fit) mid_fit.push_back(pm);
            else            mid_ext.push_back(pm);
            lo.push_back(to_px(x, y - kPathZoneM));
            hi.push_back(to_px(x, y + kPathZoneM));
        }
        if (lo.size() >= 2 && hi.size() >= 2) {
            std::vector<std::vector<cv::Point>> zone{lo};
            zone[0].insert(zone[0].end(), hi.rbegin(), hi.rend());
            cv::Mat tint = panel.clone();
            cv::fillPoly(tint, zone, cv::Scalar(70, 32, 14));
            cv::addWeighted(tint, 0.28, panel, 0.72, 0, panel);
            cv::polylines(panel, lo, false, zone_clr, 1, cv::LINE_AA);
            cv::polylines(panel, hi, false, zone_clr, 1, cv::LINE_AA);
        }
        if (mid_fit.size() >= 2)
            cv::polylines(panel, mid_fit, false, kClrFusedLat, 2, cv::LINE_AA);
        if (!mid_fit.empty() && !mid_ext.empty())
            mid_ext.insert(mid_ext.begin(), mid_fit.back());
        for (std::size_t i = 1; i < mid_ext.size(); ++i) {
            if ((i % 2) == 0) continue;
            cv::line(panel, mid_ext[i - 1], mid_ext[i], ext_clr, 1, cv::LINE_AA);
        }
    }

    // Vision distances as ticks on the path, so radar vs camera is one glance.
    auto path_y_at = [&](float x) {
        return have_path ? pa * x * x + pb * x + pc : 0.f;
    };
    auto tick = [&](float dist, const cv::Scalar& c, const char* tag) {
        if (dist <= 0.f || dist > kFwdMax) return;
        const cv::Point p = to_px(dist, path_y_at(dist));
        if (!in_panel(p)) return;
        cv::line(panel, cv::Point(p.x - 9, p.y), cv::Point(p.x + 9, p.y), c, 1, cv::LINE_AA);
        cv::putText(panel, tag, cv::Point(p.x + 11, p.y + 3), kFont, 0.30, c, 1, cv::LINE_AA);
    };
    if (view.cipo.ad_meas_valid) tick(view.cipo.ad_dist_m, cv::Scalar(0, 220, 0), "AD");
    if (view.cipo.as_h_valid)    tick(view.cipo.as_h_dist_m, cv::Scalar(160, 220, 160), "H");

    // Returns, split by ego-compensated Doppler. Prefer published ego; if
    // missing, fall back to the cloud median for colour only.
    int n_static = 0, n_moving = 0, n_match = 0, n_zone = 0;
    const float ego_pub = view.ego_speed_ms;
    const float ego_est = (ego_pub > 0.5f) ? ego_pub : ego_speed_from_radar(radar.points);
    for (std::size_t i = 0; i < radar.points.size(); ++i) {
        const auto& rp = radar.points[i];
        const float x = rp.range_m * std::cos(rp.azimuth_rad);
        const float y = rp.range_m * std::sin(rp.azimuth_rad);
        const cv::Point p = to_px(x, y);
        const bool is_static =
            std::abs(rp.range_rate + ego_est * std::cos(rp.azimuth_rad)) < 1.0f;
        const bool matched = i < radar.in_match.size() && radar.in_match[i] != 0;
        bool in_zone = false;
        if (have_path && x >= 0.5f && x <= kFwdMax) {
            const float py = pa * x * x + pb * x + pc;
            in_zone = std::abs(y - py) <= kPathZoneM;
        }
        if (is_static) ++n_static; else ++n_moving;
        if (matched) ++n_match;
        if (in_zone) ++n_zone;
        if (!in_panel(p)) continue;
        cv::Scalar c = is_static
            ? (in_zone ? stat_in_clr : stat_out_clr)
            : (in_zone ? mov_in_clr  : mov_out_clr);
        if (matched) c = cipo_clr;
        cv::circle(panel, p, matched ? 3 : 2, c, -1, cv::LINE_AA);
    }

    // Ring sits on the search path at the reported match range so the label
    // cannot drift onto a random cluster member.
    if (n_match > 0 && radar.match_range_m > 0.f) {
        const cv::Point p = to_px(radar.match_range_m, path_y_at(radar.match_range_m));
        if (in_panel(p)) {
            cv::circle(panel, p, 7, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.1fm %+.1f", radar.match_range_m,
                          radar.match_rate_ms);
            cv::putText(panel, buf, cv::Point(p.x + 9, p.y - 6),
                        kFont, 0.30, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
    }

    const cv::Point ego = to_px(0.f, 0.f);
    cv::circle(panel, ego, 3, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);

    char foot[112];
    if (ego_pub > 0.5f)
        std::snprintf(foot, sizeof(foot),
                      "ego=%.1f  pts %zu  zone %d  match %d  stat %d  mov %d",
                      ego_pub, radar.points.size(), n_zone, n_match, n_static, n_moving);
    else
        std::snprintf(foot, sizeof(foot),
                      "ego=--  pts %zu  zone %d  match %d  stat %d  mov %d",
                      radar.points.size(), n_zone, n_match, n_static, n_moving);
    cv::putText(panel, foot, cv::Point(4, ph - 4), kFont, 0.30,
                cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
}

static void draw_bev_fused_ego_path(cv::Mat& img,
                                     const DebugView& view,
                                     const OverlayLayout& L)
{
    const auto& lat = view.lateral;
    const auto& radar = view.cipo.radar;
    if (!lat.path_valid && !radar.enabled) return;

    if (radar.enabled) { draw_radar_bev_panel(img, view, L); return; }

    const cv::Rect panel_rect = L.bev & cv::Rect(0, 0, img.cols, img.rows);
    if (panel_rect.width < 40 || panel_rect.height < 40) return;

    draw_panel(img, panel_rect,
               radar.enabled ? "RADAR BEV" : "FUSED PATH (top-down)",
               kClrFusedLat);

    const int pw = panel_rect.width;
    const int ph = panel_rect.height;
    cv::Mat panel = img(panel_rect);

    const float px_per_m = radar.enabled
        ? std::min((ph - 16.f) / 150.f, (pw * 0.5f) / 40.f)
        : 4.5f;
    const float lat_max = radar.enabled ? 40.f : 7.f;
    // Never extrapolate the poly past the inlier span — same as original VisionPilot.
    const float x_end = (lat.path_x_max_m > lat.path_x_min_m + 1.f)
        ? lat.path_x_max_m : 0.f;

    auto to_px = [&](float x, float y) {
        return world_to_bev_px(x, y, pw, ph, px_per_m);
    };

    if (lat.path_valid && x_end >= 2.f) {
        std::vector<cv::Point> poly;
        for (float x = 0.f; x <= x_end; x += 1.f) {
            const float y = lat.path_a * x * x + lat.path_b * x + lat.path_c;
            if (std::abs(y) > lat_max) continue;
            const cv::Point p = to_px(x, y);
            if (p.x < 2 || p.x >= pw - 2 || p.y < 20 || p.y >= ph - 2) continue;
            poly.push_back(p);
        }
        if (poly.size() >= 2)
            cv::polylines(panel, poly, false, kClrFusedLat, 2, cv::LINE_AA);
    }

    const cv::Point ego = to_px(0.f, 0.f);
    cv::circle(panel, ego, 4, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
    cv::putText(panel, "ego", cv::Point(ego.x + 6, ego.y + 4),
                kFont, 0.32, kClrNormal, 1, cv::LINE_AA);
}

// Back-project fused polynomial onto camera image via H⁻¹ (world → pixels).
static void draw_fused_path_on_image(cv::Mat& img,
                                      const fusion::LateralFusionEstimate& lat,
                                      const cv::Mat& H_inv)
{
    if (!lat.path_valid || H_inv.empty()) return;

    const float x_start = std::max(0.f, lat.path_x_min_m);
    const float x_end   = lat.path_x_max_m;
    if (x_end <= x_start + 0.5f) return;

    const float a = lat.path_a, b = lat.path_b, c = lat.path_c;
    std::vector<cv::Point2f> world_pts;
    for (float x = x_start; x <= x_end; x += 1.5f) {
        const float y = a * x * x + b * x + c;
        if (std::abs(y) > 12.f) continue;
        world_pts.emplace_back(x, y);
    }
    if (world_pts.size() < 2) return;

    std::vector<cv::Point2f> img_pts;
    cv::perspectiveTransform(world_pts, img_pts, H_inv);

    std::vector<cv::Point> poly;
    for (const auto& p : img_pts) {
        const int u = static_cast<int>(p.x);
        const int v = static_cast<int>(p.y);
        if (u < 0 || u >= img.cols || v < 0 || v >= img.rows) continue;
        poly.emplace_back(u, v);
    }
    if (poly.size() >= 2) {
        cv::polylines(img, poly, false, kClrFusedLat, 2, cv::LINE_AA);
        // Tag near the closest-to-ego (largest v) visible point
        cv::Point tag_pt = poly[0];
        for (const auto& p : poly)
            if (p.y > tag_pt.y) tag_pt = p;
        draw_tag(img, cv::Point(tag_pt.x + 8, tag_pt.y - 8), "Fused path", kClrFusedLat);
    }
}

static void draw_autosteer_ego_path(cv::Mat& img,
                                     const models::AutoSteerOutput& steer)
{
    if (!steer.valid) return;

    const int W = img.cols > 0 ? img.cols : kNetW;
    const int H = img.rows > 0 ? img.rows : kNetH;

    // Fixed row indices — same as np.linspace(0, 511, 64)
    int y_pts[kPathPts];
    for (int i = 0; i < kPathPts; ++i)
        y_pts[i] = (kPathPts <= 1) ? 0
                   : static_cast<int>(std::lround(static_cast<double>(i) * (H - 1) / (kPathPts - 1)));

    std::vector<cv::Point> poly;
    for (int i = 0; i < kPathPts; ++i) {
        if (steer.h_vector[i] < 0.5f) continue;

        const int u = static_cast<int>(steer.xp[i] * static_cast<float>(W));
        const int v = y_pts[i];
        if (u < 0 || u >= W || v < 0 || v >= H) continue;

        const cv::Point pt(u, v);
        cv::circle(img, pt, 3, kClrEgoPath, -1, cv::LINE_AA);
        poly.push_back(pt);
    }
    if (poly.size() >= 2) {
        cv::polylines(img, poly, false, kClrEgoPath, 2, cv::LINE_AA);
        cv::Point tag_pt = poly[0];
        for (const auto& p : poly)
            if (p.y > tag_pt.y) tag_pt = p;
        draw_tag(img, cv::Point(tag_pt.x - 90, tag_pt.y - 8), "AutoSteer", kClrEgoPath);
    }
}

// ─── Steering wheels from curvature (image_visualization.py) ─────────────────

static void draw_steering_wheels(cv::Mat& img, const DebugView& v)
{
    const auto& veh = v.vehicle;

    float ad_curv = 0.f, ad_steer = 0.f;
    if (v.auto_drive.valid) {
        ad_curv   = v.auto_drive.curvature_raw * veh.curv_scale;
        ad_steer  = curvature_to_steer_deg(ad_curv, veh.wheelbase_m, veh.steer_ratio);
    }

    float fused_curv = 0.f, fused_steer = 0.f;
    if (v.lateral.valid) {
        fused_curv  = v.lateral.curvature;
        fused_steer = curvature_to_steer_deg(fused_curv, veh.wheelbase_m, veh.steer_ratio);
    }

    const int pad = 8;
    const int y   = kTopBarH + 6;
    // Wheels sit in the strip between top bar and BEV panel (top-center-right)
    const int x_ad    = img.cols - pad - kWheelPx;
    const int x_fused = x_ad - pad - kWheelPx;

    std::lock_guard<std::mutex> lock(g_wheel_mu);
    if (!g_wheels_tried)
        init_wheel_assets(v.wheel_dir);

    if (!g_wheel_green.empty() && v.lateral.valid)
        paste_rgba(img, rotate_wheel_rgba(g_wheel_green, fused_steer), x_fused, y);
    if (!g_wheel_white.empty() && v.auto_drive.valid)
        paste_rgba(img, rotate_wheel_rgba(g_wheel_white, ad_steer), x_ad, y);

    // Small captions under wheels only (details in legend + HUD)
    if (v.lateral.valid)
        cv::putText(img, "fused", cv::Point(x_fused + 18, y + kWheelPx + 12),
                    kFont, 0.36, kClrSteerLbl, 1, cv::LINE_AA);
    if (v.auto_drive.valid)
        cv::putText(img, "AD", cv::Point(x_ad + 34, y + kWheelPx + 12),
                    kFont, 0.36, kClrAdWheel, 1, cv::LINE_AA);
}

// ─── HUD panel ───────────────────────────────────────────────────────────────

static void draw_top_bar(cv::Mat& img, const DebugView& v)
{
    fill_rect(img, cv::Rect(0, 0, img.cols, kTopBarH), kClrHudBg, 0.65);
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "VisionPilot  #%llu  wall=%.1f ms (%.0f fps)  pre=%.1f ms  src=%s",
                  static_cast<unsigned long long>(v.frame_id),
                  v.wall_ms, (v.wall_ms > 0) ? 1000.0 / v.wall_ms : 0.0,
                  v.pre_ms, v.src_label.c_str());
    cv::putText(img, buf, cv::Point(6, 14), kFont, kSmall, kClrTopBar, kThin, cv::LINE_AA);
}

static void draw_hud_panel(cv::Mat& img, const DebugView& v, const OverlayLayout& L)
{
    const int W  = img.cols;
    const int py = L.hud_y;

    fill_rect(img, cv::Rect(0, py, W, kHudH), kClrHudBg, 0.85);
    cv::line(img, cv::Point(0, py), cv::Point(W, py), kClrHudLine, 1, cv::LINE_AA);

    const int col_w = W / 3;
    const int c1x = 10, c2x = col_w + 10, c3x = 2 * col_w + 10;
    cv::line(img, cv::Point(col_w, py + 4), cv::Point(col_w, py + kHudH - 4),
             kClrHudLine, 1, cv::LINE_AA);
    cv::line(img, cv::Point(2 * col_w, py + 4), cv::Point(2 * col_w, py + kHudH - 4),
             kClrHudLine, 1, cv::LINE_AA);

    const int lineH = 17;
    auto text = [&](int x, int y, const std::string& s, cv::Scalar clr,
                    double scale = kSmall, int thickness = kThin) {
        cv::putText(img, s, cv::Point(x, y), kFont, scale, clr, thickness, cv::LINE_AA);
    };

    static constexpr float D_MAX = 150.f;
    const auto& veh = v.vehicle;

    // ── Column 1: per-source longitudinal ────────────────────────────────────
    int y = py + 16;
    text(c1x, y, "LONG MEASUREMENTS", kClrHeader); y += lineH;

    if (v.ego_speed_ms > 0.5f)
        text(c1x, y, "Ego    " + fd(v.ego_speed_ms, 2) + " m/s", kClrFusedLong);
    else
        text(c1x, y, "Ego    (none)", kClrNormal);
    y += lineH;

    if (v.auto_drive.valid) {
        const float d_m = v.cipo.ad_meas_valid ? v.cipo.ad_dist_m
                        : D_MAX * (1.f - v.auto_drive.dist_normalized);
        text(c1x, y, "AD     " + fd(d_m, 1) + " m  p=" + fd(v.auto_drive.flag_prob, 2),
             v.cipo.ad_meas_valid ? kClrFusedLong : kClrNormal); y += lineH;
    } else {
        text(c1x, y, "AD     (none)", kClrNormal); y += lineH;
    }
    if (v.cipo.as_h_valid)
        text(c1x, y, "AS+H   " + fd(v.cipo.as_h_dist_m, 1) + " m", kClrFusedLong);
    else
        text(c1x, y, "AS+H   (none)", kClrNormal);
    y += lineH;
    if (v.cipo.radar_meas_valid) {
        char rad[64];
        std::snprintf(rad, sizeof(rad), "Radar  %.1f m  v=%+.2f  S%d",
                      static_cast<double>(v.cipo.radar_dist_m),
                      static_cast<double>(v.cipo.radar_vel_ms),
                      v.cipo.radar_scenario);
        text(c1x, y, rad, kClrFusedLong);
    } else if (v.cipo.radar.enabled) {
        text(c1x, y, "Radar  (no match)", kClrNormal);
    } else {
        text(c1x, y, "Radar  (off)", kClrNormal);
    }
    y += lineH;
    if (v.auto_drive.valid) {
        const float curv = v.auto_drive.curvature_raw * veh.curv_scale;
        const int flag = (v.auto_drive.flag_prob >= veh.flag_threshold) ? 1 : 0;
        text(c1x, y, "CIPO flag " + std::to_string(flag)
             + "  curv " + fd(curv, 4), kClrNormal);
    }

    // ── Column 2: fused longitudinal ─────────────────────────────────────────
    y = py + 16;
    text(c2x, y, "FUSED LONGITUDINAL", kClrFusedLong); y += lineH;
    if (v.cipo.valid) {
        text(c2x, y, "distance  " + fd(v.cipo.distance_m, 1) + " m",
             kClrFusedLong, kNorm, kBold); y += lineH;
        text(c2x, y, "velocity  " + fd(v.cipo.velocity_ms, 2) + " m/s",
             kClrFusedLong, kNorm, kBold); y += lineH;
        text(c2x, y, "uncert.   +/-" + fd(v.cipo.distance_stddev_m, 1) + " m",
             kClrNormal);
        if (v.cipo.cut_in_detected && v.cipo.radar_scenario == 1)
            text(c2x, py + kHudH - 10, "CUT-IN", kClrL2, kSmall, kBold);
        else if (v.cipo.radar_scenario == 2)
            text(c2x, py + kHudH - 10, "S2 L3 ON PATH", kClrL3, kSmall, kBold);
        else if (v.cipo.radar_scenario == 3)
            text(c2x, py + kHudH - 10, "S3 PATH MOVING", kClrNormal, kSmall, kBold);
        else if (v.cipo.radar_scenario == 1)
            text(c2x, py + kHudH - 10, "S1 L1/L2 RADAR", kClrFusedLong, kSmall, kBold);
    } else {
        text(c2x, y, "(waiting for tracker)", kClrNormal);
    }

    // ── Column 3: fused lateral ────────────────────────────────────────────────
    y = py + 16;
    text(c3x, y, "FUSED LATERAL", kClrFusedLat); y += lineH;
    if (v.lateral.valid) {
        text(c3x, y, "CTE       " + fd(v.lateral.cte_m, 2) + " m  "
                     + fd(v.lateral.cte_rate_mps, 2) + " m/s",
             kClrFusedLat, kNorm, kBold); y += lineH;
        text(c3x, y, "yaw       " + fd(v.lateral.yaw_rad, 3) + " rad  "
                     + fd(v.lateral.yaw_rate_rps, 3) + " rad/s",
             kClrFusedLat, kNorm, kBold); y += lineH;
        text(c3x, y, "curvature " + fd(v.lateral.curvature, 4) + " 1/m",
             kClrFusedLat, kNorm, kBold);
        if (v.lateral.path_valid) {
            char path_buf[64];
            std::snprintf(path_buf, sizeof(path_buf), "path fit  %d inliers / %d pts",
                          v.lateral.path_inliers, v.lateral.path_points);
            text(c3x, y + lineH, path_buf, kClrNormal, 0.36);
        }
    } else {
        text(c3x, y, "(no path / fusion)", kClrNormal);
    }
}

// ─── Public ───────────────────────────────────────────────────────────────────

void annotate_frame(cv::Mat& frame, const DebugView& view,
                    const cv::Mat& H_world_to_px)
{
    // Resolve which H to use for fused-path back-projection:
    //   • if caller supplies one (resized-frame mode), use it directly.
    //   • otherwise fall back to the hardcoded warped-BEV inverse.
    cv::Mat H_use = H_world_to_px;
    if (H_use.empty()) {
        std::lock_guard<std::mutex> lock(g_homo_mu);
        if (!g_homo_tried) {
            g_homo_tried     = true;
            g_H_world_to_img = load_homography_inv();
        }
        H_use = g_H_world_to_img;
    }

    const OverlayLayout layout = layout_for(frame, view.cipo.radar.enabled);

    // Scene overlays (paths, boxes) — drawn first
    draw_autospeed_detections(frame, view.auto_speed);
    draw_autosteer_ego_path(frame, view.auto_steer);
    draw_fused_path_on_image(frame, view.lateral, H_use);

    // Chrome: legend, BEV, wheels, bars (on top, fixed zones)
    draw_legend(frame, layout, view);
    draw_bev_fused_ego_path(frame, view, layout);
    draw_steering_wheels(frame, view);
    draw_top_bar(frame, view);
    draw_hud_panel(frame, view, layout);
}

bool visualize(cv::Mat& frame,
               const models::InferenceFrameResult& result,
               const std::string& src_label,
               const std::string& wheel_dir,
               const cv::Mat& H_world_to_px,
               float ego_speed_ms)
{
    if (frame.empty()) return false;
    auto dv = debug_view_from(result, src_label, wheel_dir, ego_speed_ms);
    annotate_frame(frame, dv, H_world_to_px);
    cv::namedWindow("VisionPilot", cv::WINDOW_NORMAL);
    cv::resizeWindow("VisionPilot", frame.cols, frame.rows);
    cv::imshow("VisionPilot", frame);
    cv::waitKey(1);
    return true;
}

}  // namespace visionpilot::debug
