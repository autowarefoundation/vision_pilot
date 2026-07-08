#include <visualization/visualization.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>
#include <visualization/webrtc_stream.hpp>

#include "visualization/local_display.hpp"

namespace visualization {

namespace fs = std::filesystem;

// ─── Layout constants ─────────────────────────────────────────────────────────
static constexpr int   kNetW     = 1024;
static constexpr int   kNetH     = 512;
static constexpr int   kPathPts  = 64;
static constexpr float kDMax     = 150.f;
static constexpr int   kIconPx   = 80;   // default icon display size (px)
static constexpr int   kHudFont  = cv::FONT_HERSHEY_DUPLEX;

// ─── Warped-pixel → world homography (copy of LongitudinalFusion H_) ──────────
// Input:  pixel (u, v) in the 1024×512 warped BEV image
// Output: world (x_fwd, y_lat) in metres; +x ahead, +y left of ego
static cv::Mat make_H_warp_to_world() {
    return (cv::Mat_<double>(3, 3) <<
         0.00209514907,  -0.000941721466, -9.24906396,
         0.00662758637,  -0.000352940531, -3.33396502,
         0.000120077371, -0.00411343505,   1.0);
}

// ─── Global state (single-threaded render loop — no mutex needed) ─────────────
static bool    g_h_ready        = false;
static cv::Mat g_H_warp2world;   // float32 3×3
static cv::Mat g_H_world2px;     // float32 3×3  (inverse of above)

static bool    g_ico_ready      = false;
static cv::Mat g_icon_brake;
static cv::Mat g_icon_collision;
static cv::Mat g_icon_rld;       // right lane departure  (BGRA, kIconPx²)
static cv::Mat g_icon_lld;       // left  lane departure  (BGRA, horizontally flipped)

// ─── Helper: alpha-composite BGRA icon centred at (cx, cy) onto BGR base ──────
static void paste_icon(cv::Mat& base, const cv::Mat& icon, int cx, int cy, int px = 0) {
    if (icon.empty() || base.empty()) return;

    cv::Mat draw_icon = icon;
    if (px > 0 && (icon.cols != px || icon.rows != px))
        cv::resize(icon, draw_icon, cv::Size(px, px), 0, 0, cv::INTER_AREA);

    const int x  = cx - draw_icon.cols / 2;
    const int y  = cy - draw_icon.rows / 2;
    const int x1 = std::max(x, 0);
    const int y1 = std::max(y, 0);
    const int x2 = std::min(x + draw_icon.cols, base.cols);
    const int y2 = std::min(y + draw_icon.rows, base.rows);
    if (x2 <= x1 || y2 <= y1) return;

    const cv::Rect src_rect(x1 - x, y1 - y, x2 - x1, y2 - y1);
    cv::Mat roi = base(cv::Rect(x1, y1, x2 - x1, y2 - y1));
    cv::Mat src = draw_icon(src_rect);

    if (src.channels() == 4) {
        std::vector<cv::Mat> ch;
        cv::split(src, ch);
        cv::Mat rgb, a_f, a3;
        cv::merge(std::vector<cv::Mat>{ch[0], ch[1], ch[2]}, rgb);
        ch[3].convertTo(a_f, CV_32F, 1.0 / 255.0);
        cv::cvtColor(a_f, a3, cv::COLOR_GRAY2BGR);
        cv::Mat rf, bf;
        rgb.convertTo(rf, CV_32FC3, 1.0 / 255.0);
        roi.convertTo(bf, CV_32FC3, 1.0 / 255.0);
        cv::Mat blended = rf.mul(a3) + bf.mul(cv::Scalar(1.0, 1.0, 1.0) - a3);
        blended.convertTo(roi, CV_8UC3, 255.0);
    } else {
        src.copyTo(roi);
    }
}

// ─── Helper: semi-transparent filled rectangle ────────────────────────────────
static void fill_rect_alpha(cv::Mat& img, cv::Rect r, cv::Scalar color, double alpha) {
    r &= cv::Rect(0, 0, img.cols, img.rows);
    if (r.width <= 0 || r.height <= 0) return;
    cv::Mat roi = img(r);
    cv::Mat block(roi.size(), roi.type(), color);
    cv::addWeighted(block, alpha, roi, 1.0 - alpha, 0.0, roi);
}

// ─── Helper: icon loader ──────────────────────────────────────────────────────
static cv::Mat load_icon(const fs::path& p, int px) {
    if (!fs::exists(p)) return {};
    cv::Mat img = cv::imread(p.string(), cv::IMREAD_UNCHANGED);
    if (img.empty()) return {};
    cv::resize(img, img, cv::Size(px, px), 0, 0, cv::INTER_AREA);
    if (img.channels() == 3) cv::cvtColor(img, img, cv::COLOR_BGR2BGRA);
    return img;
}

// ─── Helper: locate assets folder ────────────────────────────────────────────
static std::string resolve_icons_dir(const std::string& hint) {
    if (!hint.empty() && fs::is_directory(hint)) return hint;
    for (const char* c : {"../assets/icons", "assets/icons", "VisionPilot/assets/icons", "/usr/share/visionpilot/assets/icons"}) {
        if (fs::is_directory(c)) return std::string(c);
    }
    return {};
}

// ─── Helper: pick solid corridor color from acceleration ──────────────────────
// Color schema (BGR):
//   acc < -5          hard brake  → (93, 0, 255)   rgb(255,0,93)
//   -5 ≤ acc ≤ -3     moderate    → (0, 102, 255)   rgb(255,102,0)
//   -3 < acc ≤ -1     light brake → (0, 213, 255)   rgb(255,213,0)
//   acc > -1          cruise/accel→ (174, 255, 0)   rgb(0,255,174)
static cv::Scalar path_color(double acc) {
    if      (acc < -5.0) return cv::Scalar( 93,   0, 255);
    else if (acc < -3.0) return cv::Scalar(  0, 102, 255);
    else if (acc < -1.0) return cv::Scalar(  0, 213, 255);
    else                 return cv::Scalar(174, 255,   0);
}


// ─── Public: init_production_assets ──────────────────────────────────────────
void init_production_assets(const std::string& icons_dir) {
    if (!g_h_ready) {
        cv::Mat H64 = make_H_warp_to_world();
        H64.convertTo(g_H_warp2world, CV_32F);
        g_H_world2px = g_H_warp2world.inv();
        g_h_ready = true;
    }

    if (g_ico_ready) return;
    g_ico_ready = true;

    const std::string dir = resolve_icons_dir(icons_dir);
    if (dir.empty()) return;

    g_icon_brake     = load_icon(fs::path(dir) / "brake.png",               kIconPx);
    g_icon_collision = load_icon(fs::path(dir) / "collision.png",            kIconPx);
    g_icon_rld       = load_icon(fs::path(dir) / "right_lane_departure.png", kIconPx);

    if (!g_icon_rld.empty())
        cv::flip(g_icon_rld, g_icon_lld, 1);   // horizontal flip → left departure

}

// ─── Draw: fused-path corridor in world space (±1 m, gradient fill) ──────────
//
// Algorithm:
//   1. LateralFusion RANSAC gives polynomial y = a·x² + b·x + c in world frame.
//   2. Sample at 0.5 m steps over [path_x_min_m, path_x_max_m].
//   3. Left boundary = (x, y_center + 1 m), right = (x, y_center − 1 m).
//   4. Project both boundaries through H_world2px → warped image pixels.
//   5. Fill corridor as gradient trapezoids from far (small pixel-v) → near.
//
// The polynomial is already in metric world space, so there is no pixel→world
// projection step.  The ±1 m corridor stays exactly 2 m wide regardless of road
// curvature — it correctly fans out in image-space on the inner side of bends.
static void draw_path_corridor(cv::Mat& img, const ProductionView& view) {
    if (!view.path_valid) return;
    if (view.path_x_max_m <= view.path_x_min_m + 1.f) return;

    // Use the per-view H (from H_resized) when set; fall back to global warped H.
    const cv::Mat& H_w2px = (!view.H_world2px.empty()) ? view.H_world2px
                                                        : g_H_world2px;
    if (H_w2px.empty() && !g_h_ready) return;

    auto world_to_px = [&](float xw, float yw) -> cv::Point {
        std::vector<cv::Point2f> s = {cv::Point2f(xw, yw)}, d;
        cv::perspectiveTransform(s, d, H_w2px);
        return cv::Point(static_cast<int>(std::lround(d[0].x)),
                         static_cast<int>(std::lround(d[0].y)));
    };

    // ── Sample polynomial; build left/right boundary pixel arrays ─────────────
    const float a = view.path_a, b = view.path_b, c = view.path_c;
    const float x_start = std::max(0.5f, view.path_x_min_m);
    const float x_end   = view.path_x_max_m;

    constexpr float kStep = 0.5f;
    std::vector<cv::Point> lp, rp;

    for (float x = x_start; x <= x_end + 0.01f; x += kStep) {
        const float yc = a * x * x + b * x + c;
        lp.push_back(world_to_px(x, yc + 1.f));   // +1 m left of ego path
        rp.push_back(world_to_px(x, yc - 1.f));   // −1 m right of ego path
    }

    const int n = static_cast<int>(lp.size());
    if (n < 2) return;

    // ── Solid fill with single acceleration-derived color, semi-transparent ─────
    const cv::Scalar color = path_color(view.acceleration);
    cv::Mat overlay = img.clone();
    const cv::Rect bounds(0, 0, img.cols, img.rows);

    for (int i = 0; i < n - 1; ++i) {
        const std::vector<cv::Point> quad = {lp[i], rp[i], rp[i + 1], lp[i + 1]};
        bool any_in = false;
        for (const auto& p : quad)
            if (bounds.contains(p)) { any_in = true; break; }
        if (!any_in) continue;
        cv::fillConvexPoly(overlay, quad, color, cv::LINE_AA);
    }

    cv::addWeighted(overlay, 0.35, img, 0.65, 0.0, img);
}

// AutoSpeed bbox colors — match debug_draw.cpp det_color()
static cv::Scalar autospeed_det_color(int class_id) {
    switch (class_id) {
        case 1: return cv::Scalar(0, 0, 220);     // L1 red
        case 2: return cv::Scalar(0, 210, 255);   // L2 yellow
        case 3: return cv::Scalar(220, 200, 0);   // L3 blue
        default: return cv::Scalar(80, 200, 80);  // other green
    }
}

static cv::Scalar autospeed_det_fill(int class_id) {
    const cv::Scalar c = autospeed_det_color(class_id);
    return cv::Scalar(c[0] * 0.35, c[1] * 0.35, c[2] * 0.35);
}

// ─── Draw: AutoSpeed bounding boxes + CIPO distance label ─────────────────────
static void draw_cipo_boxes(cv::Mat& img, const ProductionView& view) {
    if (view.detections.empty()) return;

    // Semi-transparent fill pass
    cv::Mat overlay = img.clone();
    for (const auto& d : view.detections) {
        cv::rectangle(overlay,
            cv::Point(static_cast<int>(d.x1), static_cast<int>(d.y1)),
            cv::Point(static_cast<int>(d.x2), static_cast<int>(d.y2)),
            autospeed_det_fill(d.class_id), -1);
    }
    cv::addWeighted(overlay, 0.32, img, 0.68, 0.0, img);

    // Outline pass (hard edges on top)
    for (const auto& d : view.detections) {
        cv::rectangle(img,
            cv::Point(static_cast<int>(d.x1), static_cast<int>(d.y1)),
            cv::Point(static_cast<int>(d.x2), static_cast<int>(d.y2)),
            autospeed_det_color(d.class_id), 2, cv::LINE_AA);
    }

    // ── Distance label on the closest L1 (most centred) ──────────────────────
    if (view.cipo.valid && view.cipo.distance_m > 0.f) {
        const ProductionView::BBox* best = nullptr;
        float best_cx_err = 1e9f;
        for (const auto& d : view.detections) {
            if (d.class_id != 1) continue;
            const float cx_err = std::abs((d.x1 + d.x2) * 0.5f - kNetW * 0.5f);
            if (cx_err < best_cx_err) { best_cx_err = cx_err; best = &d; }
        }

        if (best) {
            char lbl[32];
            std::snprintf(lbl, sizeof(lbl), "%.0fm",
                static_cast<double>(view.cipo.distance_m));

            const int tx = static_cast<int>((best->x1 + best->x2) * 0.5f);
            const int ty = static_cast<int>(best->y1) - 6;
            if (ty > 12) {
                int bl = 0;
                const cv::Size ts = cv::getTextSize(
                    lbl, cv::FONT_HERSHEY_SIMPLEX, 0.55, 2, &bl);
                cv::rectangle(img,
                    cv::Point(tx - ts.width / 2 - 6, ty - ts.height - 6),
                    cv::Point(tx + ts.width / 2 + 6, ty + 4),
                    cv::Scalar(20, 20, 20), -1);
                cv::putText(img, lbl,
                    cv::Point(tx - ts.width / 2, ty),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            }
        }
    }
}

// ─── Draw: ego speed readout (top-centre) ────────────────────────────────────
static void draw_speed(cv::Mat& img, double speed_ms) {
    char num_buf[32];
    std::snprintf(num_buf, sizeof(num_buf), "%.0f", speed_ms * 2.23694);
    const char* unit_buf = "mph";

    int bl = 0;
    const cv::Size num_ts  = cv::getTextSize(num_buf,  cv::FONT_HERSHEY_DUPLEX, 1.1, 2, &bl);
    const cv::Size unit_ts = cv::getTextSize(unit_buf, cv::FONT_HERSHEY_DUPLEX, 0.5, 1, &bl);

    const int num_tx  = (img.cols - num_ts.width)  / 2;
    const int unit_tx = (img.cols - unit_ts.width) / 2;
    const int num_ty  = 44;
    const int unit_ty = num_ty + num_ts.height -4 ;

    cv::Mat overlay;
    img.copyTo(overlay);

    cv::putText(overlay, num_buf, cv::Point(num_tx + 2, num_ty + 2),
                cv::FONT_HERSHEY_DUPLEX, 1.1, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(overlay, num_buf, cv::Point(num_tx, num_ty),
                cv::FONT_HERSHEY_DUPLEX, 1.1, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    cv::putText(overlay, unit_buf, cv::Point(unit_tx + 2, unit_ty + 2),
                cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
    cv::putText(overlay, unit_buf, cv::Point(unit_tx, unit_ty),
                cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    cv::addWeighted(overlay, 1.0, img, 0.5, 0, img);
}

// ─── Helper: production HUD text (Duplex + subtle shadow) ────────────────────
static void draw_hud_text(cv::Mat& img, const std::string& text, cv::Point origin,
                          double scale, cv::Scalar color, int thickness = 1) {
    cv::putText(img, text, origin + cv::Point(1, 1), kHudFont, scale,
                cv::Scalar(0, 0, 0), thickness + 1, cv::LINE_AA);
    cv::putText(img, text, origin, kHudFont, scale, color, thickness, cv::LINE_AA);
}

static void draw_hud_text_centered(cv::Mat& img, const std::string& text, int cx,
                                   int baseline_y, double scale, cv::Scalar color,
                                   int thickness = 1) {
    int bl = 0;
    const cv::Size ts = cv::getTextSize(text, kHudFont, scale, thickness, &bl);
    draw_hud_text(img, text, cv::Point(cx - ts.width / 2, baseline_y),
                  scale, color, thickness);
}

// ─── Draw: alert overlays + icons ────────────────────────────────────────────
static bool has_warning(const std::vector<uint8_t>& ws, uint8_t v) {
    for (auto w : ws) if (w == v) return true;
    return false;
}

static void draw_alerts(cv::Mat& img, const ProductionView& view) {
    const int W  = img.cols;
    const int H  = img.rows;
    const int sw = W * 14 / 100;

    const bool fcw  = has_warning(view.warnings, 1);
    const bool aeb  = has_warning(view.warnings, 2);
    const bool lldw = has_warning(view.warnings, 3);
    const bool rldw = has_warning(view.warnings, 4);

    static const cv::Scalar kOrange{0, 120, 255};
    static const cv::Scalar kWhite {255, 255, 255};
    static constexpr double kAlertFont = 0.50;

    auto draw_side_alert = [&](bool left, const cv::Mat& icon,
                               const char* line1, const char* line2) {
        const int cx = left ? sw / 2 : W - sw / 2;
        const int x0 = left ? 0 : W - sw;
        fill_rect_alpha(img, cv::Rect(x0, 0, sw, H), kOrange, 0.45);

        const int icon_px = std::min(52, sw - 20);
        const int icon_cy = H * 36 / 100;
        paste_icon(img, icon, cx, icon_cy, icon_px);

        const int icon_bottom = icon_cy + icon_px / 2;
        const int line1_y = icon_bottom + 22;
        const int line2_y = line1_y + 20;
        draw_hud_text_centered(img, line1, cx, line1_y, kAlertFont, kWhite);
        draw_hud_text_centered(img, line2, cx, line2_y, kAlertFont, kWhite);
    };

    auto draw_bottom_alert = [&](const cv::Mat& icon, const char* label,
                                 cv::Scalar bg, double bg_alpha, int strip_pct) {
        const int bh = H * strip_pct / 100;
        const int y0 = H - bh;
        fill_rect_alpha(img, cv::Rect(0, y0, W, bh), bg, bg_alpha);

        int bl = 0;
        const cv::Size ts = cv::getTextSize(label, kHudFont, kAlertFont, 1, &bl);
        const int text_baseline = H - std::max(10, bh / 10);
        const int text_top = text_baseline - ts.height;

        const int icon_px = std::min(46, std::max(28, (text_top - y0 - 14) * 2));
        const int icon_cy = y0 + (text_top - y0) / 2;

        paste_icon(img, icon, W / 2, icon_cy, icon_px);
        draw_hud_text_centered(img, label, W / 2, text_baseline, kAlertFont, kWhite);
    };

    if (lldw)
        draw_side_alert(true,  g_icon_lld, "Left Lane",  "Departure");
    if (rldw)
        draw_side_alert(false, g_icon_rld, "Right Lane", "Departure");

    if (aeb)
        draw_bottom_alert(g_icon_brake, "Emergency Braking",
                          cv::Scalar(0, 0, 180), 0.50, 22);
    else if (fcw)
        draw_bottom_alert(g_icon_collision, "Collision Alert",
                          cv::Scalar(0, 130, 230), 0.45, 18);
}

// ─── Draw: top-to-bottom gradient black vignette (Comma AI style) ────────────
// Darkens the top 45% of the frame from 60% black opacity at the very top
// down to fully transparent at the fade boundary.  Vectorized: converts the
// affected rows to float32, scales each row's brightness by (1 - alpha), then
// converts back — no per-pixel loop, uses OpenCV SIMD internally.
static void draw_top_vignette(cv::Mat& img) {
    static constexpr float kMaxAlpha = 0.60f;
    const int fade_h = img.rows * 45 / 100;

    // Work in float [0,1] so the scale doesn't clip
    cv::Mat img_f;
    img.convertTo(img_f, CV_32FC3, 1.0 / 255.0);

    for (int y = 0; y < fade_h; ++y) {
        const float scale = 1.f - kMaxAlpha * (1.f - static_cast<float>(y) / fade_h);
        img_f.row(y) *= scale;
    }

    img_f.convertTo(img, CV_8UC3, 255.0);
}

// ─── Draw: max speed limit box (top-left, Comma AI style) ────────────────────
static void draw_speed_limit(cv::Mat& img, double speed_limit_ms) {
    if (speed_limit_ms <= 0.0) return;
    const int mph = static_cast<int>(std::round(speed_limit_ms * 2.23694));

    char lbl[16];
    std::snprintf(lbl, sizeof(lbl), "%d", mph);

    const int box_x = 14, box_y = 14;
    const int box_w = 68, box_h = 76;

    // White rounded rectangle outline
    cv::rectangle(img, cv::Point(box_x, box_y),
                  cv::Point(box_x + box_w, box_y + box_h),
                  cv::Scalar(255, 255, 255), 3, cv::LINE_AA);

    // "MAX" label
    draw_hud_text_centered(img, "MAX", box_x + box_w / 2, box_y + 20, 0.38,
                           cv::Scalar(255, 255, 255));

    // Speed number
    draw_hud_text_centered(img, lbl, box_x + box_w / 2, box_y + 58, 0.90,
                           cv::Scalar(255, 255, 255), 2);
}

// ─── Draw: AutoDrive-only in-path CIPO arrow (AD detects, AutoSpeed misses) ──
// Projects AD distance onto the fused path centerline; Comma-style filled triangle.
static void draw_ad_only_cipo(cv::Mat& img, const ProductionView& view) {
    if (!view.ad_cipo_only) return;
    if (view.ad_distance_m <= 0.f || view.ad_distance_m >= kDMax) return;

    const cv::Mat& H_w2px = (!view.H_world2px.empty()) ? view.H_world2px : g_H_world2px;
    if (H_w2px.empty()) return;

    // Place on path center at AD distance (not fixed lateral y=0).
    const float xw = view.ad_distance_m;
    float yw = 0.f;
    if (view.path_valid)
        yw = view.path_a * xw * xw + view.path_b * xw + view.path_c;

    std::vector<cv::Point2f> src = {cv::Point2f(xw, yw)}, dst;
    cv::perspectiveTransform(src, dst, H_w2px);

    const int px = static_cast<int>(std::lround(dst[0].x));
    const int py = static_cast<int>(std::lround(dst[0].y));

    if (px < 0 || px >= img.cols || py < 0 || py >= img.rows) return;

    // Comma-style: filled orange triangle, tip toward horizon (30% smaller than original).
    static constexpr int kAw = 20;
    static constexpr int kAh = 25;
    static const cv::Scalar kOrange(80, 180, 255);  // BGR — soft orange

    const std::vector<cv::Point> tri = {
        cv::Point(px,      py - kAh),   // tip — ahead on path
        cv::Point(px - kAw, py),         // base-left
        cv::Point(px + kAw, py),         // base-right
    };
    cv::fillConvexPoly(img, tri, kOrange, cv::LINE_AA);
    cv::polylines(img, tri, true, kOrange, 1, cv::LINE_AA);

    char lbl[16];
    std::snprintf(lbl, sizeof(lbl), "%.0fm", static_cast<double>(view.ad_distance_m));
    int bl = 0;
    cv::Size ts = cv::getTextSize(lbl, cv::FONT_HERSHEY_SIMPLEX, 0.52, 2, &bl);
    const int tx = px - ts.width / 2;
    const int ty = py - kAh - 8;
    if (ty > ts.height) {
        cv::rectangle(img,
            cv::Point(tx - 4, ty - ts.height - 2),
            cv::Point(tx + ts.width + 4, ty + 4),
            cv::Scalar(20, 20, 20), -1);
        cv::putText(img, lbl, cv::Point(tx, ty),
                    cv::FONT_HERSHEY_SIMPLEX, 0.52, kOrange, 2, cv::LINE_AA);
    }
}

// ─── Internal: draw production UI onto frame and show window ───────────────────
static cv::Mat draw_production_frame(cv::Mat& frame, const ProductionView& view) {
    if (frame.empty()) return cv::Mat();

    if (!g_h_ready || !g_ico_ready)
        init_production_assets(view.icons_dir);

    cv::Mat display_frame = frame.clone();

    // Render order: vignette → path → boxes → AD-only arrow → alerts → speed → speed limit
    draw_top_vignette(display_frame);
    draw_path_corridor(display_frame, view);
    draw_cipo_boxes(display_frame, view);
    draw_ad_only_cipo(display_frame, view);
    draw_alerts(display_frame, view);
    draw_speed(display_frame, view.ego_speed_ms);
    draw_speed_limit(display_frame, view.speed_limit_ms);

    return display_frame;
}

// ─── ProductionView ──────────────────────────────────────────────────────────

ProductionView ProductionView::from(
    const visionpilot::models::InferenceFrameResult& r,
    const Plan& plan,
    double ego_speed_ms,
    const cv::Mat& H_resized,
    double speed_limit_ms)
{
    ProductionView pv;
    pv.ego_speed_ms   = ego_speed_ms;
    pv.speed_limit_ms = speed_limit_ms;
    pv.acceleration   = plan.acceleration;

    // H_resized maps resized-px → world.  Invert to get world → display-px.
    if (!H_resized.empty()) {
        cv::Mat H64;
        H_resized.convertTo(H64, CV_64F);
        cv::Mat H64_inv = H64.inv();          // MatExpr → cv::Mat
        H64_inv.convertTo(pv.H_world2px, CV_32F);
    }

    pv.warnings.reserve(plan.warnings.size());
    for (const auto w : plan.warnings)
        pv.warnings.push_back(static_cast<uint8_t>(w));

    if (r.lateral.path_valid) {
        pv.path_a       = r.lateral.path_a;
        pv.path_b       = r.lateral.path_b;
        pv.path_c       = r.lateral.path_c;
        pv.path_x_min_m = r.lateral.path_x_min_m;
        pv.path_x_max_m = r.lateral.path_x_max_m;
        pv.path_valid   = true;
    }

    pv.detections.reserve(r.auto_speed.detections.size());
    for (const auto& d : r.auto_speed.detections) {
        pv.detections.push_back({d.x1, d.y1, d.x2, d.y2, d.score, d.class_id});
    }

    pv.cipo = {
        r.cipo.valid,
        r.cipo.distance_m,
        r.cipo.velocity_ms,
        r.cipo.cipo_raw_found,
        r.cipo.cipo_raw_dist_m,
        r.cipo.cut_in_detected,
    };

    // AutoDrive-only CIPO: AD confirms an in-path object (flag_prob ≥ 40%)
    // but AutoSpeed found no bbox (cipo_raw_found == false).
    // Show a red arrow projected at the AD-estimated distance.
    static constexpr float D_MAX_M = 150.f;
    if (r.auto_drive.valid &&
        r.auto_drive.flag_prob >= 0.40f &&
        !r.cipo.cipo_raw_found) {
        pv.ad_cipo_only  = true;
        pv.ad_distance_m = D_MAX_M * (1.f - r.auto_drive.dist_normalized);
    }

    return pv;
}

cv::Mat ProductionView::render(cv::Mat& frame) const {
    return draw_production_frame(frame, *this);
}

cv::Mat ProductionView::visualize(
    cv::Mat& frame,
    const visionpilot::models::InferenceFrameResult& result,
    const Plan& plan,
    double ego_speed_ms,
    const cv::Mat& H_resized,
    double speed_limit_ms)
{
    return from(result, plan, ego_speed_ms, H_resized, speed_limit_ms).render(frame);
}

// ─── Plain frame display (warm-up / no inference yet) ────────────────────────
bool show_frame(const cv::Mat& frame, const std::string& window_name) {
    if (frame.empty()) return false;

    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name, frame.cols, frame.rows);
    cv::imshow(window_name, frame);
    cv::waitKey(1);
    return true;
}

void close_windows() {
    cv::destroyAllWindows();
}

Visualization::Visualization(Config cfg)
{
    if (cfg.webrtc_on)
    {
        visual_interface = std::make_unique<WebRTCStreamer>();
        static_cast<WebRTCStreamer*>(visual_interface.get())->init(cfg.webrtc_port);
    } else
    {
        visual_interface = std::make_unique<LocalDisplay>();
    }
}


cv::Mat Visualization::build_frame(cv::Mat& frame,
        const visionpilot::models::InferenceFrameResult& result,
        const Plan& plan,
        double ego_speed_ms,
        const cv::Mat& H_resized,
        double speed_limit_ms)
{
    return ProductionView::from(result, plan, ego_speed_ms, H_resized, speed_limit_ms).render(frame);
}

bool Visualization::render_frame(const cv::Mat& display_frame)
{
    return visual_interface -> render_frame(display_frame);
}

}  // namespace visualization
