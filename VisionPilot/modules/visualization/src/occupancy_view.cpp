#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <visualization/occupancy_view.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <utility>
#include <vector>

namespace visualization
{
namespace occupancy
{

static void fill_rect_alpha(cv::Mat & img, cv::Rect r, cv::Scalar color, double alpha)
{
  r &= cv::Rect(0, 0, img.cols, img.rows);
  if (r.width <= 0 || r.height <= 0) return;
  cv::Mat roi = img(r);
  cv::Mat overlay(roi.size(), roi.type(), color);
  cv::addWeighted(overlay, alpha, roi, 1.0 - alpha, 0.0, roi);
}

// ─── Occupancy 3D camera (orbit / pan / zoom) ──────────────────────────
struct OccCam
{
  float yaw = 0.12f;    // rad around vertical
  float pitch = 0.92f;  // rad from horizontal (look-down)
  float dist = 52.f;    // metres from look-at
  float tx = 18.f;      // look-at forward (m)
  float ty = 0.f;       // look-at lateral (m)
  float tz = 0.f;
  int mode = 0;  // 0 idle, 1 orbit, 2 pan
  int lx = 0, ly = 0;
};

static OccCam g_occ_cam;

static void occ_cam_reset()
{
  g_occ_cam = OccCam{};
}

void on_key(int key)
{
  const int k = key & 0xff;
  if (k == 'r' || k == 'R')
    occ_cam_reset();
  else if (k == '+' || k == '=')
    g_occ_cam.dist = std::clamp(g_occ_cam.dist * 0.92f, 12.f, 160.f);
  else if (k == '-' || k == '_')
    g_occ_cam.dist = std::clamp(g_occ_cam.dist * 1.08f, 12.f, 160.f);
}

void on_mouse(int event, int x, int y, int flags, void *)
{
  auto & c = g_occ_cam;
  if (event == cv::EVENT_LBUTTONDOWN) {
    c.mode = 1;
    c.lx = x;
    c.ly = y;
  } else if (event == cv::EVENT_RBUTTONDOWN || event == cv::EVENT_MBUTTONDOWN) {
    c.mode = 2;
    c.lx = x;
    c.ly = y;
  } else if (
    event == cv::EVENT_LBUTTONUP || event == cv::EVENT_RBUTTONUP || event == cv::EVENT_MBUTTONUP) {
    c.mode = 0;
  } else if (event == cv::EVENT_MOUSEMOVE && c.mode != 0) {
    const float dx = static_cast<float>(x - c.lx);
    const float dy = static_cast<float>(y - c.ly);
    c.lx = x;
    c.ly = y;
    if (c.mode == 1) {
      c.yaw += dx * 0.0075f;
      c.pitch = std::clamp(c.pitch + dy * 0.0055f, 0.18f, 1.45f);
    } else {
      // Pan look-at in ground plane along camera right / forward-flat
      const float cy = std::cos(c.yaw), sy = std::sin(c.yaw);
      const float scale = c.dist * 0.0018f;
      c.tx += (-sy * dx + cy * dy) * scale;
      c.ty += (cy * dx + sy * dy) * scale;
      c.tx = std::clamp(c.tx, -10.f, 90.f);
      c.ty = std::clamp(c.ty, -25.f, 25.f);
    }
  } else if (event == cv::EVENT_MOUSEWHEEL || event == cv::EVENT_MOUSEHWHEEL) {
    const int delta = cv::getMouseWheelDelta(flags);
    const float factor = (delta > 0) ? 0.90f : 1.11f;
    c.dist = std::clamp(c.dist * factor, 12.f, 160.f);
  } else if (event == cv::EVENT_LBUTTONDBLCLK) {
    occ_cam_reset();
  }
}

struct OccBasis
{
  float eye_x, eye_y, eye_z;
  float r0, r1, r2;  // right
  float u0, u1, u2;  // up
  float f0, f1, f2;  // forward
  float foc;         // focal length in px
  int pw, ph;
};

static OccBasis make_occ_basis(int pw, int ph)
{
  const auto & c = g_occ_cam;
  const float cp = std::cos(c.pitch), sp = std::sin(c.pitch);
  const float cy = std::cos(c.yaw), sy = std::sin(c.yaw);

  OccBasis b;
  b.pw = pw;
  b.ph = ph;
  // Behind look-at along -x when yaw=0, elevated by pitch
  b.eye_x = c.tx - c.dist * cp * cy;
  b.eye_y = c.ty - c.dist * cp * sy;
  b.eye_z = c.tz + c.dist * sp;

  const float fx = c.tx - b.eye_x;
  const float fy = c.ty - b.eye_y;
  const float fz = c.tz - b.eye_z;
  const float fl = std::sqrt(fx * fx + fy * fy + fz * fz) + 1e-6f;
  b.f0 = fx / fl;
  b.f1 = fy / fl;
  b.f2 = fz / fl;

  // right = forward × world_up
  float rx = b.f1 * 1.f - b.f2 * 0.f;
  float ry = b.f2 * 0.f - b.f0 * 1.f;
  float rz = b.f0 * 0.f - b.f1 * 0.f;
  // world_up=(0,0,1): right = f × up = (f1, -f0, 0)
  rx = b.f1;
  ry = -b.f0;
  rz = 0.f;
  float rl = std::sqrt(rx * rx + ry * ry + rz * rz) + 1e-6f;
  b.r0 = rx / rl;
  b.r1 = ry / rl;
  b.r2 = rz / rl;

  // up = right × forward
  b.u0 = b.r1 * b.f2 - b.r2 * b.f1;
  b.u1 = b.r2 * b.f0 - b.r0 * b.f2;
  b.u2 = b.r0 * b.f1 - b.r1 * b.f0;

  constexpr float kFovDeg = 48.f;
  const float fov = kFovDeg * 0.01745329252f;
  b.foc = 0.5f * static_cast<float>(ph) / std::tan(0.5f * fov);
  return b;
}

static bool project_xyz(
  const OccBasis & b, float x, float y, float z, cv::Point & out, float * depth = nullptr)
{
  const float dx = x - b.eye_x, dy = y - b.eye_y, dz = z - b.eye_z;
  const float cam_z = dx * b.f0 + dy * b.f1 + dz * b.f2;
  if (cam_z < 0.8f) return false;
  const float cam_x = dx * b.r0 + dy * b.r1 + dz * b.r2;
  const float cam_y = dx * b.u0 + dy * b.u1 + dz * b.u2;
  out.x = static_cast<int>(std::lround(0.5f * b.pw + b.foc * cam_x / cam_z));
  out.y = static_cast<int>(std::lround(0.5f * b.ph - b.foc * cam_y / cam_z));
  if (depth) *depth = cam_z;
  return true;
}

// ─── Occupancy vehicle primitives: clean extruded boxes (car vs truck) ────────

static bool detection_looks_like_truck(float width_m, float bbox_w, float bbox_h)
{
  if (width_m >= 2.25f) return true;
  if (width_m >= 1.95f && bbox_h > bbox_w * 0.85f) return true;
  if (bbox_h > 140.f && bbox_w > 90.f && width_m >= 1.7f) return true;
  return false;
}

static cv::Scalar vehicle_paint(int class_id, bool is_truck)
{
  // Ego (class_id 0) stays pearl white; other traffic is #0000FF.
  if (class_id == 0) return cv::Scalar(215, 212, 208);
  (void)is_truck;
  return cv::Scalar(220, 200, 0);  // BGR for #0000FF
}

static float wall_shade_pts(cv::Point a, cv::Point b)
{
  const float ex = static_cast<float>(b.x - a.x);
  const float ey = static_cast<float>(b.y - a.y);
  float nx = ey, ny = -ex;
  const float len = std::sqrt(nx * nx + ny * ny);
  if (len < 1e-3f) return 0.55f;
  nx /= len;
  ny /= len;
  constexpr float lx = 0.40f, ly = -0.85f;
  return std::clamp(0.38f + 0.55f * (nx * lx + ny * ly), 0.22f, 0.92f);
}

static void extrude_box_3d(
  cv::Mat & layer, const OccBasis & basis, float x0, float y0, float x1, float y1, float z0,
  float z1, cv::Scalar paint, float fade, float top_boost = 1.10f)
{
  if (z1 <= z0 + 0.05f) return;
  auto scale_bgr = [](cv::Scalar c, float s) -> cv::Scalar {
    return cv::Scalar(
      std::min(255.0, c[0] * s), std::min(255.0, c[1] * s), std::min(255.0, c[2] * s));
  };

  // Corner order: 0=(x0,y1), 1=(x0,y0), 2=(x1,y0), 3=(x1,y1)
  const float xs[4] = {x0, x0, x1, x1};
  const float ys[4] = {y1, y0, y0, y1};
  cv::Point foot[4], top[4];
  float fd[4], td[4];
  for (int i = 0; i < 4; ++i) {
    if (!project_xyz(basis, xs[i], ys[i], z0, foot[i], &fd[i])) return;
    if (!project_xyz(basis, xs[i], ys[i], z1, top[i], &td[i])) return;
  }

  struct Face
  {
    float depth;
    int kind;
    int i;
  };  // kind 0=wall, 1=top
  std::array<Face, 5> faces;
  for (int i = 0; i < 4; ++i) {
    const int j = (i + 1) % 4;
    faces[static_cast<size_t>(i)] = {0.25f * (fd[i] + fd[j] + td[i] + td[j]), 0, i};
  }
  faces[4] = {0.25f * (td[0] + td[1] + td[2] + td[3]), 1, 0};
  std::sort(
    faces.begin(), faces.end(), [](const Face & a, const Face & b) { return a.depth > b.depth; });

  for (const auto & f : faces) {
    if (f.kind == 1) {
      const std::vector<cv::Point> poly = {top[0], top[1], top[2], top[3]};
      cv::fillConvexPoly(layer, poly, scale_bgr(paint, top_boost * fade), cv::LINE_AA);
      cv::polylines(
        layer, poly, true, scale_bgr(cv::Scalar(255, 255, 255), 0.35f * fade), 1, cv::LINE_AA);
      cv::line(
        layer, top[0], top[1], scale_bgr(cv::Scalar(255, 255, 255), 0.55f * fade), 1, cv::LINE_AA);
    } else {
      const int i = f.i;
      const int j = (i + 1) % 4;
      const float sh = wall_shade_pts(foot[i], foot[j]);
      const std::vector<cv::Point> wall = {foot[i], foot[j], top[j], top[i]};
      cv::fillConvexPoly(layer, wall, scale_bgr(paint, sh * fade), cv::LINE_AA);
    }
  }
}

// Axis-aligned extruded box corners rotated by yaw about vehicle center.
static void extrude_box_yaw(
  cv::Mat & layer, const OccBasis & basis, float x_fwd, float y_lat, float half_l, float half_w,
  float z0, float z1, float yaw, const cv::Scalar & paint, float fade, float top_boost = 1.12f)
{
  const float cy = std::cos(yaw), sy = std::sin(yaw);
  // Local corners (forward, left) then rotate into world (+x fwd, +y left).
  const float local[4][2] = {
    {-half_l, -half_w},
    {+half_l, -half_w},
    {+half_l, +half_w},
    {-half_l, +half_w},
  };
  float xs[4], ys[4];
  for (int i = 0; i < 4; ++i) {
    const float lx = local[i][0], ly = local[i][1];
    xs[i] = x_fwd + cy * lx - sy * ly;
    ys[i] = y_lat + sy * lx + cy * ly;
  }

  auto scale_bgr = [](cv::Scalar c, float s) -> cv::Scalar {
    return cv::Scalar(
      std::min(255.0, c[0] * s), std::min(255.0, c[1] * s), std::min(255.0, c[2] * s));
  };
  auto wall_shade_pts = [](const cv::Point & a, const cv::Point & b) -> float {
    const float dx = static_cast<float>(b.x - a.x);
    const float dy = static_cast<float>(b.y - a.y);
    const float len = std::sqrt(dx * dx + dy * dy) + 1e-3f;
    const float nx = -dy / len;
    return std::clamp(0.55f + 0.45f * nx, 0.35f, 1.0f);
  };

  cv::Point foot[4], top[4];
  float fd[4], td[4];
  for (int i = 0; i < 4; ++i) {
    if (!project_xyz(basis, xs[i], ys[i], z0, foot[i], &fd[i])) return;
    if (!project_xyz(basis, xs[i], ys[i], z1, top[i], &td[i])) return;
  }
  struct Face
  {
    float d;
    int i;
  };
  Face faces[4];
  for (int i = 0; i < 4; ++i) {
    faces[i] = {0.5f * (fd[i] + fd[(i + 1) % 4]), i};
  }
  std::sort(faces, faces + 4, [](const Face & a, const Face & b) { return a.d > b.d; });

  const std::vector<cv::Point> roof = {top[0], top[1], top[2], top[3]};
  cv::fillConvexPoly(layer, roof, scale_bgr(paint, top_boost * fade), cv::LINE_AA);
  for (const auto & f : faces) {
    const int i = f.i;
    const int j = (i + 1) % 4;
    const float sh = wall_shade_pts(foot[i], foot[j]);
    const std::vector<cv::Point> wall = {foot[i], foot[j], top[j], top[i]};
    cv::fillConvexPoly(layer, wall, scale_bgr(paint, sh * fade), cv::LINE_AA);
  }
}

static void draw_extruded_vehicle(
  cv::Mat & layer, const OccBasis & basis, float x_fwd, float y_lat, float length_m, float width_m,
  bool is_truck, int class_id, float fade, bool is_cipo = false, float yaw = 0.f)
{
  auto scale_bgr = [](cv::Scalar c, float s) -> cv::Scalar {
    return cv::Scalar(
      std::min(255.0, c[0] * s), std::min(255.0, c[1] * s), std::min(255.0, c[2] * s));
  };

  // Nearest in-lane vehicle (CIPO) → solid red so it stands out.
  const cv::Scalar paint = is_cipo ? cv::Scalar(40, 40, 220) : vehicle_paint(class_id, is_truck);
  const float half_l = 0.5f * length_m;
  const float half_w = 0.5f * width_m;

  // Ground shadow
  {
    cv::Point sc;
    if (project_xyz(basis, x_fwd, y_lat, 0.f, sc)) {
      const float d = std::max(0.4f, fade);
      cv::ellipse(
        layer, sc + cv::Point(1, 2),
        cv::Size(
          std::max(4, static_cast<int>(width_m * 5.0f * d)),
          std::max(2, static_cast<int>(length_m * 2.2f * d))),
        yaw * 180.f / static_cast<float>(CV_PI), 0, 360, cv::Scalar(14, 12, 10), -1, cv::LINE_AA);
    }
  }

  if (std::abs(yaw) > 1e-3f) {
    // Yaw-aware draw (used for ego during lane change / heading error).
    if (!is_truck) {
      extrude_box_yaw(
        layer, basis, x_fwd, y_lat, half_l, half_w, 0.f, 1.45f, yaw, paint, fade,
        is_cipo ? 1.05f : 1.12f);
    } else {
      extrude_box_yaw(
        layer, basis, x_fwd, y_lat, half_l, half_w, 0.f, 3.0f, yaw, paint, fade, 1.05f);
    }
    return;
  }

  if (!is_truck) {
    extrude_box_3d(
      layer, basis, x_fwd - half_l, y_lat - half_w, x_fwd + half_l, y_lat + half_w, 0.f, 1.45f,
      paint, fade, is_cipo ? 1.05f : 1.12f);
  } else {
    const float split = x_fwd + half_l * 0.35f;
    extrude_box_3d(
      layer, basis, x_fwd - half_l, y_lat - half_w, split, y_lat + half_w, 0.f, 3.2f,
      scale_bgr(paint, is_cipo ? 1.0f : 0.90f), fade, 1.05f);
    extrude_box_3d(
      layer, basis, split - half_l * 0.02f, y_lat - half_w * 0.88f, x_fwd + half_l,
      y_lat + half_w * 0.88f, 0.f, 2.5f, scale_bgr(paint, is_cipo ? 1.05f : 1.05f), fade, 1.12f);
  }
}

static cv::Scalar radar_return_paint(const Scene::RadarReturn & p)
{
  if (p.in_match) return cv::Scalar(0, 255, 100);
  return p.moving ? cv::Scalar(0, 90, 160) : cv::Scalar(88, 88, 88);
}

// ─── Occupancy BEV panel (extruded boxes) — separate interactive 3D window ────
cv::Mat render(const Scene & scene)
{
  constexpr int pw = 560;
  constexpr int ph = 700;
  constexpr float kXMax = 150.f;
  constexpr float kYMax = 12.f;

  // Path-centered frame: keep the green corridor near y=0 and slide the ego
  // car by −CTE so a lane change reads as the white car moving laterally.
  // +cte = ego right of path; +y = left ⇒ ego_y = −cte.
  static float g_cte_s = 0.f;
  static float g_yaw_s = 0.f;
  const float cte_tgt = std::clamp(scene.cte_m, -4.5f, 4.5f);
  const float yaw_tgt = std::clamp(scene.yaw_rad, -0.55f, 0.55f);
  constexpr float kSmooth = 0.20f;  // ~5-frame settle at 11 fps
  g_cte_s += kSmooth * (cte_tgt - g_cte_s);
  g_yaw_s += kSmooth * (yaw_tgt - g_yaw_s);
  const float y_shift = g_cte_s;   // subtract from world-y (path → ~0)
  const float ego_y = -g_cte_s;    // ego slides opposite the path offset
  const float ego_yaw = -g_yaw_s;  // body yaw vs path heading

  const OccBasis basis = make_occ_basis(pw, ph);

  auto world_to_panel = [&](float x_fwd, float y_lat) -> cv::Point {
    cv::Point p(pw / 2, ph / 2);
    project_xyz(basis, x_fwd, y_lat, 0.f, p);
    return p;
  };

  auto depth_fade = [&](float x_fwd, float y_lat) -> float {
    float d = 40.f;
    cv::Point unused;
    project_xyz(basis, x_fwd, y_lat, 0.f, unused, &d);
    // Fade past the camera's own standoff distance, not from zero.
    const float rel = std::max(0.f, d - g_occ_cam.dist * 0.75f);
    return std::clamp(1.f - 0.006f * rel, 0.55f, 1.f);
  };

  auto scale_bgr = [](cv::Scalar c, float s) -> cv::Scalar {
    return cv::Scalar(
      std::min(255.0, c[0] * s), std::min(255.0, c[1] * s), std::min(255.0, c[2] * s));
  };

  const cv::Scalar kSkyFar(34, 27, 22);
  const cv::Scalar kGroundNear(52, 43, 36);
  const cv::Scalar kGridMaj(92, 83, 72);
  const cv::Scalar kGridMin(72, 63, 54);
  const cv::Scalar kPathFill(120, 190, 55);
  const cv::Scalar kPathEdge(180, 240, 110);
  const cv::Scalar kLane(90, 88, 86);

  cv::Mat panel(ph, pw, CV_8UC3);
  for (int y = 0; y < ph; ++y) {
    const float ty = static_cast<float>(y) / std::max(1, ph - 1);
    const float b = kSkyFar[0] + (kGroundNear[0] - kSkyFar[0]) * ty;
    const float g = kSkyFar[1] + (kGroundNear[1] - kSkyFar[1]) * ty;
    const float r = kSkyFar[2] + (kGroundNear[2] - kSkyFar[2]) * ty;
    unsigned char * row = panel.ptr<unsigned char>(y);
    for (int x = 0; x < pw; ++x) {
      const float nx = (x - pw * 0.5f) / (pw * 0.5f);
      const float edge = 1.f - 0.18f * nx * nx;
      row[x * 3 + 0] = static_cast<unsigned char>(b * edge);
      row[x * 3 + 1] = static_cast<unsigned char>(g * edge);
      row[x * 3 + 2] = static_cast<unsigned char>(r * edge);
    }
  }

  // Ground plane
  {
    std::vector<cv::Point> ground;
    ground.reserve(4);
    for (auto [x, y] :
         {std::pair<float, float>{0.f, kYMax}, {0.f, -kYMax}, {kXMax, -kYMax}, {kXMax, kYMax}}) {
      cv::Point p;
      if (project_xyz(basis, x, y, 0.f, p)) ground.push_back(p);
    }
    if (ground.size() == 4)
      cv::fillConvexPoly(panel, ground, cv::Scalar(58, 48, 40), cv::LINE_AA);
  }

  // Grid
  for (float y = -kYMax; y <= kYMax + 0.01f; y += 1.f) {
    const bool major = (static_cast<int>(std::lround(std::abs(y))) % 2 == 0);
    cv::Point a, b;
    if (!project_xyz(basis, 0.f, y, 0.f, a)) continue;
    if (!project_xyz(basis, kXMax, y, 0.f, b)) continue;
    cv::line(panel, a, b, major ? kGridMaj : kGridMin, 1, cv::LINE_AA);
  }
  for (float x = 0.f; x <= kXMax + 0.01f; x += 5.f) {
    const float fade = depth_fade(x, 0.f);
    cv::Point a, b;
    if (!project_xyz(basis, x, -kYMax, 0.f, a)) continue;
    if (!project_xyz(basis, x, kYMax, 0.f, b)) continue;
    cv::line(panel, a, b, scale_bgr(kGridMaj, fade), 1, cv::LINE_AA);
    if (x > 0.f && static_cast<int>(x) % 20 == 0) {
      char lbl[8];
      std::snprintf(lbl, sizeof(lbl), "%.0f", static_cast<double>(x));
      cv::Point p;
      if (project_xyz(basis, x, -kYMax + 0.6f, 0.f, p))
        cv::putText(
          panel, lbl, p + cv::Point(3, -3), cv::FONT_HERSHEY_SIMPLEX, 0.33,
          scale_bgr(cv::Scalar(140, 138, 134), fade), 1, cv::LINE_AA);
    }
  }
  for (float x : {20.f, 40.f, 60.f, 80.f, 100.f, 120.f, 150.f}) {
    const float fade = depth_fade(x, 0.f) * 0.7f;
    cv::Point a, b;
    if (!project_xyz(basis, x, -kYMax, 0.f, a)) continue;
    if (!project_xyz(basis, x, kYMax, 0.f, b)) continue;
    cv::line(panel, a, b, scale_bgr(cv::Scalar(82, 74, 64), fade), 1, cv::LINE_AA);
  }

  // Green path corridor — always extend to kXMax when path is valid
  if (scene.path_valid) {
    const float a = scene.path_a, b = scene.path_b, c = scene.path_c;
    const float x0 = 0.5f;
    const float x1 = kXMax;  // force full-range ribbon
    std::vector<cv::Point> lp, rp, mid;
    for (float x = x0; x <= x1 + 0.01f; x += 1.0f) {
      const float yc = a * x * x + b * x + c - y_shift;
      cv::Point pl, pr, pm;
      if (!project_xyz(basis, x, yc + 1.15f, 0.02f, pl)) continue;
      if (!project_xyz(basis, x, yc - 1.15f, 0.02f, pr)) continue;
      if (!project_xyz(basis, x, yc, 0.02f, pm)) continue;
      lp.push_back(pl);
      rp.push_back(pr);
      mid.push_back(pm);
    }
    if (lp.size() >= 2) {
      cv::Mat fill = panel.clone();
      for (size_t i = 0; i + 1 < lp.size(); ++i) {
        const std::vector<cv::Point> quad = {lp[i], rp[i], rp[i + 1], lp[i + 1]};
        cv::fillConvexPoly(fill, quad, kPathFill, cv::LINE_AA);
      }
      cv::addWeighted(fill, 0.22, panel, 0.78, 0.0, panel);
      cv::polylines(panel, lp, false, kPathEdge, 1, cv::LINE_AA);
      cv::polylines(panel, rp, false, kPathEdge, 1, cv::LINE_AA);
      if (mid.size() >= 2)
        cv::polylines(panel, mid, false, cv::Scalar(210, 255, 160), 1, cv::LINE_AA);
    }
  }

  if (scene.lane_world.size() >= 2) {
    std::vector<cv::Point> lane_px;
    lane_px.reserve(scene.lane_world.size());
    for (const auto & w : scene.lane_world) {
      if (w.x < 0.f || w.x > kXMax) continue;
      cv::Point p;
      if (project_xyz(basis, w.x, w.y - y_shift, 0.03f, p)) lane_px.push_back(p);
    }
    if (lane_px.size() >= 2) cv::polylines(panel, lane_px, false, kLane, 2, cv::LINE_AA);
  }

  auto px_to_world = [&](float u, float v, float & xw, float & yw) -> bool {
    if (scene.H_px2world.empty()) return false;
    std::vector<cv::Point2f> src = {cv::Point2f(u, v)}, dst;
    cv::perspectiveTransform(src, dst, scene.H_px2world);
    xw = dst[0].x;
    yw = dst[0].y;
    if (!std::isfinite(xw) || !std::isfinite(yw)) return false;
    if (xw < -2.f || xw > kXMax + 8.f) return false;
    return true;
  };

  struct Voxel
  {
    float x, y, half_w, half_l;
    int class_id;
    bool is_truck;
    bool is_cipo;
    float cam_depth;
  };
  std::vector<Voxel> voxels;
  voxels.reserve(scene.detections.size());
  for (const auto & d : scene.detections) {
    float x_bl = 0, y_bl = 0, x_br = 0, y_br = 0, x_bc = 0, y_bc = 0;
    const float cx_box = 0.5f * (d.x1 + d.x2);
    if (!px_to_world(d.x1, d.y2, x_bl, y_bl)) continue;
    if (!px_to_world(d.x2, d.y2, x_br, y_br)) continue;
    if (!px_to_world(cx_box, d.y2, x_bc, y_bc)) continue;
    const float width_m = std::clamp(std::abs(y_bl - y_br), 0.9f, 3.2f);
    const float bbox_w = std::max(1.f, d.x2 - d.x1);
    const float bbox_h = std::max(1.f, d.y2 - d.y1);
    const bool truck = detection_looks_like_truck(width_m, bbox_w, bbox_h);
    Voxel v;
    v.x = x_bc;
    v.y = y_bc - y_shift;
    v.half_l = truck ? 5.0f : 2.25f;
    v.half_w = truck ? 1.20f : 0.90f;
    v.class_id = d.class_id;
    v.is_truck = truck;
    v.is_cipo = false;
    v.cam_depth = 0.f;
    cv::Point unused;
    project_xyz(basis, v.x, v.y, 0.f, unused, &v.cam_depth);
    voxels.push_back(v);
  }

  // Paint nearest in-lane vehicle (CIPO) red; keep its world pose for the label.
  const float focus_x =
    (scene.cipo_valid && scene.cipo_distance_m > 0.5f)
      ? scene.cipo_distance_m
      : ((scene.ad_cipo_only && scene.ad_distance_m > 0.5f) ? scene.ad_distance_m : -1.f);
  if (focus_x > 0.f && !voxels.empty()) {
    auto path_y = [&](float x) -> float {
      if (!scene.path_valid) return 0.f;
      return scene.path_a * x * x + scene.path_b * x + scene.path_c - y_shift;
    };
    size_t best_i = 0;
    float best_score = 1e9f;
    bool found = false;
    for (size_t i = 0; i < voxels.size(); ++i) {
      const float dy = std::abs(voxels[i].y - path_y(voxels[i].x));
      if (dy > 2.0f) continue;
      const float dx = std::abs(voxels[i].x - focus_x);
      const float score = dx + (voxels[i].class_id == 1 ? 0.f : 2.f);
      if (score < best_score) {
        best_score = score;
        best_i = i;
        found = true;
      }
    }
    if (!found) {
      for (size_t i = 0; i < voxels.size(); ++i) {
        const float dx = std::abs(voxels[i].x - focus_x);
        if (dx < best_score) {
          best_score = dx;
          best_i = i;
          found = true;
        }
      }
    }
    if (found && best_score < 25.f) voxels[best_i].is_cipo = true;
  }

  std::sort(voxels.begin(), voxels.end(), [](const Voxel & a, const Voxel & b) {
    return a.cam_depth > b.cam_depth;
  });

  // After sort, find the red CIPO car again for label anchoring.
  const Voxel * cipo_car = nullptr;
  for (const auto & vox : voxels) {
    if (vox.is_cipo) {
      cipo_car = &vox;
      break;
    }
  }

  for (const auto & vox : voxels) {
    draw_extruded_vehicle(
      panel, basis, vox.x, vox.y, vox.half_l * 2.f, vox.half_w * 2.f, vox.is_truck, vox.class_id,
      depth_fade(vox.x, vox.y), vox.is_cipo);
  }

  // Distance label sits on the red car (no separate floating dot).
  // Use the car's forward position so the number matches the box.
  if (cipo_car != nullptr) {
    const float dist_m = cipo_car->x;
    cv::Point p;
    const float z_lbl = cipo_car->is_truck ? 3.4f : 1.7f;
    if (project_xyz(basis, cipo_car->x, cipo_car->y, z_lbl, p)) {
      char lbl[16];
      std::snprintf(lbl, sizeof(lbl), "%.0fm", static_cast<double>(dist_m));
      int bl = 0;
      const cv::Size ts = cv::getTextSize(lbl, cv::FONT_HERSHEY_SIMPLEX, 0.48, 1, &bl);
      const cv::Point origin(p.x - ts.width / 2, p.y - 8);
      cv::rectangle(
        panel, origin + cv::Point(-4, -ts.height - 2), origin + cv::Point(ts.width + 4, 4),
        cv::Scalar(20, 20, 20), -1);
      cv::putText(
        panel, lbl, origin, cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(80, 80, 255), 1,
        cv::LINE_AA);
    }
  } else if (scene.ad_cipo_only && scene.ad_distance_m > 0.5f && scene.ad_distance_m < kXMax) {
    // No AutoSpeed box — keep a small in-path marker only for AD-only CIPO.
    float yw = 0.f;
    if (scene.path_valid) {
      const float x = scene.ad_distance_m;
      yw = scene.path_a * x * x + scene.path_b * x + scene.path_c - y_shift;
    }
    cv::Point p;
    if (project_xyz(basis, scene.ad_distance_m, yw, 0.4f, p))
      cv::arrowedLine(
        panel, p + cv::Point(0, 16), p, cv::Scalar(40, 40, 220), 2, cv::LINE_AA, 0, 0.35);
  }

  // Ego: slides laterally with CTE (lane change / lane departure) and yaws with heading error.
  draw_extruded_vehicle(panel, basis, 1.5f, ego_y, 4.5f, 1.8f, false, 0, 1.0f, false, ego_yaw);

  // Radar returns only. Vehicle volumes remain camera-derived so roadside
  // structure cannot appear as a vehicle box.
  if (scene.radar_enabled) {
    struct Stick
    {
      float depth;
      cv::Point g;
      cv::Point t;
      cv::Scalar c;
      int r;
    };
    std::vector<Stick> sticks;
    sticks.reserve(scene.radar_points.size());
    for (const auto & p : scene.radar_points) {
      if (p.x < -1.f || p.x > kXMax) continue;
      if (std::abs(p.y - y_shift) > kYMax + 2.f) continue;
      const float y = p.y - y_shift;
      Stick s;
      if (!project_xyz(basis, p.x, y, 0.f, s.g, &s.depth)) continue;
      if (!project_xyz(basis, p.x, y, 0.55f, s.t)) continue;
      s.c = radar_return_paint(p);
      s.r = p.in_match ? 4 : 2;
      sticks.push_back(s);
    }
    std::sort(sticks.begin(), sticks.end(), [](const Stick & a, const Stick & b) {
      return a.depth > b.depth;
    });
    for (const auto & s : sticks) {
      cv::line(panel, s.g, s.t, s.c, 1, cv::LINE_AA);
      cv::circle(panel, s.t, s.r, s.c, -1, cv::LINE_AA);
    }
  }

  // Small lane-offset cue when meaningfully off the path center.
  if (std::abs(g_cte_s) > 0.35f) {
    char cte_lbl[32];
    std::snprintf(cte_lbl, sizeof(cte_lbl), "lane %+0.1fm", static_cast<double>(-g_cte_s));
    cv::putText(
      panel, cte_lbl, cv::Point(12, 48), cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(230, 230, 230),
      1, cv::LINE_AA);
  }

  cv::rectangle(
    panel, cv::Point(0, 0), cv::Point(pw - 1, ph - 1), cv::Scalar(70, 85, 50), 1, cv::LINE_AA);
  fill_rect_alpha(panel, cv::Rect(0, 0, pw, 26), cv::Scalar(10, 9, 8), 0.62);
  cv::line(panel, cv::Point(0, 26), cv::Point(pw, 26), cv::Scalar(90, 110, 55), 1, cv::LINE_AA);
  cv::putText(
    panel, scene.radar_enabled ? "OCCUPANCY | RADAR" : "OCCUPANCY", cv::Point(12, 18),
    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(210, 220, 180), 1, cv::LINE_AA);
  char range_lbl[32];
  std::snprintf(range_lbl, sizeof(range_lbl), "0-%.0fm", static_cast<double>(kXMax));
  int bl = 0;
  const cv::Size ts = cv::getTextSize(range_lbl, cv::FONT_HERSHEY_SIMPLEX, 0.38, 1, &bl);
  cv::putText(
    panel, range_lbl, cv::Point(pw - ts.width - 12, 18), cv::FONT_HERSHEY_SIMPLEX, 0.38,
    cv::Scalar(150, 160, 130), 1, cv::LINE_AA);

  // Interaction hint
  if (scene.radar_enabled) {
    char radar_lbl[128];
    std::snprintf(
      radar_lbl, sizeof(radar_lbl), "%zu radar pts   L-drag orbit  R-drag pan  wheel zoom  R reset",
      scene.radar_points.size());
    cv::putText(
      panel, radar_lbl, cv::Point(12, ph - 12), cv::FONT_HERSHEY_SIMPLEX, 0.36,
      cv::Scalar(140, 150, 120), 1, cv::LINE_AA);
  } else {
    cv::putText(
      panel, "L-drag orbit  R-drag pan  wheel zoom  R reset", cv::Point(12, ph - 12),
      cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(140, 150, 120), 1, cv::LINE_AA);
  }

  (void)world_to_panel;
  return panel;
}

}  // namespace occupancy
}  // namespace visualization
