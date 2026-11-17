#include <iostream>
#include <ostream>
#include <vector>
#include <cmath>
#include <string>
#include <planning/config.hpp>
#include <planning/longitudinal_planning.hpp>
#include <planning/mpc.hpp>
#include <opencv2/opencv.hpp>

// ---------------------------------------------------------------------------
// Canvas constants
// ---------------------------------------------------------------------------
static const int    W        = 1200;
static const int    H        = 700;
static const double SCALE    = 8.0;
static const int    ORIGIN_X = W / 2;
static const int    ORIGIN_Y = H / 2;   // vehicle always centred

// ---------------------------------------------------------------------------
// Camera position — follows vehicle every frame
// ---------------------------------------------------------------------------
static double cam_x = 0.0;
static double cam_y = 0.0;

// ---------------------------------------------------------------------------
// Colours (BGR)
// ---------------------------------------------------------------------------
static const cv::Scalar COL_BG         ( 20,  20,  20);
static const cv::Scalar COL_GRID       ( 45,  45,  45);
static const cv::Scalar COL_REF        (  0, 255, 255);   // yellow
static const cv::Scalar COL_PRED       (  0, 200,   0);   // green
static const cv::Scalar COL_CURRENT_ARC( 80,  80, 255);   // red
static const cv::Scalar COL_VEHICLE    (255, 255, 255);   // white
static const cv::Scalar COL_TEXT       (220, 220, 220);

using namespace planning;

// ---------------------------------------------------------------------------
// world2canvas — camera-relative so vehicle stays centred
// ---------------------------------------------------------------------------
cv::Point world2canvas(double x, double y) {
    return cv::Point(
        ORIGIN_X + static_cast<int>((x - cam_x) * SCALE),
        ORIGIN_Y - static_cast<int>((y - cam_y) * SCALE)
    );
}

// ---------------------------------------------------------------------------
// Reference path generators
// ---------------------------------------------------------------------------
void buildStraightPath(std::vector<double>& rx, std::vector<double>& ry,
                       Eigen::VectorXd& kappa_ref, int nPts, double ds) {
    rx.resize(nPts); ry.resize(nPts);
    kappa_ref = Eigen::VectorXd::Zero(N);
    for (int i = 0; i < nPts; i++) { rx[i] = 0.0; ry[i] = i * ds; }
}

void buildCirclePath(std::vector<double>& rx, std::vector<double>& ry,
                     Eigen::VectorXd& kappa_ref, int nPts, double ds) {
    const double R = 30.0;
    rx.resize(nPts); ry.resize(nPts);
    kappa_ref = Eigen::VectorXd::Constant(N, 1.0 / R);
    for (int i = 0; i < nPts; i++) {
        double angle = i * ds / R;
        rx[i] =  R * std::sin(angle);
        ry[i] = -R * std::cos(angle) + R;
    }
}

void buildSinePath(std::vector<double>& rx, std::vector<double>& ry,
                   Eigen::VectorXd& kappa_ref, int nPts, double ds) {
    const double A  = 6.0;
    const double wl = 40.0;
    const double w  = 2.0 * M_PI / wl;
    rx.resize(nPts); ry.resize(nPts);
    for (int i = 0; i < nPts; i++) {
        double s = i * ds;
        rx[i] = A * std::sin(w * s);
        ry[i] = s;
    }
    for (int i = 0; i < (int)N; i++) {
        double s   = i * ds;
        double xp  =  A * w     * std::cos(w * s);
        double xpp = -A * w * w * std::sin(w * s);
        kappa_ref[i] = xpp / std::pow(1.0 + xp * xp, 1.5);
    }
}

// ---------------------------------------------------------------------------
// Reconstruct predicted arc from MPC horizon [delta_i, a_i]
// ---------------------------------------------------------------------------
void reconstructArc(const std::vector<double>& mpc_result,
                    double px, double py, double path_yaw, double epsi, double v0,
                    std::vector<cv::Point>& arc_pts) {
    arc_pts.clear();
    arc_pts.push_back(world2canvas(px, py));
    double x = px, y = py, th = path_yaw + epsi, v = v0;
    for (int i = 0; i < (int)N - 1; i++) {
        double delta_i = mpc_result[2 + 2 * i];
        double a_i     = mpc_result[2 + 2 * i + 1];
        double kappa_i = std::tan(delta_i) / Lf;
        v += a_i * dt;
        double s = v * dt;
        x  += s * std::cos(th);
        y  += s * std::sin(th);
        th += kappa_i * s;
        arc_pts.push_back(world2canvas(x, y));
    }
}

// ---------------------------------------------------------------------------
// Reconstruct constant-kappa arc (steering held fixed)
// ---------------------------------------------------------------------------
void reconstructConstantArc(double delta_now, double px, double py,
                             double theta, double v0,
                             std::vector<cv::Point>& arc_pts) {
    arc_pts.clear();
    arc_pts.push_back(world2canvas(px, py));
    double kappa = std::tan(delta_now) / Lf;
    double x = px, y = py, th = theta, v = v0;
    for (int i = 0; i < (int)N - 1; i++) {
        double s = v * dt;
        x  += s * std::cos(th);
        y  += s * std::sin(th);
        th += kappa * s;
        arc_pts.push_back(world2canvas(x, y));
    }
}

// ---------------------------------------------------------------------------
// Draw vehicle rectangle oriented by theta
// ---------------------------------------------------------------------------
void drawVehicle(cv::Mat& canvas, double px, double py, double theta) {
    const double len = 4.5 * SCALE;
    const double wid = 2.0 * SCALE;
    double c = std::cos(theta), s = std::sin(theta);
    cv::Point2f corners[4] = {
        { (float)( len/2*c - wid/2*(-s)), (float)-(len/2*s + wid/2*(-c)) },
        { (float)( len/2*c + wid/2*(-s)), (float)-(len/2*s - wid/2*(-c)) },
        { (float)(-len/2*c + wid/2*(-s)), (float)-(-len/2*s - wid/2*(-c)) },
        { (float)(-len/2*c - wid/2*(-s)), (float)-(-len/2*s + wid/2*(-c)) }
    };
    cv::Point cp = world2canvas(px, py);
    std::vector<cv::Point> poly(4);
    for (int i = 0; i < 4; i++)
        poly[i] = cp + cv::Point((int)corners[i].x, (int)corners[i].y);
    cv::fillConvexPoly(canvas, poly, COL_VEHICLE);
    cv::polylines(canvas, poly, true, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
}

// ---------------------------------------------------------------------------
// HUD
// ---------------------------------------------------------------------------
void drawHUD(cv::Mat& canvas, int step, double cte, double epsi,
             double v, double delta_cmd, double a_cmd, double kappa_cmd) {
    auto put = [&](const std::string& txt, int row, cv::Scalar col = COL_TEXT) {
        cv::putText(canvas, txt, cv::Point(15, 25 + row * 22),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, col, 1, cv::LINE_AA);
    };
    put("Step : " + std::to_string(step),                                        0);
    put("v    : " + std::to_string(v).substr(0,5) + " m/s",                      1);
    put("CTE  : " + std::to_string(cte).substr(0,6) + " m",                      2);
    put("eYaw : " + std::to_string(epsi * 180.0/M_PI).substr(0,5) + " deg",      3);
    put("delta: " + std::to_string(delta_cmd * 180.0/M_PI).substr(0,5) + " deg", 4);
    put("accel: " + std::to_string(a_cmd).substr(0,5),                           5);
    put("kappa: " + std::to_string(kappa_cmd).substr(0,7),                       6);

    int lx = W - 230, ly = 20;
    auto leg = [&](const std::string& lbl, cv::Scalar col, int row) {
        cv::line(canvas, {lx, ly+row*22}, {lx+30, ly+row*22}, col, 2, cv::LINE_AA);
        cv::putText(canvas, lbl, {lx+35, ly+row*22+5},
                    cv::FONT_HERSHEY_SIMPLEX, 0.50, col, 1, cv::LINE_AA);
    };
    leg("Reference path",    COL_REF,        0);
    leg("MPC predicted arc", COL_PRED,        1);
    leg("Current steer arc", COL_CURRENT_ARC, 2);
    leg("Vehicle",           COL_VEHICLE,     3);
}

// ---------------------------------------------------------------------------
// Grid — scrolls with camera
// ---------------------------------------------------------------------------
void drawGrid(cv::Mat& canvas) {
    const int GRID_PX = (int)(10.0 * SCALE);
    int ox = ((int)(cam_x * SCALE) % GRID_PX + GRID_PX) % GRID_PX;
    int oy = ((int)(cam_y * SCALE) % GRID_PX + GRID_PX) % GRID_PX;
    for (int gx = ORIGIN_X - ox; gx < W; gx += GRID_PX)
        cv::line(canvas, {gx, 0}, {gx, H}, COL_GRID, 1);
    for (int gy = ORIGIN_Y - oy; gy < H; gy += GRID_PX)
        cv::line(canvas, {0, gy}, {W, gy}, COL_GRID, 1);
}

// ---------------------------------------------------------------------------
// MAIN
// ---------------------------------------------------------------------------
int main(int argc, char* argv[]) {

    std::cout << "MPC Test — curvature-based [cte, epsi, v]" << std::endl;

    int scenario = (argc > 1) ? std::atoi(argv[1]) : 2;
    std::string scenario_name;

    const int    PATH_PTS = 500;
    const double DS       = 0.5;

    std::vector<double> ref_x, ref_y;
    Eigen::VectorXd     kappa_ref(N);

    switch (scenario) {
        case 0: buildStraightPath(ref_x, ref_y, kappa_ref, PATH_PTS, DS);
                scenario_name = "Straight road"; break;
        case 1: buildCirclePath  (ref_x, ref_y, kappa_ref, PATH_PTS, DS);
                scenario_name = "Constant radius turn (R=30m)"; break;
        default:buildSinePath    (ref_x, ref_y, kappa_ref, PATH_PTS, DS);
                scenario_name = "S-bend (sinusoidal)"; break;
    }

    // ------------------------------------------------------------------
    // Precompute curvature for every point on the reference path
    // ------------------------------------------------------------------
    std::vector<double> path_kappa(PATH_PTS, 0.0);
    for (int i = 0; i < PATH_PTS - 2; i++) {
        double dx1 = ref_x[i+1] - ref_x[i];
        double dy1 = ref_y[i+1] - ref_y[i];
        double dx2 = ref_x[i+2] - ref_x[i+1];
        double dy2 = ref_y[i+2] - ref_y[i+1];
        double cross = dx1 * dy2 - dy1 * dx2;
        double norm  = std::pow(dx1*dx1 + dy1*dy1, 1.5);
        path_kappa[i] = (norm > 1e-9) ? cross / norm : 0.0;
    }
    path_kappa[PATH_PTS-2] = path_kappa[PATH_PTS-3];
    path_kappa[PATH_PTS-1] = path_kappa[PATH_PTS-3];

    // ------------------------------------------------------------------
    // Initial state — vehicle placed exactly on the path, zero offset
    // cte=0, epsi=0: vehicle centre coincides with the yellow line
    // ------------------------------------------------------------------
    double cte  = 0.0;   // no lateral offset — on the line from step 0
    double epsi = 0.0;   // heading aligned with path tangent
    double v    = 10.0;

    // World position = first path point exactly
    double veh_x = ref_x[0];
    double veh_y = ref_y[0];

    // Heading from path tangent at start
    double dx0     = ref_x[1] - ref_x[0];
    double dy0     = ref_y[1] - ref_y[0];
    double veh_yaw = std::atan2(dy0, dx0);

    // Seed camera to vehicle
    cam_x = veh_x;
    cam_y = veh_y;

    LateralPlanner mpc;
    // const int SIM_STEPS = 2000;
    const int SIM_STEPS = 8000;
    // const int DELAY_MS  = 50;
    const int DELAY_MS  = 100;

    cv::namedWindow("MPC Test: " + scenario_name, cv::WINDOW_NORMAL);
    cv::resizeWindow("MPC Test: " + scenario_name, W, H);

    double delta_cmd = 0.0, a_cmd = 0.0;
    int closest_idx = 0;

    for (int step = 0; step < SIM_STEPS; step++) {

        // 1. Call MPC
        Eigen::VectorXd state(3);
        state << cte, epsi, v;
        auto result = mpc.Solve(state, kappa_ref);

        delta_cmd = result[0];
        a_cmd     = result[1];
        double kappa_cmd = std::tan(delta_cmd) / Lf;

        // 2. Frenet integration
        double kref0 = kappa_ref[0];
        double s     = v * dt;

        cte  = cte + v * std::sin(epsi) * dt;
        epsi = epsi + (kappa_cmd - kref0) * s;

        while (epsi >  M_PI) epsi -= 2.0 * M_PI;
        while (epsi < -M_PI) epsi += 2.0 * M_PI;

        v = std::max(0.1, v + a_cmd * dt);

        // Advance world position along current heading
        veh_x += s * std::cos(veh_yaw);
        veh_y += s * std::sin(veh_yaw);

        // Find closest path point, then snap world position to:
        //   path_point + cte * perp_direction
        // This keeps veh_x/veh_y consistent with the Frenet cte state so
        // the rectangle rides directly on the yellow line when cte ~ 0
        {
            double best = 1e9;
            int end = std::min(PATH_PTS - 2, closest_idx + 80);
            for (int i = std::max(0, closest_idx - 5); i < end; i++) {
                double d = std::hypot(ref_x[i] - veh_x, ref_y[i] - veh_y);
                if (d < best) { best = d; closest_idx = i; }
            }
            double tdx = ref_x[closest_idx+1] - ref_x[closest_idx];
            double tdy = ref_y[closest_idx+1] - ref_y[closest_idx];
            double path_tangent = std::atan2(tdy, tdx);

            // Recompute world position from Frenet state so visual and
            // numerical state are always in sync
            double perp = path_tangent + M_PI / 2.0;
            veh_x   = ref_x[closest_idx] + cte * std::cos(perp);
            veh_y   = ref_y[closest_idx] + cte * std::sin(perp);
            veh_yaw = path_tangent + epsi;
        }

        // Resample kappa_ref from precomputed path curvature ahead of vehicle
        for (int i = 0; i < (int)N; i++) {
            int idx = std::min(closest_idx + i, PATH_PTS - 1);
            kappa_ref[i] = path_kappa[idx];
        }

        // Camera follows vehicle
        cam_x = veh_x;
        cam_y = veh_y;

        // 3. Build arcs for visualisation
        double tdx_arc = ref_x[closest_idx+1] - ref_x[closest_idx];
        double tdy_arc = ref_y[closest_idx+1] - ref_y[closest_idx];
        double path_yaw_arc = std::atan2(tdy_arc, tdx_arc);

        std::vector<cv::Point> pred_arc, const_arc;
        reconstructArc(result, veh_x, veh_y, path_yaw_arc, epsi, v, pred_arc);
        reconstructConstantArc(delta_cmd, veh_x, veh_y, path_yaw_arc + epsi, v, const_arc);

        // 4. Draw
        // Layer order:
        //   (a) grid
        //   (b) full reference path
        //   (c) arcs
        //   (d) vehicle rectangle  ← on top so it covers the line
        //   (e) redraw short yellow segment through vehicle body ← line visible through car
        cv::Mat canvas(H, W, CV_8UC3, COL_BG);
        drawGrid(canvas);

        // (b) Full reference path
        for (int i = 1; i < PATH_PTS; i++)
            cv::line(canvas,
                     world2canvas(ref_x[i-1], ref_y[i-1]),
                     world2canvas(ref_x[i],   ref_y[i]),
                     COL_REF, 2, cv::LINE_AA);

        // (c) Constant-kappa arc (dashed)
        for (int i = 1; i < (int)const_arc.size(); i += 2)
            if (i + 1 < (int)const_arc.size())
                cv::line(canvas, const_arc[i], const_arc[i+1],
                         COL_CURRENT_ARC, 1, cv::LINE_AA);

        // (c) MPC predicted arc
        for (int i = 1; i < (int)pred_arc.size(); i++)
            cv::line(canvas, pred_arc[i-1], pred_arc[i], COL_PRED, 2, cv::LINE_AA);
        for (auto& pt : pred_arc)
            cv::circle(canvas, pt, 3, COL_PRED, -1, cv::LINE_AA);

        // (d) Vehicle rectangle drawn on top of everything
        drawVehicle(canvas, veh_x, veh_y, veh_yaw);

        // (e) Redraw the yellow line through the vehicle centre so it is always
        //     visible passing along the longitudinal axis of the rectangle
        {
            int lo = std::max(0,          closest_idx - 6);
            int hi = std::min(PATH_PTS-1, closest_idx + 8);
            for (int i = lo + 1; i <= hi; i++)
                cv::line(canvas,
                         world2canvas(ref_x[i-1], ref_y[i-1]),
                         world2canvas(ref_x[i],   ref_y[i]),
                         COL_REF, 2, cv::LINE_AA);
        }

        cv::putText(canvas, "Scenario: " + scenario_name,
                    cv::Point(W/2 - 160, H - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, COL_TEXT, 1, cv::LINE_AA);

        drawHUD(canvas, step, cte, epsi, v, delta_cmd, a_cmd, kappa_cmd);
        cv::imshow("MPC Test: " + scenario_name, canvas);

        if (step % 10 == 0)
            std::printf("step=%3d  cte=%+6.3f m  epsi=%+6.3f deg  v=%5.2f m/s"
                        "  delta=%+5.2f deg  a=%+5.3f\n",
                        step, cte, epsi * 180.0/M_PI, v,
                        delta_cmd * 180.0/M_PI, a_cmd);

        int key = cv::waitKey(DELAY_MS);
        if (key == 27 || key == 'q') break;

        if (std::abs(cte) < 0.01 && std::abs(epsi) < 0.001 && step % 20 == 0)
            std::cout << "Tracking tight at step " << step
                      << "  cte=" << cte << "  epsi=" << epsi * 180.0/M_PI << " deg" << std::endl;

        if (closest_idx >= PATH_PTS - 3) {
            std::cout << "End of path reached at step " << step << std::endl;
            cv::waitKey(1500);
            break;
        }
    }

    cv::waitKey(0);
    cv::destroyAllWindows();
    std::cout << "Done." << std::endl;
    return 0;
}