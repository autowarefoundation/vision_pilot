#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <planning/planning.hpp>
#include <opencv2/opencv.hpp>

static const int    W        = 1200;
static const int    H        =  700;
static const double SCALE    =  8.0;
static const int    ORIGIN_X = W / 2;
static const int    ORIGIN_Y = H / 2;

static double cam_x = 0.0;
static double cam_y = 0.0;

static const cv::Scalar COL_BG         ( 20,  20,  20);
static const cv::Scalar COL_GRID       ( 45,  45,  45);
static const cv::Scalar COL_REF        (  0, 255, 255);
static const cv::Scalar COL_PRED       (  0, 200,   0);
static const cv::Scalar COL_CURRENT_ARC( 80,  80, 255);
static const cv::Scalar COL_VEHICLE    (255, 255, 255);
static const cv::Scalar COL_LEAD       ( 30, 140, 255);
static const cv::Scalar COL_TEXT       (220, 220, 220);
static const cv::Scalar COL_MATCHED    (  0, 220,   0);
static const cv::Scalar COL_GAP_LINE   (100, 100, 100);

static const int    LEAD_SPAWN_STEP  = 50;
static const double LEAD_SPEED       = 5.0;
static const double LEAD_SPAWN_AHEAD = 30.0;
static const double VEHICLE_LENGTH   = 4.5;

static const double SPEED_LIMIT = 16.666; // m/s
static const double Lf = 2.67;            // Front axle to CoG (m). Wheelbase L = Lf + Lr


cv::Point world2canvas(double x, double y) {
    return cv::Point(
        ORIGIN_X + static_cast<int>((x - cam_x) * SCALE),
        ORIGIN_Y - static_cast<int>((y - cam_y) * SCALE)
    );
}

void buildStraightPath(std::vector<double>& rx, std::vector<double>& ry,
                       std::vector<double>& kappa_path, int nPts, double ds) {
    rx.resize(nPts); ry.resize(nPts);
    kappa_path.assign(nPts, 0.0);
    for (int i = 0; i < nPts; i++) { rx[i] = 0.0; ry[i] = i * ds; }
}

void buildCirclePath(std::vector<double>& rx, std::vector<double>& ry,
                     std::vector<double>& kappa_path, int nPts, double ds) {
    const double R = 30.0;
    rx.resize(nPts); ry.resize(nPts);
    kappa_path.assign(nPts, 1.0 / R);
    for (int i = 0; i < nPts; i++) {
        double angle = i * ds / R;
        rx[i] =  R * std::sin(angle);
        ry[i] = -R * std::cos(angle) + R;
    }
}

void buildSinePath(std::vector<double>& rx, std::vector<double>& ry,
                   std::vector<double>& kappa_path, int nPts, double ds) {
    const double A  = 6.0;
    const double wl = 40.0;
    const double w  = 2.0 * M_PI / wl;
    rx.resize(nPts); ry.resize(nPts);
    kappa_path.resize(nPts);
    for (int i = 0; i < nPts; i++) {
        double s  = i * ds;
        rx[i] = A * std::sin(w * s);
        ry[i] = s;
    }
    for (int i = 0; i < nPts; i++) {
        double s   = i * ds;
        double xp  =  A * w     * std::cos(w * s);
        double xpp = -A * w * w * std::sin(w * s);
        // SIGNED heading-rate curvature (CCW positive), per the MPC's epsi
        // convention.  The path is x = g(y) with y the parameter, so the
        // signed curvature is  -x'' / (1 + x'^2)^1.5 .  The leading minus is
        // essential: without it the curvature handed to the MPC is negated,
        // the steering feedforward fights the feedback, and the predicted arc
        // bends the wrong way (while strong cte feedback still holds the line).
        kappa_path[i] = -xpp / std::pow(1.0 + xp * xp, 1.5);
    }
}

// Arc reconstruction uses the full MPC result vector
void reconstructArc(const std::vector<double>& mpc_result,
                    const Eigen::VectorXd& v_schedule,
                    double px, double py, double path_yaw, double epsi,
                    std::vector<cv::Point>& arc_pts) {
    arc_pts.clear();
    arc_pts.push_back(world2canvas(px, py));
    double x = px, y = py, th = path_yaw + epsi;
    for (int i = 0; i < (int)N - 1; i++) {
        double delta_i = mpc_result[1 + i];
        double kappa_i = std::tan(delta_i) / Lf;
        double s = v_schedule[i] * dt;
        x  += s * std::cos(th);
        y  += s * std::sin(th);
        th += kappa_i * s;
        arc_pts.push_back(world2canvas(x, y));
    }
}

void reconstructConstantArc(double delta_now, double px, double py,
                             double theta, double v,
                             std::vector<cv::Point>& arc_pts) {
    arc_pts.clear();
    arc_pts.push_back(world2canvas(px, py));
    double kappa = std::tan(delta_now) / Lf;
    double x = px, y = py, th = theta;
    for (int i = 0; i < (int)N - 1; i++) {
        double s = v * dt;
        x  += s * std::cos(th);
        y  += s * std::sin(th);
        th += kappa * s;
        arc_pts.push_back(world2canvas(x, y));
    }
}

void drawVehicle(cv::Mat& canvas, double px, double py, double theta,
                 cv::Scalar fill = cv::Scalar(255, 255, 255)) {
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
    cv::fillConvexPoly(canvas, poly, fill);
    cv::polylines(canvas, poly, true, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
}

void drawLeadVehicle(cv::Mat& canvas,
                     double lead_px, double lead_py, double lead_theta,
                     double ego_px,  double ego_py,
                     double arc_gap) {
    drawVehicle(canvas, lead_px, lead_py, lead_theta, COL_LEAD);

    cv::Point lead_centre = world2canvas(lead_px, lead_py);
    std::string spd_label = std::to_string(LEAD_SPEED).substr(0, 3) + " m/s";
    int baseline = 0;
    cv::Size ts = cv::getTextSize(spd_label, cv::FONT_HERSHEY_SIMPLEX,
                                  0.45, 1, &baseline);
    cv::Point spd_pos = lead_centre + cv::Point(-ts.width / 2, -24);
    cv::rectangle(canvas,
                  spd_pos + cv::Point(-3, -ts.height - 2),
                  spd_pos + cv::Point(ts.width + 3, baseline + 2),
                  cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(canvas, spd_label, spd_pos,
                cv::FONT_HERSHEY_SIMPLEX, 0.45, COL_LEAD, 1, cv::LINE_AA);

    cv::Point ego_pt  = world2canvas(ego_px,  ego_py);
    cv::Point lead_pt = world2canvas(lead_px, lead_py);
    double dx  = lead_pt.x - ego_pt.x;
    double dy  = lead_pt.y - ego_pt.y;
    double len = std::hypot(dx, dy);
    if (len > 1.0) {
        int n_dashes = std::max(1, (int)(len / 12));
        for (int d = 0; d < n_dashes; d += 2) {
            double t0 = (double)d       / n_dashes;
            double t1 = (double)(d + 1) / n_dashes;
            cv::Point p0(ego_pt.x + (int)(dx * t0), ego_pt.y + (int)(dy * t0));
            cv::Point p1(ego_pt.x + (int)(dx * t1), ego_pt.y + (int)(dy * t1));
            cv::line(canvas, p0, p1, COL_GAP_LINE, 1, cv::LINE_AA);
        }
    }

    cv::Point mid((ego_pt.x + lead_pt.x) / 2, (ego_pt.y + lead_pt.y) / 2);
    std::string gap_label = "gap=" + std::to_string(arc_gap).substr(0, 4) + "m";
    cv::Size gs = cv::getTextSize(gap_label, cv::FONT_HERSHEY_SIMPLEX,
                                  0.40, 1, &baseline);
    cv::Point gap_pos = mid + cv::Point(-gs.width / 2, -6);
    cv::rectangle(canvas,
                  gap_pos + cv::Point(-2, -gs.height - 1),
                  gap_pos + cv::Point(gs.width + 2, baseline + 1),
                  cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(canvas, gap_label, gap_pos,
                cv::FONT_HERSHEY_SIMPLEX, 0.40, COL_GAP_LINE, 1, cv::LINE_AA);
}

void drawHUD(cv::Mat& canvas, int step, double cte, double epsi,
             double v, double delta_cmd, double kappa_cmd,
             double accel, bool lead_active, double gap, double lead_v) {
    auto put = [&](const std::string& txt, int row, cv::Scalar col = COL_TEXT) {
        cv::putText(canvas, txt, cv::Point(15, 25 + row * 22),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, col, 1, cv::LINE_AA);
    };
    put("Step : " + std::to_string(step),                                        0);
    put("v ego: " + std::to_string(v).substr(0,5) + " m/s",                      1);
    if (lead_active) {
        bool matched   = std::abs(v - lead_v) < 0.5;
        cv::Scalar col = matched ? COL_MATCHED : COL_LEAD;
        std::string tag = matched ? " [matched]" : "";
        put("v_lead: " + std::to_string(lead_v).substr(0,5) + " m/s" + tag,      2, col);
    } else {
        put("v_lead: --  (free road)",                                            2,
            cv::Scalar(80, 80, 80));
    }
    put("accel: " + std::to_string(accel).substr(0,6) + " m/s2",                 3);
    put("CTE  : " + std::to_string(cte).substr(0,6) + " m",                      4);
    put("eYaw : " + std::to_string(epsi * 180.0/M_PI).substr(0,5) + " deg",      5);
    put("delta: " + std::to_string(delta_cmd * 180.0/M_PI).substr(0,5) + " deg", 6);
    put("kappa: " + std::to_string(kappa_cmd).substr(0,7),                        7);
    if (lead_active) {
        cv::Scalar gap_col = (gap < 8.0) ? cv::Scalar(50, 50, 230) : COL_TEXT;
        put("gap(s): " + std::to_string(gap).substr(0,5) + " m  [arc]",          8, gap_col);
    } else {
        put("lead : left path — free road",                                       8,
            cv::Scalar(120, 120, 120));
    }

    int lx = W - 230, ly = 20;
    auto leg = [&](const std::string& lbl, cv::Scalar col, int row) {
        cv::line(canvas, {lx, ly+row*22}, {lx+30, ly+row*22}, col, 2, cv::LINE_AA);
        cv::putText(canvas, lbl, {lx+35, ly+row*22+5},
                    cv::FONT_HERSHEY_SIMPLEX, 0.50, col, 1, cv::LINE_AA);
    };
    leg("Reference path",    COL_REF,         0);
    leg("MPC predicted arc", COL_PRED,         1);
    leg("Current steer arc", COL_CURRENT_ARC,  2);
    leg("Ego vehicle",       COL_VEHICLE,      3);
    if (lead_active)
        leg("Lead (const spd)", COL_LEAD,      4);
}

void drawGrid(cv::Mat& canvas) {
    const int GRID_PX = (int)(10.0 * SCALE);
    int ox = ((int)(cam_x * SCALE) % GRID_PX + GRID_PX) % GRID_PX;
    int oy = ((int)(cam_y * SCALE) % GRID_PX + GRID_PX) % GRID_PX;
    for (int gx = ORIGIN_X - ox; gx < W; gx += GRID_PX)
        cv::line(canvas, {gx, 0}, {gx, H}, COL_GRID, 1);
    for (int gy = ORIGIN_Y - oy; gy < H; gy += GRID_PX)
        cv::line(canvas, {0, gy}, {W, gy}, COL_GRID, 1);
}

int main(int argc, char* argv[]) {

    std::cout << "Test vehicle longitudinal and lateral planning" << std::endl;

    int scenario = (argc > 1) ? std::atoi(argv[1]) : 2;
    std::string scenario_name;

    const int    PATH_PTS   = 500;
    const double DS         = 0.5;
    const double LEAD_S_MAX = (PATH_PTS - 2) * DS;

    std::vector<double> ref_x, ref_y, path_kappa;

    switch (scenario) {
        case 0: buildStraightPath(ref_x, ref_y, path_kappa, PATH_PTS, DS);
                scenario_name = "Straight road"; break;
        case 1: buildCirclePath  (ref_x, ref_y, path_kappa, PATH_PTS, DS);
                scenario_name = "Constant radius turn (R=30m)"; break;
        default:buildSinePath    (ref_x, ref_y, path_kappa, PATH_PTS, DS);
                scenario_name = "S-bend (sinusoidal)"; break;
    }

    Planner planner(SPEED_LIMIT, Lf); // speed_limit in m/s, Front axle to CoG (m). Wheelbase L = Lf + Lr

    double cte   = 0.0;
    double epsi  = 0.0;
    double v     = 10.0;
    double accel = 0.0;

    double veh_x   = ref_x[0];
    double veh_y   = ref_y[0];
    double veh_yaw = std::atan2(ref_y[1] - ref_y[0], ref_x[1] - ref_x[0]);

    cam_x = veh_x;
    cam_y = veh_y;

    double ego_s = 0.0;

    bool   lead_active   = false;
    double lead_s        = 0.0;
    double gap           = 9999.0;
    double lead_v_actual = LEAD_SPEED;

    const double SPEED_LIMIT = 60.0 / 3.6;

    const int SIM_STEPS = 8000;
    const int DELAY_MS  =  100;

    cv::namedWindow("MPC Test: " + scenario_name, cv::WINDOW_NORMAL);
    cv::resizeWindow("MPC Test: " + scenario_name, W, H);

    double delta_cmd   = 0.0;
    int    closest_idx = 0;

    Eigen::VectorXd v_schedule_last = Eigen::VectorXd::Constant(N, v);

    for (int step = 0; step < SIM_STEPS; step++) {

        // Lead vehicle logic
        if (step == LEAD_SPAWN_STEP) {
            lead_active   = true;
            lead_v_actual = LEAD_SPEED;
            lead_s        = ego_s + LEAD_SPAWN_AHEAD;
            std::cout << "Lead vehicle spawned at s=" << lead_s
                      << " m  (ego at s=" << ego_s << " m)" << std::endl;
        }

        if (lead_active) {
            lead_s += LEAD_SPEED * dt;
            if (lead_s > LEAD_S_MAX) {
                lead_active = false;
                gap         = 9999.0;
                std::cout << "Lead vehicle left path at step " << step
                          << "  (ego at s=" << ego_s << " m)" << std::endl;
            }
        }

        if (lead_active) {
            double lead_rear_s = lead_s - VEHICLE_LENGTH / 2.0;
            double ego_front_s = ego_s  + VEHICLE_LENGTH / 2.0;
            gap = std::max(0.5, lead_rear_s - ego_front_s);
        }

        // Current road curvature for the planner (single scalar)
        // Sample path_kappa at the ego's TRUE continuous arc-length (the snapped
        // index plus the projection onto the current segment) and interpolate,
        // rather than reading path_kappa[closest_idx] directly.  A snapped scalar
        // would step every sample as the car advances and make the prediction
        // twitch; the interpolated value changes smoothly.
        double kappa_road;
        {
            double sx = ref_x[closest_idx + 1] - ref_x[closest_idx];
            double sy = ref_y[closest_idx + 1] - ref_y[closest_idx];
            double seg2 = sx * sx + sy * sy;
            double tproj = 0.0;
            if (seg2 > 1e-9) {
                tproj = ((veh_x - ref_x[closest_idx]) * sx +
                         (veh_y - ref_y[closest_idx]) * sy) / seg2;
                tproj = std::max(0.0, std::min(1.0, tproj));
            }
            int    j = closest_idx;
            double f = tproj;
            if (j >= PATH_PTS - 1) { j = PATH_PTS - 2; f = 1.0; }
            kappa_road = path_kappa[j] * (1.0 - f) + path_kappa[j + 1] * f;
        }

        // Unified planner call
        double cipo_v    = lead_active ? lead_v_actual : SPEED_LIMIT;
        double cipo_dist = lead_active ? gap           : 9999.0;

        Plan plan = planner.compute_plan(
            cte, epsi,
            kappa_road,   // current signed road curvature (interpolated, smooth)
            v,
            cipo_dist < 9000.0,
            cipo_v,
            cipo_dist
        );

        accel     = plan.acceleration;
        delta_cmd = plan.steering[0];
        double kappa_cmd = std::tan(delta_cmd) / Lf;

        // Rebuild v_schedule locally for arc reconstruction
        {
            double v_sim   = v;
            double gap_sim = cipo_dist;
            for (int i = 0; i < (int)N; i++) {
                v_schedule_last[i] = v_sim;
                double a_sim = (v_sim < SPEED_LIMIT) ? plan.acceleration : 0.0;
                if (gap_sim < 9000.0)
                    gap_sim = std::max(0.5, gap_sim + (cipo_v - v_sim) * dt);
                v_sim = std::max(0.0, v_sim + a_sim * dt);
            }
        }

        // Integrate velocity (post-planner)
        v = std::max(0.0, v + accel * dt);
        if (v < 0.01 && gap > 3.0) v = std::max(v, 0.01);

        // Kinematic state update
        double kref0 = path_kappa[closest_idx];
        double s     = v * dt;
        ego_s += s;

        cte  = cte  + v * std::sin(epsi) * dt;
        epsi = epsi + (kappa_cmd - kref0) * s;

        while (epsi >  M_PI) epsi -= 2.0 * M_PI;
        while (epsi < -M_PI) epsi += 2.0 * M_PI;

        // Vehicle pose update
        veh_x += s * std::cos(veh_yaw);
        veh_y += s * std::sin(veh_yaw);

        {
            double best   = 1e9;
            int    best_i = closest_idx;
            int    end    = std::min(PATH_PTS - 2, closest_idx + 80);
            for (int i = closest_idx; i < end; i++) {
                double d = std::hypot(ref_x[i] - veh_x, ref_y[i] - veh_y);
                if (d < best) { best = d; best_i = i; }
            }
            closest_idx = best_i;
            double tdx  = ref_x[closest_idx+1] - ref_x[closest_idx];
            double tdy  = ref_y[closest_idx+1] - ref_y[closest_idx];
            veh_yaw     = std::atan2(tdy, tdx) + epsi;
        }

        cam_x = veh_x;
        cam_y = veh_y;

        // Arc reconstruction for visualisation
        double path_yaw_arc;
        {
            double best    = 1e9;
            int    arc_idx = closest_idx;
            int    lo = std::max(0,          closest_idx - 5);
            int    hi = std::min(PATH_PTS-2, closest_idx + 5);
            for (int i = lo; i <= hi; i++) {
                double d = std::hypot(ref_x[i] - veh_x, ref_y[i] - veh_y);
                if (d < best) { best = d; arc_idx = i; }
            }
            path_yaw_arc = std::atan2(ref_y[arc_idx+1] - ref_y[arc_idx],
                                      ref_x[arc_idx+1] - ref_x[arc_idx]);
        }

        std::vector<cv::Point> pred_arc, const_arc;
        reconstructArc(plan.steering, v_schedule_last,
                       veh_x, veh_y, path_yaw_arc, epsi, pred_arc);
        reconstructConstantArc(delta_cmd, veh_x, veh_y,
                               path_yaw_arc + epsi, v, const_arc);

        // Rendering
        cv::Mat canvas(H, W, CV_8UC3, COL_BG);
        drawGrid(canvas);

        for (int i = 1; i < PATH_PTS; i++)
            cv::line(canvas,
                     world2canvas(ref_x[i-1], ref_y[i-1]),
                     world2canvas(ref_x[i],   ref_y[i]),
                     COL_REF, 2, cv::LINE_AA);

        for (int i = 1; i < (int)const_arc.size(); i += 2)
            if (i + 1 < (int)const_arc.size())
                cv::line(canvas, const_arc[i], const_arc[i+1],
                         COL_CURRENT_ARC, 1, cv::LINE_AA);

        for (int i = 1; i < (int)pred_arc.size(); i++)
            cv::line(canvas, pred_arc[i-1], pred_arc[i], COL_PRED, 2, cv::LINE_AA);
        for (auto& pt : pred_arc)
            cv::circle(canvas, pt, 3, COL_PRED, -1, cv::LINE_AA);

        if (lead_active) {
            int    lead_idx = std::clamp((int)(lead_s / DS), 0, PATH_PTS - 2);
            double ldx      = ref_x[lead_idx+1] - ref_x[lead_idx];
            double ldy      = ref_y[lead_idx+1] - ref_y[lead_idx];
            drawLeadVehicle(canvas,
                            ref_x[lead_idx], ref_y[lead_idx],
                            std::atan2(ldy, ldx),
                            veh_x, veh_y, gap);
        }

        drawVehicle(canvas, veh_x, veh_y, veh_yaw);

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

        drawHUD(canvas, step, cte, epsi, v, delta_cmd, kappa_cmd,
                accel, lead_active, gap,
                lead_active ? lead_v_actual : 0.0);

        cv::imshow("MPC Test: " + scenario_name, canvas);

        if (step % 10 == 0) {
            if (lead_active)
                std::printf("step=%3d  cte=%+6.3f m  epsi=%+6.3f deg"
                            "  v_ego=%5.2f m/s  v_lead=%4.1f m/s"
                            "  accel=%+5.2f m/s2  ego_s=%6.1f m  lead_s=%6.1f m"
                            "  gap=%6.2f m  delta=%+5.2f deg\n",
                            step, cte, epsi * 180.0/M_PI, v, lead_v_actual,
                            accel, ego_s, lead_s, gap, delta_cmd * 180.0/M_PI);
            else
                std::printf("step=%3d  cte=%+6.3f m  epsi=%+6.3f deg  v=%5.2f m/s"
                            "  accel=%+5.2f m/s2  free road"
                            "  delta=%+5.2f deg\n",
                            step, cte, epsi * 180.0/M_PI, v,
                            accel, delta_cmd * 180.0/M_PI);
        }

        int key = cv::waitKey(DELAY_MS);
        if (key == 27 || key == 'q') break;

        if (std::abs(cte) < 0.01 && std::abs(epsi) < 0.001 && step % 20 == 0)
            std::cout << "Tracking tight at step " << step
                      << "  cte=" << cte
                      << "  epsi=" << epsi * 180.0/M_PI << " deg\n";

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