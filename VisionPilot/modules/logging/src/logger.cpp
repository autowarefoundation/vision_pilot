#include <logging/logger.hpp>

#include <models/inference.hpp>
#include <common/types.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>

namespace fs = std::filesystem;

namespace logging {

namespace {
std::string g_base_dir;
std::mutex  g_mutex;

std::string make_frame_dir(uint64_t frame_id) {
	std::ostringstream ss;
	ss << g_base_dir << "/frame_" << std::setfill('0') << std::setw(6) << frame_id;
	return ss.str();
}

void write_bytes(const fs::path& p, const std::vector<uchar>& data) {
	std::ofstream ofs(p.generic_string(), std::ios::binary);
	ofs.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
}

std::string escape_json(const std::string& s) {
	std::string out;
	for (char c : s) {
		if (c == '"') out += "\\\"";
		else if (c == '\\') out += "\\\\";
		else if (c == '\b') out += "\\b";
		else if (c == '\f') out += "\\f";
		else if (c == '\n') out += "\\n";
		else if (c == '\r') out += "\\r";
		else if (c == '\t') out += "\\t";
		else out += c;
	}
	return out;
}

} // namespace

void Rerun::init(const std::string& out_dir) {
	std::lock_guard<std::mutex> lk(g_mutex);
	g_base_dir = out_dir;
	if (g_base_dir.empty()) g_base_dir = "rerun_logs";
	fs::create_directories(g_base_dir);
}

void Rerun::log_frame_images(uint64_t frame_id,
							 const cv::Mat& frame,
							 const cv::Mat& warped,
							 const cv::Mat& resized) {
	std::lock_guard<std::mutex> lk(g_mutex);
	if (g_base_dir.empty()) init();
	const std::string dir = make_frame_dir(frame_id);
	fs::create_directories(dir + "/images");

	auto write_image = [&](const cv::Mat& im, const std::string& name) {
		if (im.empty()) return;
		std::vector<uchar> buf;
		std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 3};
		cv::Mat bgr = im;
		if (im.channels() == 4) cv::cvtColor(im, bgr, cv::COLOR_BGRA2BGR);
		cv::imencode(".png", bgr, buf, params);
		write_bytes(fs::path(dir) / "images" / (name + ".png"), buf);
	};

	write_image(frame, "camera_frame");
	write_image(warped, "warped_bev");
	write_image(resized, "resized");
}

void Rerun::log_inference(uint64_t frame_id, const visionpilot::models::InferenceFrameResult& r) {
	std::lock_guard<std::mutex> lk(g_mutex);
	if (g_base_dir.empty()) init();
	const std::string dir = make_frame_dir(frame_id);
	fs::create_directories(dir);

	std::ofstream meta(fs::path(dir) / "inference.json");
	meta << "{\n";
	meta << "  \"frame_id\": " << r.frame_id << ",\n";
	meta << "  \"wall_ms\": " << r.wall_ms << ",\n";
	meta << "  \"pre_ms\": " << r.pre_ms << ",\n";
	meta << "  \"ad_ms\": " << r.ad_ms << ",\n";
	meta << "  \"as_ms\": " << r.as_ms << ",\n";
	meta << "  \"asp_ms\": " << r.asp_ms << ",\n";

	// AutoDrive
	meta << "  \"auto_drive\": {\n";
	meta << "    \"dist_normalized\": " << r.auto_drive.dist_normalized << ",\n";
	meta << "    \"curvature_raw\": " << r.auto_drive.curvature_raw << ",\n";
	meta << "    \"flag_prob\": " << r.auto_drive.flag_prob << ",\n";
	meta << "    \"valid\": " << (r.auto_drive.valid ? "true" : "false") << "\n";
	meta << "  },\n";

	// AutoSteer
	meta << "  \"auto_steer\": {\n";
	meta << "    \"xp\": [";
	for (size_t i = 0; i < r.auto_steer.xp.size(); ++i) {
		if (i) meta << ", ";
		meta << r.auto_steer.xp[i];
	}
	meta << "],\n";
	meta << "    \"h_vector\": [";
	for (size_t i = 0; i < r.auto_steer.h_vector.size(); ++i) {
		if (i) meta << ", ";
		meta << r.auto_steer.h_vector[i];
	}
	meta << "],\n";
	meta << "    \"valid\": " << (r.auto_steer.valid ? "true" : "false") << "\n";
	meta << "  },\n";

	// AutoSpeed detections
	meta << "  \"auto_speed\": {\n";
	meta << "    \"detections\": [\n";
	for (size_t i = 0; i < r.auto_speed.detections.size(); ++i) {
		const auto& d = r.auto_speed.detections[i];
		meta << "      {\"x1\": " << d.x1 << ", \"y1\": " << d.y1
			 << ", \"x2\": " << d.x2 << ", \"y2\": " << d.y2
			 << ", \"score\": " << d.score << ", \"class_id\": " << d.class_id << " }";
		if (i + 1 < r.auto_speed.detections.size()) meta << ",\n";
		else meta << "\n";
	}
	meta << "    ],\n";
	meta << "    \"valid\": " << (r.auto_speed.valid ? "true" : "false") << "\n";
	meta << "  },\n";

	// CIPOFusion
	meta << "  \"cipo\": {\n";
	meta << "    \"valid\": " << (r.cipo.valid ? "true" : "false") << ",\n";
	meta << "    \"distance_m\": " << r.cipo.distance_m << ",\n";
	meta << "    \"velocity_ms\": " << r.cipo.velocity_ms << "\n";
	meta << "  },\n";

	// LateralFusion
	meta << "  \"lateral\": {\n";
	meta << "    \"valid\": " << (r.lateral.valid ? "true" : "false") << ",\n";
	meta << "    \"path_valid\": " << (r.lateral.path_valid ? "true" : "false") << ",\n";
	meta << "    \"path_a\": " << r.lateral.path_a << ",\n";
	meta << "    \"path_b\": " << r.lateral.path_b << ",\n";
	meta << "    \"path_c\": " << r.lateral.path_c << ",\n";
	// Fused (tracked) scalars ready for planning
	meta << "    \"cte_m\": " << r.lateral.cte_m << ",\n";
	meta << "    \"yaw_rad\": " << r.lateral.yaw_rad << ",\n";
	meta << "    \"curvature\": " << r.lateral.curvature << ",\n";
	meta << "    \"cte_stddev_m\": " << r.lateral.cte_stddev_m << ",\n";
	meta << "    \"yaw_stddev_rad\": " << r.lateral.yaw_stddev_rad << ",\n";
	meta << "    \"curv_stddev\": " << r.lateral.curv_stddev << ",\n";
	// Raw auxiliaries
	meta << "    \"raw_cte_m\": " << r.lateral.raw_cte_m << ",\n";
	meta << "    \"raw_yaw_rad\": " << r.lateral.raw_yaw_rad << ",\n";
	meta << "    \"raw_path_curvature\": " << r.lateral.raw_path_curvature << ",\n";
	meta << "    \"raw_ad_curvature\": " << r.lateral.raw_ad_curvature << ",\n";
	meta << "    \"path_x_min_m\": " << r.lateral.path_x_min_m << ",\n";
	meta << "    \"path_x_max_m\": " << r.lateral.path_x_max_m << ",\n";
	// BEV coords of filtered path, sampled from y = ax^2 + bx + c in
	// world/BEV frame (x = forward (m), y = lateral (m, left as +)).
	meta << "    \"path_bev\": [";
	{
		const float a = r.lateral.path_a;
		const float b = r.lateral.path_b;
		const float c = r.lateral.path_c;
		float x0 = r.lateral.path_x_min_m;
		float x1 = r.lateral.path_x_max_m;
		// Fallback range when extent unknown
		if (!(x1 > x0)) { 
			x0 = 2.f; 
			x1 = 40.f; 
		}
		constexpr int N = 30;
		for (int i = 0; i < N; ++i) {
			const float x = x0 + (x1 - x0) * (static_cast<float>(i) / (N - 1));
			const float y = a * x * x + b * x + c;
			if (i) meta << ", ";
			meta << "[" << x << ", " << y << "]";
		}
	}
	meta << "]\n";
	meta << "  }\n";

	meta << "}\n";
}

void Rerun::log_plan(uint64_t frame_id, const Plan& p) {
	std::lock_guard<std::mutex> lk(g_mutex);
	if (g_base_dir.empty()) init();
	const std::string dir = make_frame_dir(frame_id);
	fs::create_directories(dir);

	std::ofstream meta(fs::path(dir) / "plan.json");
	meta << "{\n";
	meta << "  \"acceleration\": " << p.acceleration << ",\n";
	meta << "  \"steering\": [";
	for (size_t i = 0; i < p.steering.size(); ++i) {
		if (i) meta << ", ";
		meta << p.steering[i];
	}
	meta << "],\n";
	meta << "  \"warnings\": [";
	for (size_t i = 0; i < p.warnings.size(); ++i) {
		if (i) meta << ", ";
		meta << static_cast<int>(p.warnings[i]);
	}
	meta << "]\n";
	meta << "}\n";
}

void Rerun::log_ego_speed(
	uint64_t frame_id, 
	double ego_speed_ms
) {
	std::lock_guard<std::mutex> lk(g_mutex);
	if (g_base_dir.empty()) init();
	const std::string dir = make_frame_dir(frame_id);
	fs::create_directories(dir);

	std::ofstream meta(fs::path(dir) / "vehicle.json");
	meta << "{\n";
	meta << "  \"ego_speed_ms\": " << ego_speed_ms << "\n";
	meta << "}\n";
}

void Rerun::log_visualization(
	uint64_t frame_id, 
	const cv::Mat& viz
) {
	std::lock_guard<std::mutex> lk(g_mutex);
	if (viz.empty()) return;
	if (g_base_dir.empty()) init();
	const std::string dir = make_frame_dir(frame_id);
	fs::create_directories(dir + "/images");

	std::vector<uchar> buf;
	std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 3};
	cv::Mat bgr = viz;
	if (viz.channels() == 4) cv::cvtColor(viz, bgr, cv::COLOR_BGRA2BGR);
	cv::imencode(".png", bgr, buf, params);
	write_bytes(fs::path(dir) / "images" / "visualization.png", buf);
}

void Rerun::shutdown() {
	std::lock_guard<std::mutex> lk(g_mutex);
	// no-op for now
}

} // namespace logging

