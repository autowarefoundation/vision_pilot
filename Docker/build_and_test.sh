#!/usr/bin/env bash
# Configure + build VisionPilot inside the container, then smoke-test the planning core.
# Source is at /workspace/VisionPilot (bind-mounted from the host by run.sh).
#
# The tree's tests/ are interactive OpenCV demos (they open a window and block on
# waitKey(0); image_preprocessing_tests needs an image arg; test_vision_pilot needs a
# config + ONNX weights). So the pass/fail gate here is a clean BUILD of every target,
# plus a bounded headless smoke of test_planning asserting the MPC converges. Run the
# visual demos themselves interactively via run.sh with the shared X display.
set -euo pipefail

VP_DIR="${VP_DIR:-/workspace/VisionPilot}"
# Default OFF: the video/v4l2 build compiles the app + all tests without ROS2.
# Set ENABLE_ROS2_INTERFACE=ON to also build the ROS2 camera/vehicle interface.
ENABLE_ROS2_INTERFACE="${ENABLE_ROS2_INTERFACE:-OFF}"
# Own build dir so the container never collides with a host-created build/ cache.
BUILD_DIR="${BUILD_DIR:-build_docker}"

cd "${VP_DIR}"

echo "[build_and_test] configure  (ROS2=${ENABLE_ROS2_INTERFACE}, ORT=${ONNXRUNTIME_ROOT}, dir=${BUILD_DIR})"
# Ubuntu's libcppad-dev keeps temp_file()/error handlers in a compiled lib (libcppad_lib),
# so every executable linking `planning` needs -lcppad_lib. Inject it here rather than
# editing the in-tree CMake (which targets a header-only CppAD build). STANDARD_LIBRARIES
# is appended at the END of each link line, after libplanning.a, so static ordering resolves.
cmake -B "${BUILD_DIR}" \
    -DENABLE_ROS2_INTERFACE="${ENABLE_ROS2_INTERFACE}" \
    -DONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT}" \
    -DCMAKE_CXX_STANDARD_LIBRARIES="-lcppad_lib"

echo "[build_and_test] build"
cmake --build "${BUILD_DIR}" -j"$(nproc)"
echo "[build_and_test] build OK — all targets compiled"

# --- Headless smoke: planning / MPC core ----------------------------------
# test_planning renders a window and ends on waitKey(0); run it offscreen under a
# timeout and assert the MPC reached steady-state tracking ("Tracking tight").
echo "[build_and_test] smoke: test_planning (offscreen, straight road)"
smoke_log="$(QT_QPA_PLATFORM=offscreen timeout 45 "./${BUILD_DIR}/test_planning" 0 2>&1 || true)"
if grep -q "Tracking tight" <<<"${smoke_log}"; then
    echo "[build_and_test] smoke PASSED — MPC tracked the path (cte→0)"
    status=0
else
    echo "[build_and_test] smoke FAILED — MPC did not converge:"
    tail -n 20 <<<"${smoke_log}"
    status=1
fi

echo "[build_and_test] interactive demos (run via run.sh with the shared X display):"
echo "    ./${BUILD_DIR}/test_planning [0|1|2]                 # MPC: straight / circle / S-bend"
echo "    ./${BUILD_DIR}/image_preprocessing_tests <image>     # homography warp preview"
echo "    ./${BUILD_DIR}/VisionPilot -c config/vision_pilot.conf   # full app (needs ONNX weights)"

[ "${status}" -eq 0 ] && echo "[build_and_test] DONE — build + smoke OK" \
                      || echo "[build_and_test] DONE — FAILURES above"
exit "${status}"
