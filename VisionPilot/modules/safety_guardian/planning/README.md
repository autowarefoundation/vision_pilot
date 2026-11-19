# Planning module

## Dependencies

The planning module requires:

- IPOPT built with at least one supported linear-solver backend.
- CppAD, including `libcppad_lib.so`.
- Eigen.
- OpenCV.

The planner does not hardcode a specific IPOPT linear solver. The selected
IPOPT installation must provide the backend it uses, such as HSL, MUMPS, or
Pardiso.

Verify the IPOPT linear-solver backend before configuring. IPOPT reports the
supported `linear_solver` values when an unsupported option is supplied.

```bash
pkg-config --libs ipopt
```

## Configuration

For a headless board:

```bash
cmake .. \
  -DBUILD_TESTING=ON \
  -DVISIONPILOT_ENABLE_GUI=OFF
```

For a desktop with OpenCV GUI support:

```bash
cmake .. \
  -DBUILD_TESTING=ON \
  -DVISIONPILOT_ENABLE_GUI=ON
```

CMake requires IPOPT and CppAD during configuration. The planner reports a
clear diagnostic if IPOPT returns no usable solution at runtime.
