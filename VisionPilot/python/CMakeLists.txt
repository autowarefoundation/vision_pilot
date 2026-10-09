# The `vision_pilot._core` extension, built by scikit-build-core from
# ../pyproject.toml. See python/README.md.

find_package(Python 3.10
        REQUIRED COMPONENTS Interpreter Development.Module
        OPTIONAL_COMPONENTS Development.SABIModule)

# nanobind ships its CMake package inside the Python wheel (a build requirement).
execute_process(
        COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE
        OUTPUT_VARIABLE nanobind_ROOT
        COMMAND_ERROR_IS_FATAL ANY)
find_package(nanobind CONFIG REQUIRED)

# STABLE_ABI builds an abi3 extension on Python >= 3.12 (one wheel for every
# later CPython) and is ignored on older interpreters. NB_STATIC links
# libnanobind into the extension; LTO keeps the binding layer small.
nanobind_add_module(_core
        STABLE_ABI
        NB_STATIC
        LTO
        bindings.cpp
)
target_link_libraries(_core PRIVATE pilot)

# Runtime libraries, all found relative to the extension in site-packages:
#   $ORIGIN                    ONNX Runtime, shipped next to the extension
#   $ORIGIN/../casadi          Ipopt and its solvers (the casadi wheel)
#   $ORIGIN/../cmeel.prefix/lib  libcppad_lib (the cmeel-cppad wheel)
set_target_properties(_core PROPERTIES
        INSTALL_RPATH "$ORIGIN;$ORIGIN/../casadi;$ORIGIN/../cmeel.prefix/lib")

install(TARGETS _core LIBRARY DESTINATION vision_pilot COMPONENT python)

set(_core_dir "${CMAKE_CURRENT_BINARY_DIR}/stub_root/vision_pilot")
set_target_properties(_core PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${_core_dir}")

# ONNX Runtime ships under a hashed name, as auditwheel does for the libraries it
# grafts: a libonnxruntime.so.1 next to the extension would lose to any other
# one on LD_LIBRARY_PATH (searched before RUNPATH), e.g. an older system ORT in
# /usr/local/lib, and fail with "version `VERS_1.26.0' not found".
execute_process(
        COMMAND "${Python_EXECUTABLE}" -c "import sysconfig; print(sysconfig.get_path('scripts'))"
        OUTPUT_VARIABLE _py_scripts
        OUTPUT_STRIP_TRAILING_WHITESPACE)
find_program(PATCHELF_EXECUTABLE patchelf HINTS "${_py_scripts}" REQUIRED)  # the `patchelf` build requirement

file(GLOB _ort_libs "${ONNXRUNTIME_ROOT}/lib/libonnxruntime*.so*")
foreach(_lib IN LISTS _ort_libs)
    get_filename_component(_name "${_lib}" NAME)
    file(REAL_PATH "${_lib}" _real)
    if(_name MATCHES "^libonnxruntime\\.so\\.[0-9]+$")
        # libonnxruntime.so.1.26.0 -> libonnxruntime-<sha256[:8]>.so.1.26.0
        file(SHA256 "${_real}" _hash)
        string(SUBSTRING "${_hash}" 0 8 _hash)
        get_filename_component(_real_name "${_real}" NAME)
        string(REGEX REPLACE "^libonnxruntime" "libonnxruntime-${_hash}" _ort_vendored "${_real_name}")
        set(_ort_soname "${_name}")
        set(_ort_real "${_real}")
    elseif(_name MATCHES "^libonnxruntime_providers_[a-z_]+\\.so$")
        # Provider plugins are loaded by ORT from its own directory, by full path.
        install(FILES "${_real}" DESTINATION vision_pilot COMPONENT python)
    endif()
endforeach()
if(NOT _ort_vendored)
    message(FATAL_ERROR "[python] No libonnxruntime.so.<N> in ${ONNXRUNTIME_ROOT}/lib")
endif()

# The renamed copy sits next to the build-tree extension too, for the stubs below.
add_custom_command(
        OUTPUT "${_core_dir}/${_ort_vendored}"
        COMMAND ${CMAKE_COMMAND} -E copy "${_ort_real}" "${_core_dir}/${_ort_vendored}"
        COMMAND "${PATCHELF_EXECUTABLE}" --set-soname "${_ort_vendored}" "${_core_dir}/${_ort_vendored}"
        DEPENDS "${_ort_real}"
        COMMENT "Vendoring ONNX Runtime as ${_ort_vendored}")
add_custom_target(_core_onnxruntime DEPENDS "${_core_dir}/${_ort_vendored}")
add_dependencies(_core _core_onnxruntime)
add_custom_command(TARGET _core POST_BUILD
        COMMAND "${PATCHELF_EXECUTABLE}" --replace-needed "${_ort_soname}" "${_ort_vendored}" "$<TARGET_FILE:_core>")
set_property(TARGET _core APPEND PROPERTY BUILD_RPATH "$ORIGIN")
install(FILES "${_core_dir}/${_ort_vendored}" DESTINATION vision_pilot COMPONENT python)

# Typed stubs for the extension, generated from the bindings themselves. This
# imports the extension from the build tree, whose build RPATH reaches the
# Ipopt / CppAD wheels of the build environment (the install tree's relative
# RPATH only resolves once installed into site-packages).
nanobind_add_stub(_core_stub
        MODULE vision_pilot._core
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/stub_root/vision_pilot/_core.pyi"
        PYTHON_PATH "${CMAKE_CURRENT_BINARY_DIR}/stub_root"
        DEPENDS _core
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/stub_root/vision_pilot/_core.pyi"
        DESTINATION vision_pilot COMPONENT python)
