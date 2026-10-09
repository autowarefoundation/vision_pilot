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

# The ONNX Runtime shared library ships next to the extension, so the wheel
# needs no LD_LIBRARY_PATH and no system-wide ONNX Runtime.
set_target_properties(_core PROPERTIES INSTALL_RPATH "$ORIGIN")

install(TARGETS _core LIBRARY DESTINATION vision_pilot COMPONENT python)

file(GLOB _ort_libs "${ONNXRUNTIME_ROOT}/lib/libonnxruntime*.so*")
foreach(_lib IN LISTS _ort_libs)
    get_filename_component(_name "${_lib}" NAME)
    # Ship each library once, under the name the dynamic linker looks up:
    # libonnxruntime.so.1 (its SONAME) and the unversioned provider plugins.
    if(_name MATCHES "^libonnxruntime\\.so\\.[0-9]+$" OR _name MATCHES "^libonnxruntime_providers_[a-z_]+\\.so$")
        file(REAL_PATH "${_lib}" _real)
        install(FILES "${_real}" DESTINATION vision_pilot RENAME "${_name}" COMPONENT python)
    endif()
endforeach()

# Typed stubs for the extension, generated from the bindings themselves when
# the wheel is assembled (the install tree is importable at that point).
nanobind_add_stub(_core_stub
        INSTALL_TIME
        COMPONENT python
        MODULE vision_pilot._core
        OUTPUT vision_pilot/_core.pyi
        PYTHON_PATH "."
)
