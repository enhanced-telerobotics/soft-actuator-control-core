cmake_minimum_required(VERSION 3.20)
project(legacy_control_pcc_baseline LANGUAGES CXX)
set(LEGACY_ROOT "" CACHE PATH "Original Soft_gripper directory (read only)")
set(CHAI_ROOT "" CACHE PATH "Original chai3d-3.2.0 directory (read only)")
set(VIPER_INCLUDE "" CACHE PATH "Original ViperPNOGrabberMI/Inc directory (read only)")
foreach(required "${LEGACY_ROOT}/ControlPCC.cpp" "${LEGACY_ROOT}/frameTrans.cpp"
        "${CHAI_ROOT}/src/math/CMatrix3d.h" "${VIPER_INCLUDE}/ViperInterface.h")
  if(NOT EXISTS "${required}")
    message(FATAL_ERROR "Missing original dependency: ${required}")
  endif()
endforeach()
add_executable(legacy_baseline generate.cpp
  "${LEGACY_ROOT}/ControlPCC.cpp"
  "${LEGACY_ROOT}/frameTrans.cpp"
  "${LEGACY_ROOT}/TimeUtil.cpp")
# The original bundled Eigen predates C++17's removal of std::binder1st/2nd.
# C++14 matches the legacy Visual Studio project without modifying its headers.
set_target_properties(legacy_baseline PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES)
target_include_directories(legacy_baseline PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/compat" "${LEGACY_ROOT}"
  "${CHAI_ROOT}/src" "${CHAI_ROOT}/external/Eigen"
  "${CHAI_ROOT}/external/glew/include" "${VIPER_INCLUDE}")
if(MSVC)
  target_compile_options(legacy_baseline PRIVATE /fp:precise /W3)
  target_compile_definitions(legacy_baseline PRIVATE NOMINMAX _CRT_SECURE_NO_WARNINGS)
endif()
