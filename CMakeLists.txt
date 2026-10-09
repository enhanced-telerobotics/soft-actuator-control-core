cmake_minimum_required(VERSION 3.16)
project(soft_actuator_core VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

add_library(soft_actuator_core STATIC
    src/controller.cpp src/kernel.cpp src/model.cpp src/output_policy.cpp)
add_library(soft_actuator::core ALIAS soft_actuator_core)
set_target_properties(soft_actuator_core PROPERTIES EXPORT_NAME core)
target_compile_features(soft_actuator_core PUBLIC cxx_std_17)
target_include_directories(soft_actuator_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
if(MSVC)
    target_compile_options(soft_actuator_core PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(soft_actuator_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

option(SOFT_ACTUATOR_BUILD_EXAMPLES "Build offline examples" ON)
if(SOFT_ACTUATOR_BUILD_EXAMPLES)
    add_executable(soft_actuator_offline examples/offline.cpp)
    target_link_libraries(soft_actuator_offline PRIVATE soft_actuator::core)
endif()

if(BUILD_TESTING)
    add_executable(soft_actuator_tests tests/controller_tests.cpp)
    target_link_libraries(soft_actuator_tests PRIVATE soft_actuator::core)
    add_test(NAME controller_contract COMMAND soft_actuator_tests
        "${CMAKE_CURRENT_SOURCE_DIR}/profiles/actuatorprofile_4.txt")
    add_executable(output_policy_tests tests/output_policy_tests.cpp)
    target_link_libraries(output_policy_tests PRIVATE soft_actuator::core)
    add_test(NAME output_policy_and_model COMMAND output_policy_tests
        "${CMAKE_CURRENT_SOURCE_DIR}/profiles/actuatorprofile_4.txt")
    add_executable(legacy_replay tests/legacy_replay.cpp)
    target_include_directories(legacy_replay PRIVATE src)
    target_link_libraries(legacy_replay PRIVATE soft_actuator::core)
    add_test(NAME legacy_numerical_replay COMMAND legacy_replay
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/legacy_profile4.csv"
        "${CMAKE_CURRENT_SOURCE_DIR}/profiles/actuatorprofile_4.txt")
    add_test(NAME installed_consumer COMMAND ${CMAKE_COMMAND}
        "-DCORE_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DCORE_BUILD=${CMAKE_CURRENT_BINARY_DIR}"
        "-DGENERATOR=${CMAKE_GENERATOR}"
        "-DPLATFORM=${CMAKE_GENERATOR_PLATFORM}"
        "-DCONFIG=$<CONFIG>"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/install_consumer.cmake")
    if(SOFT_ACTUATOR_BUILD_EXAMPLES)
        foreach(mode open closed)
            add_test(NAME offline_${mode} COMMAND soft_actuator_offline
                "${CMAKE_CURRENT_SOURCE_DIR}/profiles/actuatorprofile_4.txt" ${mode})
        endforeach()
    endif()
endif()

install(TARGETS soft_actuator_core EXPORT soft_actuator_coreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY profiles/ DESTINATION ${CMAKE_INSTALL_DATADIR}/soft_actuator_core/profiles)
install(FILES README.md THIRD_PARTY_NOTICES.md
    DESTINATION ${CMAKE_INSTALL_DATADIR}/soft_actuator_core)
install(EXPORT soft_actuator_coreTargets NAMESPACE soft_actuator::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/soft_actuator_core)
configure_package_config_file(cmake/soft_actuator_coreConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/soft_actuator_coreConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/soft_actuator_core)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/soft_actuator_coreConfigVersion.cmake"
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/soft_actuator_coreConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/soft_actuator_coreConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/soft_actuator_core)
