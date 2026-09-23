cmake_minimum_required(VERSION 3.16)
project(fiducial-detector-service VERSION 0.1.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Portability contract: no CUDA, no TensorRT, no platform-conditional code.
#
# Every dependency below is apt-installable on both JetPack 6 (Ubuntu 22.04
# aarch64) and Raspberry Pi OS Bookworm. Adding a GPU dependency would
# invalidate the Raspberry Pi target.
#
# The only platform-varying value in the tree is the GStreamer pipeline string,
# which comes from YAML config.
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# ---- Dependencies ---------------------------------------------------------

find_package(Protobuf REQUIRED)
# ArUco moved between modules in OpenCV 4.7: before it, it is the separate
# contrib module opencv_aruco; from 4.7 it is part of core objdetect. The
# version is only known after a first find_package, so discover it and then
# request the correct module set. Omitting opencv_aruco on an older OpenCV
# still compiles and fails at link with undefined cv::aruco symbols.
find_package(OpenCV REQUIRED)
if(OpenCV_VERSION VERSION_LESS 4.7.0)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)
else()
    find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d objdetect)
endif()
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

pkg_check_modules(GST REQUIRED IMPORTED_TARGET
    gstreamer-1.0
    gstreamer-app-1.0
    gstreamer-video-1.0
)

message(STATUS "OpenCV   ${OpenCV_VERSION}")
message(STATUS "GStreamer ${GST_gstreamer-1.0_VERSION}")

# OpenCV moved ArUco from contrib (opencv2/aruco.hpp, Ptr-based API) into core
# objdetect (cv::aruco::ArucoDetector) in 4.7.0. Raspberry Pi OS Bookworm ships
# 4.6.0; JetPack 6 ships 4.8+. Both are supported via include/aruco_compat.h.
if(OpenCV_VERSION VERSION_LESS 4.7.0)
    message(STATUS "OpenCV < 4.7: using legacy cv::aruco contrib API (opencv_aruco)")
else()
    message(STATUS "OpenCV >= 4.7: using cv::aruco::ArucoDetector (objdetect)")
endif()

# ---- Protobuf codegen -----------------------------------------------------
#
# The contract has exactly one canonical copy, in nexus-protos. Point
# NEXUS_PROTOS_DIR at a nexus-protos checkout to override the protos/ submodule,
# for example when testing an unmerged contract change.

set(NEXUS_PROTOS_DIR "" CACHE PATH
    "Path to a nexus-protos checkout containing fiducial.proto. Empty = use the in-repo bootstrap copy.")

# protos/ is nexus-protos itself (submodule or checkout) when it carries the rest
# of that repo's files. Checking for a sibling .proto rather than for git metadata
# means this still works from a release tarball, where .git is absent.
set(_protos_is_nexus_protos FALSE)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/protos/jetson_vision.proto"
   OR EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/protos/common.proto")
    set(_protos_is_nexus_protos TRUE)
endif()

if(NEXUS_PROTOS_DIR AND EXISTS "${NEXUS_PROTOS_DIR}/fiducial.proto")
    set(PROTO_DIR  "${NEXUS_PROTOS_DIR}")
    message(STATUS "fiducial.proto: ${PROTO_DIR}/fiducial.proto (canonical, nexus-protos)")
elseif(_protos_is_nexus_protos AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/protos/fiducial.proto")
    set(PROTO_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/protos")
    message(STATUS "fiducial.proto: protos/ submodule (canonical, nexus-protos)")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/protos/fiducial.proto")
    set(PROTO_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/protos")
    message(STATUS "fiducial.proto: in-repo BOOTSTRAP copy")
    message(STATUS "  Upstream it to nexus-protos and take that as the protos/ submodule "
                   "before release.")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.gitmodules")
    # An uninitialised submodule leaves protos/ empty. Without this branch the
    # failure is "file not found", which sends people looking for a missing file
    # rather than a missing `git submodule update`.
    message(FATAL_ERROR
        "protos/ is empty: the nexus-protos submodule is not checked out.\n"
        "  git submodule update --init --recursive\n"
        "(or clone with --recursive)")
else()
    message(FATAL_ERROR
        "fiducial.proto not found. Expected it in protos/ (the nexus-protos "
        "submodule) or at -DNEXUS_PROTOS_DIR=<path to a nexus-protos checkout>.")
endif()

set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${PROTO_GEN_DIR}")

add_custom_command(
    OUTPUT "${PROTO_GEN_DIR}/fiducial.pb.cc" "${PROTO_GEN_DIR}/fiducial.pb.h"
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --cpp_out=${PROTO_GEN_DIR}
         --proto_path=${PROTO_DIR}
         ${PROTO_DIR}/fiducial.proto
    DEPENDS "${PROTO_DIR}/fiducial.proto"
    COMMENT "protoc fiducial.proto"
    VERBATIM
)

add_library(fiducial_proto STATIC "${PROTO_GEN_DIR}/fiducial.pb.cc")
target_include_directories(fiducial_proto PUBLIC "${PROTO_GEN_DIR}" ${Protobuf_INCLUDE_DIRS})
target_link_libraries(fiducial_proto PUBLIC protobuf::libprotobuf)
target_compile_options(fiducial_proto PRIVATE -w)   # generated code

# ---- Core library ---------------------------------------------------------
#
# Split from the executable so tests can link against it.

add_library(fiducial_core STATIC
    src/app_config.cpp
    src/clock_domain.cpp
    src/gst_capture.cpp
    src/detector.cpp
    src/udp_publisher.cpp
    src/stream_worker.cpp
)

target_include_directories(fiducial_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(fiducial_core PUBLIC
    fiducial_proto
    ${OpenCV_LIBS}
    PkgConfig::GST
    yaml-cpp
    Threads::Threads
)

# ---- Executable -----------------------------------------------------------

add_executable(fiducial-detector-service src/main.cpp)
target_link_libraries(fiducial-detector-service PRIVATE fiducial_core)
target_compile_definitions(fiducial-detector-service PRIVATE FD_VERSION="${PROJECT_VERSION}")

# ---- Install --------------------------------------------------------------

install(TARGETS fiducial-detector-service RUNTIME DESTINATION bin)
install(FILES systemd/fiducial-detector-service.service
        DESTINATION lib/systemd/system)
install(DIRECTORY config/ DESTINATION share/fiducial-detector-service/config
        FILES_MATCHING PATTERN "*.yaml")
