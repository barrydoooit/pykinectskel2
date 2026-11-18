cmake_minimum_required(VERSION 3.16)
project(kinectv2_sdk VERSION 1.0.0 LANGUAGES C CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(KV2_BUILD_VISUALIZER "Build the 30 FPS SDL2 + OpenCV desktop visualizer" ON)
option(KV2_BUILD_EXAMPLES   "Build C++ SDK quickstart example" ON)

if(NOT DEFINED FREENECT2_ROOT)
    if(EXISTS "$ENV{HOME}/.local/share/kinectv2-runtime/freenect2/lib")
        set(FREENECT2_ROOT "$ENV{HOME}/.local/share/kinectv2-runtime/freenect2")
    else()
        set(FREENECT2_ROOT "/usr/local")
    endif()
endif()

find_library(FREENECT2_LIB
    NAMES freenect2
    PATHS "${FREENECT2_ROOT}/lib" "${FREENECT2_ROOT}/lib64"
)
if(NOT FREENECT2_LIB)
    message(FATAL_ERROR "libfreenect2 not found. Pass -DFREENECT2_ROOT=/path/to/freenect2 or run scripts/build_libfreenect2.sh.")
endif()

find_path(FREENECT2_INCLUDE_DIR
    NAMES libfreenect2/libfreenect2.hpp
    PATHS "${FREENECT2_ROOT}/include"
)
if(NOT FREENECT2_INCLUDE_DIR)
    message(FATAL_ERROR "libfreenect2 headers not found under ${FREENECT2_ROOT}/include.")
endif()

find_package(Threads REQUIRED)

set(SDK_RPATH "${FREENECT2_ROOT}/lib;/usr/local/cuda-12.6/lib64;/usr/local/cuda/lib64;$ORIGIN")

add_library(kinectv2_sdk SHARED
    src/kinectv2_sdk.cpp
)
target_include_directories(kinectv2_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        "${FREENECT2_INCLUDE_DIR}"
)
target_compile_options(kinectv2_sdk PRIVATE -O3 -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(kinectv2_sdk PRIVATE "${FREENECT2_LIB}" Threads::Threads ${CMAKE_DL_LIBS})
set_target_properties(kinectv2_sdk PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    BUILD_RPATH "${SDK_RPATH}"
    INSTALL_RPATH "${CMAKE_INSTALL_FULL_LIBDIR};${SDK_RPATH}"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/lib"
)

install(TARGETS kinectv2_sdk
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES include/kinectv2_sdk.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(KV2_BUILD_VISUALIZER)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(SDL2 REQUIRED sdl2)

    add_executable(kinectv2_visualizer
        src/tools/kinectv2_visualizer.cpp
    )
    target_include_directories(kinectv2_visualizer PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        ${OpenCV_INCLUDE_DIRS}
        ${SDL2_INCLUDE_DIRS}
    )
    target_compile_options(kinectv2_visualizer PRIVATE -O3 -Wall -Wextra)
    target_link_libraries(kinectv2_visualizer PRIVATE
        kinectv2_sdk
        ${OpenCV_LIBS}
        ${SDL2_LIBRARIES}
        Threads::Threads
    )
    set_target_properties(kinectv2_visualizer PROPERTIES
        BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib;${SDK_RPATH}"
        INSTALL_RPATH "${CMAKE_INSTALL_FULL_LIBDIR};${SDK_RPATH}"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin"
    )
    install(TARGETS kinectv2_visualizer RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

if(KV2_BUILD_EXAMPLES)
    add_executable(cpp_sdk_demo
        examples/cpp_sdk_demo.cpp
    )
    target_include_directories(cpp_sdk_demo PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_compile_options(cpp_sdk_demo PRIVATE -O3 -Wall -Wextra)
    target_link_libraries(cpp_sdk_demo PRIVATE kinectv2_sdk Threads::Threads)
    set_target_properties(cpp_sdk_demo PROPERTIES
        BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib;${SDK_RPATH}"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin"
    )
endif()
