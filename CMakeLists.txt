cmake_minimum_required(VERSION 3.1)
project(panorama_vslam_examples LANGUAGES CXX C)

# ----- Find dependencies -----

find_package(stella_vslam REQUIRED)

# filesystem
set(filesystem_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/filesystem/include)
# popl
set(popl_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/popl/include)

# ----- Show dialog -----

find_package(pangolin_viewer QUIET)
if(pangolin_viewer_FOUND)
    message(STATUS "Viewer for examples: PangolinViewer")
endif()


# ----- Build example executables -----

set(EXECUTABLE_TARGETS "")


add_executable(run_video src/run_video.cc)
list(APPEND EXECUTABLE_TARGETS run_video)
add_executable(run_img src/run_img.cc src/util/image_util.cc)
list(APPEND EXECUTABLE_TARGETS run_img)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    # Set output directory for executables
    set_target_properties(${EXECUTABLE_TARGET} PROPERTIES
                          RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_BINARY_DIR}")

    # PangolinViewer is used on a priority basis
    if(pangolin_viewer_FOUND)
        # Set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE HAVE_PANGOLIN_VIEWER)
        # Link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE pangolin_viewer::pangolin_viewer)
    endif()
    
    # Link stella_vslam
    target_link_libraries(${EXECUTABLE_TARGET}
                          PRIVATE
                          stella_vslam::stella_vslam
                          opencv_imgcodecs
                          opencv_videoio)

    # include 3rd party library headers
    target_include_directories(${EXECUTABLE_TARGET}
                               PRIVATE
                               $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/popl/include>
                               $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/filesystem/include>
                               $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/spdlog/include>)
endforeach()