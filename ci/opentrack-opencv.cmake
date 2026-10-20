# Replacement for opentrack's cmake/opentrack-opencv.cmake (copied over it by CI).
#
# The original registers PDB installs for the opencv_* imported targets
# unconditionally on MSVC. Without OpenCV those targets don't exist and the
# CMake *generate* step fails with "No target opencv_core", even though no
# module that needs OpenCV is being built. This copy only differs by guarding
# that loop with OpenCV_FOUND. Keep it in sync with the pinned opentrack tag.
include_guard(GLOBAL)

include(opentrack-boilerplate)
find_package(OpenCV QUIET)

if(OpenCV_FOUND)
    foreach(i opencv_calib3d opencv_core opencv_highgui opencv_imgcodecs
              opencv_imgproc opencv_video opencv_videoio)
        otr_install_pdb_for_imported_target(${i})
    endforeach()
endif()
