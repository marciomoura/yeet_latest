message(
    STATUS "-- build-config: 🟢 searching eigen package for ${CMAKE_SYSTEM_NAME}"
)

include(FetchContent)

fetchcontent_declare(
    Eigen3
    SYSTEM
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR cmake) # no CMakeLists.txt in cmake, so this turns off
                         # configure

fetchcontent_makeavailable(Eigen3)

if(NOT TARGET Eigen3::Eigen)
    message(STATUS "-- build-config: 🟢 adding Eigen3::Eigen interface library")
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                   ${eigen3_SOURCE_DIR})
endif()
