message(
    STATUS
        "-- build-config: 🟢 searching qpOASES package for ${CMAKE_SYSTEM_NAME}")

include(FetchContent)

fetchcontent_declare(
    qpOASES
    SYSTEM
    GIT_REPOSITORY https://github.com/coin-or/qpOASES
    GIT_TAG releases/3.2.2
    GIT_SHALLOW TRUE)

fetchcontent_makeavailable(qpOASES)

# Add include directories and link libraries
include_directories(${qpOASES_SOURCE_DIR}/include)
