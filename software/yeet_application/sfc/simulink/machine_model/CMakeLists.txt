project(yeet_generated_LCITopology6x6Minimal CXX C)

add_library(${PROJECT_NAME} STATIC)
add_library(yeet::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_sources(
    ${PROJECT_NAME}
    PRIVATE LCITopology6x6Minimal.cpp
            LCITopology6x6Minimal_data.cpp
            rt_look.cpp
            rt_look1d.cpp
            rt_nonfinite.cpp
            rtGetInf.cpp
            rtGetNaN.cpp
    PUBLIC FILE_SET
           HEADERS
           BASE_DIRS
           ${CMAKE_CURRENT_SOURCE_DIR}
           FILES
           LCITopology6x6Minimal.h
           LCITopology6x6Minimal_types.h
           LCITopology6x6Minimal_private.h
           rtwtypes.h
           tmwtypes.h
           rtw_continuous.h
           rtw_solver.h
           multiword_types.h
           rt_defines.h
           rt_nonfinite.h
           rtGetInf.h
           rtGetNaN.h
           rt_look.h
           rt_look1d.h)

target_compile_options(
    ${PROJECT_NAME}
    PRIVATE $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-O2
            -ffast-math>)

# Workaround for Windows path length issues: Remove CMAKE_DEPFILE_FLAGS_CXX to
# prevent dependency file generation
set(CMAKE_DEPFILE_FLAGS_CXX "")
