project(yeet_generated_sfc_6p_6p_topology CXX)

# Create the static library
add_library(${PROJECT_NAME} STATIC)
add_library(yeet::yeet_generated_sfc_6p_6p_topology ALIAS ${PROJECT_NAME})

# Add library sources
target_sources(${PROJECT_NAME} PRIVATE 
    sfc_6p_6p_topology.cpp
)

# Add public headers
target_sources(${PROJECT_NAME} PUBLIC
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
    FILES sfc_6p_6p_topology.h
)

# Set include directories
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)

# Set C++ standard and features
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Set compiler-specific options for real-time code
if(MSVC)
    # Windows-specific options for real-time performance
    target_compile_options(${PROJECT_NAME} PRIVATE /O2)
else()
    # GCC/Clang options for real-time performance
    target_compile_options(${PROJECT_NAME} PRIVATE -O2 -ffast-math)
endif()

# Link dependencies (if any)
# Note: This auto-generated code appears to be self-contained
# Add dependencies here if needed
