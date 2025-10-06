set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# Specify the cross compiler
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

# Set C++ standard defaults for CMake 3.31+ compatibility
set(CMAKE_CXX_STANDARD_COMPUTED_DEFAULT 17)
set(CMAKE_CXX_EXTENSIONS_COMPUTED_DEFAULT OFF)
set(CMAKE_C_STANDARD_COMPUTED_DEFAULT 11)
set(CMAKE_C_EXTENSIONS_COMPUTED_DEFAULT OFF)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

# Set executable suffix to .elf for ARM embedded targets
set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")

# Don't run the linker during CMake's compiler tests
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# CPU and FPU options for STM32F407
set(CPU_FLAGS
    "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -flto=auto")

# Performance-optimized flags for Release builds -O3: Maximum speed optimization
# (aggressive inlining, loop unrolling, vectorization) -flto: Link-Time
# Optimization for cross-translation-unit optimization -ffunction-sections
# -fdata-sections: Allow linker dead code elimination -ffast-math: Aggressive
# floating-point optimization (safe for control algorithms) -funroll-loops:
# Unroll loops for speed (critical for PI controllers, observers)
# -finline-functions: Aggressive inlining for zero-overhead abstractions

# Common flags for both C and C++
set(RELEASE_FLAGS_COMMON
    "-O3 -flto=auto -ffunction-sections -fdata-sections -ffast-math -funroll-loops -finline-functions"
)

# C++-specific flags for Release: -fno-exceptions -fno-rtti: Remove C++ overhead
# (required for deterministic RT)
set(RELEASE_FLAGS_CXX "-fno-exceptions -fno-rtti")

# Debug-optimized flags -Og: Optimize for debugging (enables optimizations that
# don't interfere with debugging) -g3: Maximum debug information (includes macro
# definitions)

# Common debug flags for both C and C++
set(DEBUG_FLAGS_COMMON "-Og -g3")

# C++-specific flags for Debug: -fno-exceptions -fno-rtti: Still remove C++
# overhead even in debug
set(DEBUG_FLAGS_CXX "-fno-exceptions -fno-rtti")

# Apply flags based on build type
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(EMBEDDED_FLAGS_COMMON "${DEBUG_FLAGS_COMMON}")
    set(EMBEDDED_FLAGS_CXX "${DEBUG_FLAGS_CXX}")
    message(STATUS "ARM Toolchain: Debug build with -Og -g3 🐛")
else()
    set(EMBEDDED_FLAGS_COMMON "${RELEASE_FLAGS_COMMON}")
    set(EMBEDDED_FLAGS_CXX "${RELEASE_FLAGS_CXX}")
    message(STATUS "ARM Toolchain: Release build with -O3 -flto 🚀")
endif()

# Combine all flags - C gets common flags only, C++ gets common + C++-specific
# flags
set(CMAKE_C_FLAGS_INIT "${CPU_FLAGS} ${EMBEDDED_FLAGS_COMMON} -Wall")
set(CMAKE_CXX_FLAGS_INIT
    "${CPU_FLAGS} ${EMBEDDED_FLAGS_COMMON} ${EMBEDDED_FLAGS_CXX} -fno-use-cxa-atexit -fno-threadsafe-statics"
)
set(CMAKE_ASM_FLAGS_INIT "${CPU_FLAGS} -x assembler-with-cpp -MMD -MP")

# Linker flags Start with base CPU flags and linker script
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CPU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${CMAKE_EXE_LINKER_FLAGS_INIT} -T \"${CMAKE_SOURCE_DIR}/boards/stm32f407g-disc1/STM32F407XX_FLASH.ld\""
)

# Use nano.specs for smaller code size, but add nosys.specs for syscall stubs
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${CMAKE_EXE_LINKER_FLAGS_INIT} --specs=nano.specs --specs=nosys.specs")

# Wrap missing wide character functions with no-op stubs to avoid linker errors
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${CMAKE_EXE_LINKER_FLAGS_INIT} -Wl,--wrap=swprintf,--wrap=putwc,--wrap=getwc,--wrap=ungetwc"
)

# Add garbage collection, memory map, and memory usage reporting
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${CMAKE_EXE_LINKER_FLAGS_INIT} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections -Wl,--print-memory-usage"
)

# Search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
