set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# Use GCC_TOOLCHAIN_ROOT environment variable if set, otherwise fallback to PATH
if(DEFINED ENV{GCC_TOOLCHAIN_ROOT})
    # Normalize path for CMake (converts backslashes to forward slashes)
    file(TO_CMAKE_PATH "$ENV{GCC_TOOLCHAIN_ROOT}" TOOLCHAIN_ROOT_PATH)

    # Check if GCC_TOOLCHAIN_ROOT points to bin directory or root directory
    if(TOOLCHAIN_ROOT_PATH MATCHES ".*bin$")
        # Already points to bin directory, use as-is
        set(TOOLCHAIN_PREFIX "${TOOLCHAIN_ROOT_PATH}/arm-none-eabi-")
        message(
            STATUS
                "Using ARM toolchain from GCC_TOOLCHAIN_ROOT (bin): ${TOOLCHAIN_ROOT_PATH}"
        )
    else()
        # Points to root directory, append /bin
        set(TOOLCHAIN_PREFIX "${TOOLCHAIN_ROOT_PATH}/bin/arm-none-eabi-")
        message(
            STATUS
                "Using ARM toolchain from GCC_TOOLCHAIN_ROOT: ${TOOLCHAIN_ROOT_PATH}"
        )
    endif()

    # Add .exe extension on Windows
    if(CMAKE_HOST_WIN32)
        set(TOOLCHAIN_SUFFIX ".exe")
    else()
        set(TOOLCHAIN_SUFFIX "")
    endif()
else()
    set(TOOLCHAIN_PREFIX "arm-none-eabi-")
    set(TOOLCHAIN_SUFFIX "")
    message(STATUS "GCC_TOOLCHAIN_ROOT not set, using ARM toolchain from PATH")
endif()

# Specify the cross compiler
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_SUFFIX})
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX}
)# Set C++ standard defaults for CMake 3.31+ compatibility
set(CMAKE_CXX_STANDARD_COMPUTED_DEFAULT 17)
set(CMAKE_CXX_EXTENSIONS_COMPUTED_DEFAULT OFF)
set(CMAKE_C_STANDARD_COMPUTED_DEFAULT 11)
set(CMAKE_C_EXTENSIONS_COMPUTED_DEFAULT OFF)

# Use gcc-ar and gcc-ranlib wrappers for LTO support in static libraries These
# wrappers automatically load the LTO plugin First, check if they exist
find_program(GCC_AR_TOOL ${TOOLCHAIN_PREFIX}gcc-ar${TOOLCHAIN_SUFFIX})
find_program(GCC_RANLIB_TOOL ${TOOLCHAIN_PREFIX}gcc-ranlib${TOOLCHAIN_SUFFIX})
find_program(GCC_NM_TOOL ${TOOLCHAIN_PREFIX}gcc-nm${TOOLCHAIN_SUFFIX})

if(GCC_AR_TOOL AND GCC_RANLIB_TOOL)
    set(CMAKE_AR ${TOOLCHAIN_PREFIX}gcc-ar${TOOLCHAIN_SUFFIX})
    set(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}gcc-ranlib${TOOLCHAIN_SUFFIX})
    set(CMAKE_NM ${TOOLCHAIN_PREFIX}gcc-nm${TOOLCHAIN_SUFFIX})
    message(
        STATUS "Using LTO-aware archiver tools (gcc-ar, gcc-ranlib, gcc-nm)")
else()
    set(CMAKE_AR ${TOOLCHAIN_PREFIX}ar${TOOLCHAIN_SUFFIX})
    set(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}ranlib${TOOLCHAIN_SUFFIX})
    set(CMAKE_NM ${TOOLCHAIN_PREFIX}nm${TOOLCHAIN_SUFFIX})
    message(
        WARNING
            "LTO-aware tools (gcc-ar, gcc-ranlib) not found. Using standard ar/ranlib. LTO may not work correctly with static libraries!"
    )
endif()

# Set other binutils tools
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy${TOOLCHAIN_SUFFIX})
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}objdump${TOOLCHAIN_SUFFIX})
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size${TOOLCHAIN_SUFFIX})

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
