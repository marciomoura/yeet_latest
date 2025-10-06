include(FetchContent)

# Fetch STM32F4 HAL Driver
fetchcontent_declare(
    stm32f4xx_hal_driver
    GIT_REPOSITORY
        https://github.com/STMicroelectronics/stm32f4xx_hal_driver.git
    GIT_TAG v1.8.2 # Pin to specific version for reproducibility
    GIT_SHALLOW TRUE # Only download latest commit (faster)
)

# Fetch CMSIS Device files for STM32F4
fetchcontent_declare(
    cmsis_device_f4
    GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis_device_f4.git
    GIT_TAG v2.6.9 # Pin to specific version
    GIT_SHALLOW TRUE)

# Fetch CMSIS Core
fetchcontent_declare(
    cmsis_core
    GIT_REPOSITORY https://github.com/ARM-software/CMSIS_5.git
    GIT_TAG 5.9.0
    GIT_SHALLOW TRUE)

# Fetch STM32 USB Host Library (Middleware)
fetchcontent_declare(
    stm32_mw_usb_host
    GIT_REPOSITORY https://github.com/STMicroelectronics/stm32_mw_usb_host.git
    GIT_TAG v3.5.1 # Pin to specific version
    GIT_SHALLOW TRUE)

message(STATUS "📦 Fetching STM32 HAL, CMSIS, and USB Host libraries...")

# Make available without adding to build
fetchcontent_makeavailable(stm32f4xx_hal_driver)
fetchcontent_makeavailable(cmsis_device_f4)
fetchcontent_makeavailable(cmsis_core)
fetchcontent_makeavailable(stm32_mw_usb_host)

# Set convenient variables for board CMakeLists
set(STM32_HAL_DIR ${stm32f4xx_hal_driver_SOURCE_DIR})
set(CMSIS_DEVICE_DIR ${cmsis_device_f4_SOURCE_DIR})
set(CMSIS_CORE_DIR ${cmsis_core_SOURCE_DIR}/CMSIS/Core)
set(STM32_USB_HOST_DIR ${stm32_mw_usb_host_SOURCE_DIR})

message(STATUS "✅ STM32 HAL located at: ${STM32_HAL_DIR}")
message(STATUS "✅ CMSIS Device located at: ${CMSIS_DEVICE_DIR}")
message(STATUS "✅ CMSIS Core located at: ${CMSIS_CORE_DIR}")
message(STATUS "✅ STM32 USB Host located at: ${STM32_USB_HOST_DIR}")
