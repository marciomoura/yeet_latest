include(FetchContent)
message(STATUS "📦 Fetching STM32 HAL, CMSIS, and USB Host libraries...")

# Fetch STM32F4 HAL Driver
if(DEFINED ENV{STM32_HAL_DIR})
    message(STATUS "✅ Using STM32 HAL from environment: $ENV{STM32_HAL_DIR}")
    set(STM32_HAL_DIR $ENV{STM32_HAL_DIR})
else()
    fetchcontent_declare(
        stm32f4xx_hal_driver
        GIT_REPOSITORY
            https://github.com/STMicroelectronics/stm32f4xx_hal_driver.git
        GIT_TAG v1.8.2 # Pin to specific version for reproducibility
        GIT_SHALLOW TRUE # Only download latest commit (faster)
    )
    fetchcontent_makeavailable(stm32f4xx_hal_driver)
    set(STM32_HAL_DIR ${stm32f4xx_hal_driver_SOURCE_DIR})
endif()


# Fetch CMSIS Device files for STM32F4
if(DEFINED ENV{CMSIS_DEVICE_DIR})
    message(STATUS "✅ Using CMSIS Device from environment: $ENV{CMSIS_DEVICE_DIR}")
    set(CMSIS_DEVICE_DIR $ENV{CMSIS_DEVICE_DIR})
else()
    fetchcontent_declare(
        cmsis_device_f4
        GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis_device_f4.git
        GIT_TAG v2.6.9 # Pin to specific version
        GIT_SHALLOW TRUE)
    fetchcontent_makeavailable(cmsis_device_f4)
    set(CMSIS_DEVICE_DIR ${cmsis_device_f4_SOURCE_DIR})
endif()


# Fetch CMSIS Core
if(DEFINED ENV{CMSIS_CORE_DIR})
    message(STATUS "✅ Using CMSIS Core from environment: $ENV{CMSIS_CORE_DIR}")
    set(CMSIS_CORE_DIR $ENV{CMSIS_CORE_DIR})
else()
    fetchcontent_declare(
        cmsis_core
        GIT_REPOSITORY https://github.com/ARM-software/CMSIS_5.git
        GIT_TAG 5.9.0
        GIT_SHALLOW TRUE)
    fetchcontent_makeavailable(cmsis_core)
    set(CMSIS_CORE_DIR ${cmsis_core_SOURCE_DIR}/CMSIS/Core)
endif()


# Fetch STM32 USB Host Library (Middleware)
if(DEFINED ENV{STM32_USB_HOST_DIR})
    message(STATUS "✅ Using STM32 USB Host from environment: $ENV{STM32_USB_HOST_DIR}")
    set(STM32_USB_HOST_DIR $ENV{STM32_USB_HOST_DIR})
else()
    fetchcontent_declare(
        stm32_mw_usb_host
        GIT_REPOSITORY https://github.com/STMicroelectronics/stm32_mw_usb_host.git
        GIT_TAG v3.5.1 # Pin to specific version
        GIT_SHALLOW TRUE)
    fetchcontent_makeavailable(stm32_mw_usb_host)
    set(STM32_USB_HOST_DIR ${stm32_mw_usb_host_SOURCE_DIR})
endif()

message(STATUS "✅ STM32 HAL located at: ${STM32_HAL_DIR}")
message(STATUS "✅ CMSIS Device located at: ${CMSIS_DEVICE_DIR}")
message(STATUS "✅ CMSIS Core located at: ${CMSIS_CORE_DIR}")
message(STATUS "✅ STM32 USB Host located at: ${STM32_USB_HOST_DIR}")
