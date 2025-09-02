# Flash/load utilities for STM32F207

# Post-build: Generate binary and hex files
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND echo "Generated ${CMAKE_PROJECT_NAME}.bin and ${CMAKE_PROJECT_NAME}.hex"
    COMMENT "Generating binary and hex files"
)

# Custom target for ST-Link
add_custom_target(flash-stlink
    COMMAND openocd -f interface/stlink.cfg -f target/stm32f2x.cfg
            -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify reset exit"
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Flashing via ST-Link"
)

# Custom target for J-Link
add_custom_target(flash-jlink
    COMMAND echo "loadfile $<TARGET_FILE:${CMAKE_PROJECT_NAME}>" |
            JLinkExe -device STM32F207ZG -if SWD -speed 4000 -autoconnect 1
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Flashing via J-Link"
)

# Custom target for serial bootloader
add_custom_target(flash-serial
    COMMAND stm32loader -p /dev/ttyUSB0 -e -w -v ${CMAKE_PROJECT_NAME}.bin
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Flashing via serial bootloader"
)

# Custom target for DFU (USB bootloader)
add_custom_target(flash-dfu
    COMMAND dfu-util -a 0 -s 0x08000000 -D ${CMAKE_PROJECT_NAME}.bin
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Flashing via DFU"
)
