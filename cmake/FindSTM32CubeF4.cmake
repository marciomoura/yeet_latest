find_path(MX_Drivers_Dir
  NAMES .
  PATHS
    $ENV{STM32CUBE_PATH}/STM32CubeF4/Drivers
    $ENV{STM32CUBE_PATH}/Repository/STM32Cube_FW_F4_V1.28.3/Drivers
    C:/Users/$ENV{USERNAME}/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers
    C:/Users/$ENV{USER}/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers
  NO_DEFAULT_PATH
)
