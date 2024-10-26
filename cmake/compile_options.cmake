project(compile_options CXX)

add_library(${PROJECT_NAME} INTERFACE)
add_library(mpc::compile_options ALIAS ${PROJECT_NAME})

target_compile_options(
  ${PROJECT_NAME} INTERFACE -fno-exceptions -fno-rtti -Wall -Werror
                            # -O1
)
