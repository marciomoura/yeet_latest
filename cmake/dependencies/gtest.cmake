# Description: Fetch the google-test framework (https://github.com/google/googletest)

message(STATUS "-- eidgenosse build-config: 🟢 searching googletest package for ${CMAKE_SYSTEM_NAME}")

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  EXCLUDE_FROM_ALL
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

if(NOT TARGET gtest::gtest)
  add_library(gtest::gtest ALIAS gtest)
  add_library(gtest::gmock ALIAS gmock)
  add_library(gtest::gtest_main ALIAS gtest_main)
  add_library(gtest::gmock_main ALIAS gmock_main)
endif()
