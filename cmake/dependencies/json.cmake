# Description: fetch the json package from nlohmann/json

message(STATUS "-- eidgenosse build-config: 🟢 searching json package for ${CMAKE_SYSTEM_NAME}")

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)
