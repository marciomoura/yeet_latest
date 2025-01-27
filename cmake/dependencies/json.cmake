# Description: fetch the json package from nlohmann/json

message(
    STATUS
        "-- yeet build-config: 🟢 searching json package for ${CMAKE_SYSTEM_NAME}"
)

include(FetchContent)

fetchcontent_declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
fetchcontent_makeavailable(json)
