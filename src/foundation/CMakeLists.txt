add_library(av_foundation INTERFACE)

target_include_directories(av_foundation INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<INSTALL_INTERFACE:include>
)

if(Eigen3_FOUND)
    target_link_libraries(av_foundation INTERFACE Eigen3::Eigen)
endif()
if(spdlog_FOUND)
    target_link_libraries(av_foundation INTERFACE spdlog::spdlog)
endif()
if(nlohmann_json_FOUND)
    target_link_libraries(av_foundation INTERFACE nlohmann_json::nlohmann_json)
endif()

set_target_properties(av_foundation PROPERTIES
    LINKER_LANGUAGE CXX
)

# Source files (mostly header-only)
target_sources(av_foundation INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/logging.cpp
)
