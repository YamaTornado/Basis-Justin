# fc_core: platform-independent flight software (filters, state machine,
# log/telemetry formats). No ESP-IDF dependency -- builds standalone on the
# host (for sim/ and tests/) and, via the ESP_PLATFORM branch below, as a
# regular ESP-IDF component when included from firmware/.

set(FC_CORE_SOURCES
    src/attitude_filter.c
    src/altitude_filter.c
    src/flight_state.c
    src/framing.c
    src/log_format.c
    src/telemetry.c
)

if(ESP_PLATFORM)
    idf_component_register(
        SRCS ${FC_CORE_SOURCES}
        INCLUDE_DIRS include
        REQUIRES ""
    )
    target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
else()
    cmake_minimum_required(VERSION 3.16)
    project(fc_core C)

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)

    add_library(fc_core STATIC ${FC_CORE_SOURCES})
    target_include_directories(fc_core PUBLIC include)
    target_link_libraries(fc_core PUBLIC m)
    target_compile_options(fc_core PRIVATE -Wall -Wextra)

    option(FC_CORE_BUILD_TESTS "Build fc_core unit tests" ON)
    if(FC_CORE_BUILD_TESTS)
        enable_testing()
        add_subdirectory(tests)
    endif()
endif()
