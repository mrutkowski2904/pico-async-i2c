add_library(async_i2c async_i2c.c)

target_sources(async_i2c INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/async_i2c.c
        )

target_include_directories(async_i2c PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )

target_link_libraries(async_i2c
        pico_stdlib
        )