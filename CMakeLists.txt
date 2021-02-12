cmake_minimum_required(VERSION 3.12)

set(PICO_SDK_PATH "~/Development/pico-sdk")
# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
set(CMAKE_C_FLAGS_DEBUG "-g -O0")

project(my_project)

# initialize the Pico SDK
pico_sdk_init()

# rest of your project
add_executable(main main.c func.s)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(main pico_stdlib pico_multicore)
pico_enable_stdio_usb(main 0)
pico_enable_stdio_uart(main 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(main)