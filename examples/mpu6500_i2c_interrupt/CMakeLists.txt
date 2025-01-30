cmake_minimum_required(VERSION 3.19)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

add_subdirectory(../../mp92plus)

#get_filename_component(BASE_FOLDER ${CMAKE_CURRENT_SOURCE_DIR} NAME)
#set(PROJECT_NAME ${BASE_FOLDER})
set(PROJECT_NAME mpu6500_i2c_interrupt)

project(${PROJECT_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${PROJECT_NAME} 
    main.cpp
)

pico_add_extra_outputs(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
	mp92plus
) 

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
