
cmake_minimum_required(VERSION 3.10)

set (CODE_SOURCE_DIR src)
set (CODE_HEADERS_DIR inc)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(pico-neuron-serial DESCRIPTION "Program that reads the serial data from a Raspberry Pico neuron model and interacts with it" LANGUAGES C CXX)

set(SOURCES
    ${PROJECT_SOURCE_DIR}/${CODE_SOURCE_DIR}/pico-neuron-serial.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR} 
    ${PROJECT_SOURCE_DIR}/${CODE_HEADERS_DIR}
    ${PROJECT_SOURCE_DIR}/${CODE_SOURCE_DIR}
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(
        ${PROJECT_NAME} )
 
