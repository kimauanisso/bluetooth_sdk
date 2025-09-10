if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Pico SDK ---
set(PICO_BOARD pico)
set(PICO_PLATFORM rp2040)
set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(bt_kmn_project C CXX ASM)

pico_sdk_init()

add_library(bt_kmn STATIC
    src/commands.c
    src/bluetooth.c
)

target_include_directories(bt_kmn PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /usr/lib/arm-none-eabi/include
    /usr/lib/gcc/arm-none-eabi/14.2.0/include
    /usr/lib/gcc/arm-none-eabi/14.2.0/include-fixed
)

target_link_libraries(bt_kmn
    pico_stdlib
    hardware_uart
    hardware_irq
)

# --- Executável de teste ---
add_executable(bt_kmn_test
    src/test.c
)

target_link_libraries(bt_kmn_test
    bt_kmn
    pico_stdlib
)

pico_enable_stdio_uart(bt_kmn_test 1)
pico_enable_stdio_usb(bt_kmn_test 1)

pico_add_extra_outputs(bt_kmn_test)
