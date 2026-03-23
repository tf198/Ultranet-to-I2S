# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(P16-Open C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

### Stagebox ###

add_executable(op16_stagebox stagebox.c ultranet.c i2s.c ultranet.h)

#pico_set_program_name(op16_stagebox "op16_stagebox")

# Generate PIO header
pico_generate_pio_header(op16_stagebox ${CMAKE_CURRENT_LIST_DIR}/ultranet.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(op16_stagebox 1)
pico_enable_stdio_usb(op16_stagebox 1)

# Add the standard library to the build
target_link_libraries(op16_stagebox
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(op16_stagebox PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(op16_stagebox 
        hardware_pio
        hardware_pwm
        hardware_clocks
        hardware_dma
        pico_multicore
        )

pico_add_extra_outputs(op16_stagebox)

# From original
#add_custom_target(REBUILD_BINARY_INFO
#  COMMAND touch ${PICO_SDK_PATH}/src/rp2_common/pico_standard_link/binary_info.c
#)
#add_dependencies(${component} REBUILD_BINARY_INFO)

### Personal Mixer ###

add_executable(op16_mixer mixer.c ultranet.c i2s.c ultranet.h)

#pico_set_program_name(op16_mixer "op16_mixer")

# Generate PIO header
pico_generate_pio_header(op16_mixer ${CMAKE_CURRENT_LIST_DIR}/ultranet.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(op16_mixer 1)
pico_enable_stdio_usb(op16_mixer 1)

# Add the standard library to the build
target_link_libraries(op16_mixer
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(op16_mixer PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(op16_mixer 
        hardware_pio
        hardware_pwm
        hardware_clocks
        hardware_dma
        pico_multicore
        )

pico_add_extra_outputs(op16_mixer)