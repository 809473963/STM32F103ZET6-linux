cmake_minimum_required(VERSION 3.10)
project(STM32_KEY_PROJECT C CXX ASM)

option(ENCODER_MOTOR_APP "Build encoder motor app" OFF)
option(MOTOR86_APP "Build 86 stepper motor app" OFF)
option(HELLO_ACK_APP "Build hello/ack serial app" OFF)

if((ENCODER_MOTOR_APP AND MOTOR86_APP) OR
   (ENCODER_MOTOR_APP AND HELLO_ACK_APP) OR
   (MOTOR86_APP AND HELLO_ACK_APP))
    message(FATAL_ERROR "Only one app option can be enabled at a time")
endif()

if(ENCODER_MOTOR_APP)
    add_definitions(-DENCODER_MOTOR_APP)
endif()

if(MOTOR86_APP)
    add_definitions(-DMOTOR86_APP)
endif()

if(HELLO_ACK_APP)
    add_definitions(-DHELLO_ACK_APP)
endif()

# Add support for assembler
enable_language(ASM)

# ==================== CONFIGURATION ====================
# Select specific target micro-controller
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Defines from Keil project
add_definitions(-DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER)

# ==================== COMPILER OPTIONS ====================
# MCU Specific flags for STM32F103ZE (Cortex-M3)
set(MCU_FLAGS "-mcpu=cortex-m3 -mthumb -msoft-float")

# C/C++ Compiler Flags
set(CMAKE_C_FLAGS "${MCU_FLAGS} -O2 -Wall -ffunction-sections -fdata-sections --specs=nano.specs" CACHE INTERNAL "C Compiler options")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS}" CACHE INTERNAL "C++ Compiler options")
set(CMAKE_ASM_FLAGS "${MCU_FLAGS} -x assembler-with-cpp" CACHE INTERNAL "ASM Compiler options")

# Linker Flags
# Use our custom linker script
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32F103ZETx_FLASH.ld)
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} -T${LINKER_SCRIPT} -Wl,--gc-sections -Wl,-Map=${PROJECT_NAME}.map,--cref --specs=nosys.specs" CACHE INTERNAL "Linker options")

# ==================== SOURCE FILES ====================

# Automatically find all C source files in the specified directories
file(GLOB_RECURSE USER_SOURCES "USER/*.c")
if(ENCODER_MOTOR_APP)
    list(FILTER USER_SOURCES EXCLUDE REGEX "USER/main\\.c$")
    file(GLOB_RECURSE USER_ENCODER_SOURCES "USER_ENCODER_MOTOR/*.c")
    list(APPEND USER_SOURCES ${USER_ENCODER_SOURCES})
endif()
if(MOTOR86_APP)
    list(FILTER USER_SOURCES EXCLUDE REGEX "USER/main\\.c$")
    file(GLOB_RECURSE USER_MOTOR86_SOURCES "USER_MOTOR86/*.c")
    list(APPEND USER_SOURCES ${USER_MOTOR86_SOURCES})
endif()
if(HELLO_ACK_APP)
    list(FILTER USER_SOURCES EXCLUDE REGEX "USER/main\\.c$")
    file(GLOB_RECURSE USER_HELLO_SOURCES "USER_HELLO/*.c")
    list(APPEND USER_SOURCES ${USER_HELLO_SOURCES})
endif()
file(GLOB_RECURSE HARDWARE_SOURCES "HARDWARE/*.c")
list(FILTER HARDWARE_SOURCES EXCLUDE REGEX "HARDWARE/LEGACY/.*\\.c$")
file(GLOB_RECURSE SYSTEM_SOURCES "SYSTEM/*.c")
file(GLOB_RECURSE FWLIB_SOURCES "STM32F10x_FWLib/src/*.c")

# Add the GCC startup file (ignore Keil's .s)
set(STARTUP_FILE "CORE/startup_stm32f103xe_gcc.s")

# Combine all source files
set(SOURCES
    ${USER_SOURCES}
    ${HARDWARE_SOURCES}
    ${SYSTEM_SOURCES}
    ${FWLIB_SOURCES}
    ${STARTUP_FILE}
    CORE/core_cm3.c
)

# Remove the default stm32f10x_it.c or other duplicates if necessary,
# but since Keil puts it in USER, we just use the GLOB ones.
# Also, remove the Keil startup file from the list if it was accidentally grabbed
list(FILTER SOURCES EXCLUDE REGEX "startup_.*_hd.s$")

# ==================== INCLUDE DIRECTORIES ====================
# Setup include directories
include_directories(
    CORE
    STM32F10x_FWLib/inc
    SYSTEM/delay
    SYSTEM/sys
    SYSTEM/usart
    USER
)

if(ENCODER_MOTOR_APP)
    include_directories(USER_ENCODER_MOTOR)
endif()

if(MOTOR86_APP)
    include_directories(USER_MOTOR86)
endif()

if(HELLO_ACK_APP)
    include_directories(USER_HELLO)
endif()

# Auto-add all subdirectories under HARDWARE
file(GLOB HARDWARE_DIRS LIST_DIRECTORIES true "HARDWARE/*")
foreach(dir ${HARDWARE_DIRS})
    if(IS_DIRECTORY ${dir})
        include_directories(${dir})
    endif()
endforeach()

# ==================== BUILD TARGETS ====================

# Build ELF firmware
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Post-build actions to generate HEX and BIN files
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -S ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
    COMMENT "Building .hex and .bin files and displaying size"
)

# ==================== FLASH TARGET ====================
# Add a custom target for flashing using STM32CubeProgrammer
set(STM32_PROGRAMMER_CLI "STM32_Programmer_CLI")
add_custom_target(flash
    COMMAND ${STM32_PROGRAMMER_CLI} -c port=SWD -w ${PROJECT_NAME}.bin 0x08000000 -rst
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Flashing ${PROJECT_NAME}.bin to device over SWD/USB using STM32CubeProgrammer..."
)
