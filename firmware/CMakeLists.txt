cmake_minimum_required(VERSION 3.10)
project(STM32_KEY_PROJECT C CXX ASM)

option(ARM_DM_APP "Build CAN arm DM app" ON)

if(NOT ARM_DM_APP)
    message(FATAL_ERROR "This project has been reduced to ARM_DM_APP only")
endif()

add_definitions(-DARM_DM_APP)

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
list(FILTER USER_SOURCES EXCLUDE REGEX "USER/main\\.c$")
file(GLOB_RECURSE USER_ARM_DM_SOURCES "USER_ARM_DM/*.c")
list(APPEND USER_SOURCES ${USER_ARM_DM_SOURCES})
file(GLOB_RECURSE HARDWARE_SOURCES "HARDWARE/*.c")
list(FILTER HARDWARE_SOURCES EXCLUDE REGEX "HARDWARE/DM4310/dm4310_can\\.c$")
list(FILTER HARDWARE_SOURCES EXCLUDE REGEX "HARDWARE/GRIPPER42/.*")
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

include_directories(USER_ARM_DM)

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
