cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME FreeRTOS_LED)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Src/led_task.c
        Core/Inc/led_task.h
        Core/Src/numadd_task.c
        Core/Inc/numadd_task.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)


# ---------------------------------------------------------
# 添加 OpenOCD 烧录任务 (针对 ST-Link + STM32F1x)
# ---------------------------------------------------------
add_custom_target(flash
    # OpenOCD 命令解释:
    # -f interface/stlink.cfg : 使用 ST-Link 调试器
    # -f target/stm32f1x.cfg  : 目标芯片是 STM32F1 系列 (包含 C8T6)
    # -c "program ... "       : 烧录 elf 文件，校验(verify)，复位(reset)，然后退出(exit)
    COMMAND openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf verify reset exit"
    
    # 确保在烧录前先完成构建
    DEPENDS ${CMAKE_PROJECT_NAME}
    
    # 打印提示信息
    COMMENT "Flashing STM32F1x via OpenOCD + ST-Link    ..."
    USES_TERMINAL
)