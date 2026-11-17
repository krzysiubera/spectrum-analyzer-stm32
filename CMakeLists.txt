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
set(CMAKE_PROJECT_NAME stm32_uart)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# LVGL middleware configuration belongs here, outside the CubeMX-generated
# CMake file, so regenerating the STM32 project does not remove it.
set(LV_CONF_SKIP OFF CACHE BOOL "Skip lv_conf.h and use internal configuration defaults" FORCE)
set(LV_BUILD_CONF_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/lv_conf.h
    CACHE PATH "Project LVGL configuration header" FORCE
)
set(CONFIG_LV_BUILD_EXAMPLES OFF CACHE BOOL "Do not build LVGL examples" FORCE)
set(CONFIG_LV_BUILD_DEMOS OFF CACHE BOOL "Do not build LVGL demos" FORCE)
set(CONFIG_LV_USE_THORVG OFF CACHE BOOL "Do not build LVGL ThorVG support" FORCE)
set(LV_FETCH_DEPENDENCIES OFF CACHE BOOL "Do not fetch optional LVGL dependencies" FORCE)
set(LV_BUILD_INSTALL OFF CACHE BOOL "Do not install LVGL from this firmware project" FORCE)

add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/lvgl
    ${CMAKE_CURRENT_BINARY_DIR}/lvgl
)
target_link_libraries(${CMAKE_PROJECT_NAME} lvgl::lvgl)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

    # CMSIS DSP sources
    Drivers/CMSIS_DSP/Source/BasicMathFunctions/BasicMathFunctions.c
    Drivers/CMSIS_DSP/Source/CommonTables/CommonTables.c
    Drivers/CMSIS_DSP/Source/ComplexMathFunctions/ComplexMathFunctions.c
    Drivers/CMSIS_DSP/Source/FastMathFunctions/FastMathFunctions.c
    Drivers/CMSIS_DSP/Source/StatisticsFunctions/StatisticsFunctions.c
    Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctions.c
    Drivers/CMSIS_DSP/Source/TransformFunctions/TransformFunctions.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths

    # CMSIS DSP includes
    Drivers/CMSIS_DSP/Include
    Drivers/CMSIS_DSP/PrivateInclude
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols

    # CMSIS DSP compile definitions
    ARM_MATH_CM4
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
