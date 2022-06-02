# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 3.10.0)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})

SET(EXECUTABLE_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})
SET(LIBRARY_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})


project(pxp_color_key)

set(MCUX_SDK_PROJECT_NAME pxp_color_key.elf)

include(${ProjDirPath}/flags.cmake)

include(${ProjDirPath}/config.cmake)

add_executable(${MCUX_SDK_PROJECT_NAME} 
"${ProjDirPath}/../pxp_color_key.c"
"${ProjDirPath}/../pin_mux.c"
"${ProjDirPath}/../pin_mux.h"
"${ProjDirPath}/../display_support.h"
"${ProjDirPath}/../display_support.c"
"${ProjDirPath}/../board.c"
"${ProjDirPath}/../board.h"
"${ProjDirPath}/../clock_config.c"
"${ProjDirPath}/../clock_config.h"
"${ProjDirPath}/../empty_rsc_table.c"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${ProjDirPath}/..
)

set(CMAKE_MODULE_PATH
    ${ProjDirPath}/../../../../../../components/video
    ${ProjDirPath}/../../../../../../components/video/display
    ${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers
    ${ProjDirPath}/../../../../../../components/video/display/rm68200
    ${ProjDirPath}/../../../../../../components/video/display/mipi_dsi_cmd
    ${ProjDirPath}/../../../../../../components/video/display/hx8394
    ${ProjDirPath}/../../../../../../components/pca6416a
    ${ProjDirPath}/../../../../../../components/video/display/dc/lcdif
    ${ProjDirPath}/../../../../../../devices/MIMX8UD7/utilities/debug_console_lite
    ${ProjDirPath}/../../../../../../components/video/display/dc
    ${ProjDirPath}/../../../../../../devices/MIMX8UD7
    ${ProjDirPath}/../../../../../../components/uart
    ${ProjDirPath}/../../../../../../components/lists
    ${ProjDirPath}/../../../../../../CMSIS/Core/Include
    ${ProjDirPath}/../../../../../../devices/MIMX8UD7/utilities
)

# include modules
include(driver_video-common_MIMX8UD7_cm33)

include(driver_display-common_MIMX8UD7_cm33)

include(driver_lpi2c_MIMX8UD7_cm33)

include(driver_display-rm68200_MIMX8UD7_cm33)

include(driver_mipi_dsi_MIMX8UD7_cm33)

include(driver_tpm_MIMX8UD7_cm33)

include(driver_display-mipi-dsi-cmd_MIMX8UD7_cm33)

include(driver_common_MIMX8UD7_cm33)

include(driver_rgpio_MIMX8UD7_cm33)

include(driver_display-hx8394_MIMX8UD7_cm33)

include(driver_pca6416a_MIMX8UD7_cm33)

include(driver_dc-fb-lcdif_MIMX8UD7_cm33)

include(utility_debug_console_lite_MIMX8UD7_cm33)

include(utility_assert_lite_MIMX8UD7_cm33)

include(driver_pxp_MIMX8UD7_cm33)

include(driver_dc-fb-common_MIMX8UD7_cm33)

include(driver_clock_MIMX8UD7_cm33)

include(device_MIMX8UD7_CMSIS_MIMX8UD7_cm33)

include(component_lpuart_adapter_MIMX8UD7_cm33)

include(driver_flexspi_MIMX8UD7_cm33)

include(component_lists_MIMX8UD7_cm33)

include(driver_lpuart_MIMX8UD7_cm33)

include(device_MIMX8UD7_startup_MIMX8UD7_cm33)

include(driver_cache_cache64_MIMX8UD7_cm33)

include(driver_iomuxc_MIMX8UD7_cm33)

include(driver_reset_MIMX8UD7_cm33)

include(driver_upower_MIMX8UD7_cm33)

include(driver_fusion_MIMX8UD7_cm33)

include(CMSIS_Include_core_cm_MIMX8UD7_cm33)

include(driver_soc_mipi_dsi_MIMX8UD7_cm33)

include(driver_lcdif_MIMX8UD7_cm33)

include(utilities_misc_utilities_MIMX8UD7_cm33)


TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE m)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE c)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE gcc)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE nosys)

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

ADD_CUSTOM_COMMAND(TARGET ${MCUX_SDK_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY}
-Obinary ${EXECUTABLE_OUTPUT_PATH}/${MCUX_SDK_PROJECT_NAME} ${EXECUTABLE_OUTPUT_PATH}/sdk20-app.bin)

