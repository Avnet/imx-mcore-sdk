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


project(rpmsg_lite_str_echo_rtos_imxcm33)

set(MCUX_SDK_PROJECT_NAME rpmsg_lite_str_echo_rtos_imxcm33.elf)

include(${ProjDirPath}/flags.cmake)

include(${ProjDirPath}/config.cmake)

add_executable(${MCUX_SDK_PROJECT_NAME} 
"${ProjDirPath}/../main_remote.c"
"${ProjDirPath}/../FreeRTOSConfig.h"
"${ProjDirPath}/../pin_mux.c"
"${ProjDirPath}/../pin_mux.h"
"${ProjDirPath}/../rpmsg_config.h"
"${ProjDirPath}/../rsc_table.c"
"${ProjDirPath}/../rsc_table.h"
"${ProjDirPath}/../remoteproc.h"
"${ProjDirPath}/../board.c"
"${ProjDirPath}/../board.h"
"${ProjDirPath}/../clock_config.c"
"${ProjDirPath}/../clock_config.h"
"${ProjDirPath}/../app_srtm.c"
"${ProjDirPath}/../app_srtm.h"
"${ProjDirPath}/../srtm/channels/srtm_rpmsg_endpoint.h"
"${ProjDirPath}/../srtm/channels/srtm_rpmsg_endpoint.c"
"${ProjDirPath}/../srtm/services/srtm_audio_service.h"
"${ProjDirPath}/../srtm/services/srtm_audio_service.c"
"${ProjDirPath}/../srtm/services/srtm_sai_edma_adapter.h"
"${ProjDirPath}/../srtm/services/srtm_sai_edma_adapter.c"
"${ProjDirPath}/../srtm/services/srtm_i2c_service.h"
"${ProjDirPath}/../srtm/services/srtm_i2c_service.c"
"${ProjDirPath}/../srtm/services/srtm_sensor_service.h"
"${ProjDirPath}/../srtm/services/srtm_sensor_service.c"
"${ProjDirPath}/../srtm/services/srtm_io_service.h"
"${ProjDirPath}/../srtm/services/srtm_io_service.c"
"${ProjDirPath}/../srtm/services/srtm_keypad_service.h"
"${ProjDirPath}/../srtm/services/srtm_keypad_service.c"
"${ProjDirPath}/../srtm/services/srtm_pwm_service.h"
"${ProjDirPath}/../srtm/services/srtm_pwm_service.c"
"${ProjDirPath}/../srtm/services/srtm_pwm_adapter.h"
"${ProjDirPath}/../srtm/services/srtm_pwm_adapter.c"
"${ProjDirPath}/../srtm/services/srtm_rtc_service.h"
"${ProjDirPath}/../srtm/services/srtm_rtc_service.c"
"${ProjDirPath}/../srtm/services/srtm_rtc_adapter.h"
"${ProjDirPath}/../srtm/services/srtm_rtc_adapter.c"
"${ProjDirPath}/../srtm/services/srtm_lfcl_service.h"
"${ProjDirPath}/../srtm/services/srtm_lfcl_service.c"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${ProjDirPath}/..
    ${ProjDirPath}/../srtm/channels
    ${ProjDirPath}/../srtm/services
)

set(CMAKE_MODULE_PATH
    ${ProjDirPath}/../../../../../middleware/multicore
    ${ProjDirPath}/../../../../../rtos/freertos/freertos_kernel
    ${ProjDirPath}/../../../../../devices/MIMX8UD7/drivers
    ${ProjDirPath}/../../../../../devices/MIMX8UD7
    ${ProjDirPath}/../../../../../devices/MIMX8UD7/utilities
    ${ProjDirPath}/../../../../../components/uart
    ${ProjDirPath}/../../../../../components/serial_manager
    ${ProjDirPath}/../../../../../components/lists
    ${ProjDirPath}/../../../../../CMSIS/Core/Include
    ${ProjDirPath}/../../../../../components/pca6416a
    ${ProjDirPath}/../../../../../components/srtm/port
    ${ProjDirPath}/../../../../../components/srtm
    ${ProjDirPath}/../../../../../components/pwm
    ${ProjDirPath}/../../../../../components/rtc
)

# include modules
include(middleware_multicore_rpmsg_lite_MIMX8UD7_cm33)

include(middleware_multicore_rpmsg_lite_freertos_MIMX8UD7_cm33)

include(middleware_freertos-kernel_heap_4_MIMX8UD7_cm33)

include(middleware_freertos-kernel_cm33_nonsecure_port_MIMX8UD7_cm33)

include(driver_mu1_MIMX8UD7_cm33)

include(driver_rgpio_MIMX8UD7_cm33)

include(driver_clock_MIMX8UD7_cm33)

include(middleware_freertos-kernel_MIMX8UD7_cm33)

include(driver_common_MIMX8UD7_cm33)

include(device_MIMX8UD7_CMSIS_MIMX8UD7_cm33)

include(utility_debug_console_MIMX8UD7_cm33)

include(utility_assert_MIMX8UD7_cm33)

include(component_lpuart_adapter_MIMX8UD7_cm33)

include(driver_flexspi_MIMX8UD7_cm33)

include(component_serial_manager_MIMX8UD7_cm33)

include(component_lists_MIMX8UD7_cm33)

include(component_serial_manager_uart_MIMX8UD7_cm33)

include(driver_lpuart_MIMX8UD7_cm33)

include(device_MIMX8UD7_startup_MIMX8UD7_cm33)

include(driver_cache_cache64_MIMX8UD7_cm33)

include(driver_iomuxc_MIMX8UD7_cm33)

include(driver_reset_MIMX8UD7_cm33)

include(driver_upower_MIMX8UD7_cm33)

include(driver_fusion_MIMX8UD7_cm33)

include(driver_dma3_MIMX8UD7_cm33)

include(driver_lpi2c_MIMX8UD7_cm33)

include(CMSIS_Include_core_cm_MIMX8UD7_cm33)

include(driver_lpi2c_freertos_MIMX8UD7_cm33)

include(middleware_freertos-kernel_extension_MIMX8UD7_cm33)

include(driver_sai_edma_MIMX8UD7_cm33)

include(driver_sai_MIMX8UD7_cm33)

include(driver_wuu_MIMX8UD7_cm33)

include(driver_pca6416a_MIMX8UD7_cm33)

include(driver_sentinel_MIMX8UD7_cm33)

include(driver_srtm_freertos_MIMX8UD7_cm33)

include(driver_srtm_MIMX8UD7_cm33)

include(middleware_multicore_rpmsg_lite_imx8ulp_m33_freertos_MIMX8UD7_cm33)

include(component_pwm_tpm_adapter_MIMX8UD7_cm33)

include(driver_tpm_MIMX8UD7_cm33)

include(component_rtc_bbnsm_adapter_MIMX8UD7_cm33)

include(driver_bbnsm_MIMX8UD7_cm33)

include(utilities_misc_utilities_MIMX8UD7_cm33)


TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE m)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE c)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE gcc)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE nosys)

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

ADD_CUSTOM_COMMAND(TARGET ${MCUX_SDK_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY}
-Obinary ${EXECUTABLE_OUTPUT_PATH}/${MCUX_SDK_PROJECT_NAME} ${EXECUTABLE_OUTPUT_PATH}/rpmsg_lite_str_echo_rtos.bin)

