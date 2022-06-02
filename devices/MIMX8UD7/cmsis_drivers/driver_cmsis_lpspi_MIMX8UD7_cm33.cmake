include_guard(GLOBAL)
message("driver_cmsis_lpspi component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/fsl_lpspi_cmsis.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/.
)


include(driver_lpspi_MIMX8UD7_cm33)

include(CMSIS_Driver_Include_SPI_MIMX8UD7_cm33)

