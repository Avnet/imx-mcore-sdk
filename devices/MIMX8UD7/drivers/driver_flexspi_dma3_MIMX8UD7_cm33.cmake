include_guard(GLOBAL)
message("driver_flexspi_dma3 component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/fsl_flexspi_edma.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/.
)


include(driver_flexspi_MIMX8UD7_cm33)

include(driver_dma3_MIMX8UD7_cm33)

