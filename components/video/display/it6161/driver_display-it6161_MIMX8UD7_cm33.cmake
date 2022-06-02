include_guard(GLOBAL)
message("driver_display-it6161 component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/fsl_it6161.c
    ${CMAKE_CURRENT_LIST_DIR}/hdmi_tx.c
    ${CMAKE_CURRENT_LIST_DIR}/mipi_rx.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/.
)


include(driver_video-common_MIMX8UD7_cm33)

include(driver_video-i2c_MIMX8UD7_cm33)

include(driver_display-common_MIMX8UD7_cm33)

