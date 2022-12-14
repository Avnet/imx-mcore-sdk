include_guard(GLOBAL)
message("driver_display-rm68200 component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/fsl_rm68200.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/.
)


include(driver_display-common_MIMX8UD7_cm33)

include(driver_display-mipi-dsi-cmd_MIMX8UD7_cm33)

