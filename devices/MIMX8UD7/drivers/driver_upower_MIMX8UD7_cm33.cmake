include_guard(GLOBAL)
message("driver_upower component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/fsl_upower.c
    ${CMAKE_CURRENT_LIST_DIR}/upower/upower_api.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/.
    ${CMAKE_CURRENT_LIST_DIR}/upower
)


include(driver_common_MIMX8UD7_cm33)

