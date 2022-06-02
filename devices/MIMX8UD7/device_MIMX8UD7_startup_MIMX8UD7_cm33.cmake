include_guard(GLOBAL)
message("device_MIMX8UD7_startup component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/gcc/startup_MIMX8UD7_cm33.S
)


