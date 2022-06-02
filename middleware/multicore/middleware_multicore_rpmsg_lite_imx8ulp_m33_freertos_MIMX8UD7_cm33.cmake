include_guard(GLOBAL)
message("middleware_multicore_rpmsg_lite_imx8ulp_m33_freertos component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/rpmsg_lite/lib/rpmsg_lite/porting/platform/imx8ulp_m33/rpmsg_platform.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/rpmsg_lite/lib/include/platform/imx8ulp_m33
)


include(middleware_freertos-kernel_MIMX8UD7_cm33)

include(middleware_freertos-kernel_heap_4_MIMX8UD7_cm33)

