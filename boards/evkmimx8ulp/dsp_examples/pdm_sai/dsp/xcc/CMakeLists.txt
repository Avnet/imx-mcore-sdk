INCLUDE(CMakeForceCompiler)

# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

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


SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -DDEBUG")

SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -DNDEBUG")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DSDK_DEBUGCONSOLE=1")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DCPU_MIMX8UD7DVP10_dsp0")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DMCUXPRESSO_SDK")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O1")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} --xtensa-core=fusion_nxp02_dsp_prod")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} --xtensa-system=${XTENSA_SYSTEM}")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fsigned-char")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wno-unused")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wno-missing-braces")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fmessage-length=0")

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -std=gnu99")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DNDEBUG")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DSDK_DEBUGCONSOLE=1")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DCPU_MIMX8UD7DVP10_dsp0")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DMCUXPRESSO_SDK")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Os")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} --xtensa-core=fusion_nxp02_dsp_prod")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} --xtensa-system=${XTENSA_SYSTEM}")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fsigned-char")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wno-unused")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wno-missing-braces")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -fmessage-length=0")

SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -std=gnu99")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DMCUXPRESSO_SDK")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DMCUXPRESSO_SDK")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -mlsp=${ProjDirPath}/../../../../../../devices/MIMX8UD7/xtensa/gdbio")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -g")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --xtensa-core=fusion_nxp02_dsp_prod")

SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --xtensa-system=${XTENSA_SYSTEM}")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mlsp=${ProjDirPath}/../../../../../../devices/MIMX8UD7/xtensa/min-rt")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --xtensa-core=fusion_nxp02_dsp_prod")

SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} --xtensa-system=${XTENSA_SYSTEM}")

include_directories(${ProjDirPath}/..)

include_directories(${ProjDirPath}/../../../../../../components/uart)

include_directories(${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers)

include_directories(${ProjDirPath}/../../../../../../devices/MIMX8UD7/utilities/debug_console_lite)

include_directories(${ProjDirPath}/../../../../../../components/lists)

include_directories(${ProjDirPath}/../../../../../../devices/MIMX8UD7)

include_directories(${ProjDirPath}/../../../../../../CMSIS/Core/Include)

add_executable(dsp_pdm_sai_fusionf1.elf 
"${ProjDirPath}/../main_dsp.c"
"${ProjDirPath}/../dsp_config.h"
"${ProjDirPath}/../pin_mux.c"
"${ProjDirPath}/../pin_mux.h"
"${ProjDirPath}/../board_fusionf1.c"
"${ProjDirPath}/../board_fusionf1.h"
"${ProjDirPath}/../../../../../../components/uart/fsl_adapter_uart.h"
"${ProjDirPath}/../../../../../../components/uart/fsl_adapter_lpuart.c"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_lpuart.h"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_lpuart.c"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/utilities/debug_console_lite/fsl_debug_console.c"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/utilities/debug_console_lite/fsl_debug_console.h"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/utilities/debug_console_lite/fsl_assert.c"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_edma.h"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_edma.c"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_pdm_edma.h"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_pdm_edma.c"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_sai_edma.h"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_sai_edma.c"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_common.h"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_common.c"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_common_dsp.c"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_common_dsp.h"
"${ProjDirPath}/../../../../../../components/lists/fsl_component_generic_list.h"
"${ProjDirPath}/../../../../../../components/lists/fsl_component_generic_list.c"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_clock.h"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_clock.c"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_reset.h"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_reset.c"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/MIMX8UD7_dsp0.h"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/MIMX8UD7_dsp0_features.h"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/system_MIMX8UD7_dsp0.c"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/system_MIMX8UD7_dsp0.h"
"${ProjDirPath}/../../../../../../CMSIS/Core/Include/cmsis_armcc.h"
"${ProjDirPath}/../../../../../../CMSIS/Core/Include/cmsis_armclang.h"
"${ProjDirPath}/../../../../../../CMSIS/Core/Include/cmsis_armclang_ltm.h"
"${ProjDirPath}/../../../../../../CMSIS/Core/Include/cmsis_compiler.h"
"${ProjDirPath}/../../../../../../CMSIS/Core/Include/cmsis_gcc.h"
"${ProjDirPath}/../../../../../../CMSIS/Core/Include/cmsis_iccarm.h"
"${ProjDirPath}/../../../../../../CMSIS/Core/Include/cmsis_version.h"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_pdm.h"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_pdm.c"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_sai.h"
"${ProjDirPath}/../../../../../../devices/MIMX8UD7/drivers/fsl_sai.c"
)


TARGET_LINK_LIBRARIES(dsp_pdm_sai_fusionf1.elf -Wl,--start-group)
target_link_libraries(dsp_pdm_sai_fusionf1.elf debug xos)

target_link_libraries(dsp_pdm_sai_fusionf1.elf optimized xos)

TARGET_LINK_LIBRARIES(dsp_pdm_sai_fusionf1.elf -Wl,--end-group)

ADD_CUSTOM_COMMAND(TARGET dsp_pdm_sai_fusionf1.elf POST_BUILD COMMAND ${CMAKE_OBJCOPY}
--xtensa-params= -Obinary ${EXECUTABLE_OUTPUT_PATH}/dsp_pdm_sai_fusionf1.elf ${EXECUTABLE_OUTPUT_PATH}/../../binary/dsp_reset_${CMAKE_BUILD_TYPE}.bin
--xtensa-core=fusion_nxp02_dsp_prod
--xtensa-system=${XTENSA_SYSTEM}
--only-section=.ResetVector.text
)

ADD_CUSTOM_COMMAND(TARGET dsp_pdm_sai_fusionf1.elf POST_BUILD COMMAND ${CMAKE_OBJCOPY}
--xtensa-params= -Obinary ${EXECUTABLE_OUTPUT_PATH}/dsp_pdm_sai_fusionf1.elf ${EXECUTABLE_OUTPUT_PATH}/../../binary/dsp_text_${CMAKE_BUILD_TYPE}.bin
--xtensa-core=fusion_nxp02_dsp_prod
--xtensa-system=${XTENSA_SYSTEM}
--only-section=.WindowVectors.text
--only-section=.Level2InterruptVector.text
--only-section=.Level3InterruptVector.text
--only-section=.DebugExceptionVector.text
--only-section=.NMIExceptionVector.text
--only-section=.KernelExceptionVector.text
--only-section=.UserExceptionVector.text
--only-section=.DoubleExceptionVector.text
--only-section=.text
)

ADD_CUSTOM_COMMAND(TARGET dsp_pdm_sai_fusionf1.elf POST_BUILD COMMAND ${CMAKE_OBJCOPY}
--xtensa-params= -Obinary ${EXECUTABLE_OUTPUT_PATH}/dsp_pdm_sai_fusionf1.elf ${EXECUTABLE_OUTPUT_PATH}/../../binary/dsp_data_${CMAKE_BUILD_TYPE}.bin
--xtensa-core=fusion_nxp02_dsp_prod
--xtensa-system=${XTENSA_SYSTEM}
--only-section=.rtos.rodata
--only-section=.rodata
--only-section=.rtos.percpu.data
--only-section=.data
--only-section=.bss
)

