# THIS FILE IS AUTOMATICALLY GENERATED. DO NOT EDIT.
# BASED ON c:\BLACKBOARD\AAA_UNITS\AAA_HONORS\mmc_example

function(add_st_target_properties TARGET_NAME)

target_compile_definitions(
    ${TARGET_NAME} PRIVATE
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:ASM>>:DEBUG>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:DEBUG>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:USE_HAL_DRIVER>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:STM32L476xx>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:USE_HAL_DRIVER>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:STM32L476xx>"
)

target_include_directories(
    ${TARGET_NAME} PRIVATE
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Core\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_DEVICE\\App>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_DEVICE\\Target>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32L4xx_HAL_Driver\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32L4xx_HAL_Driver\\Inc\\Legacy>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\ST\\STM32_USB_Device_Library\\Core\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\ST\\STM32_USB_Device_Library\\Class\\MSC\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Device\\ST\\STM32L4xx\\Include>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Include>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Core\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_DEVICE\\App>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_DEVICE\\Target>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32L4xx_HAL_Driver\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32L4xx_HAL_Driver\\Inc\\Legacy>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\ST\\STM32_USB_Device_Library\\Core\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\ST\\STM32_USB_Device_Library\\Class\\MSC\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Device\\ST\\STM32L4xx\\Include>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Include>"
)

target_compile_options(
    ${TARGET_NAME} PRIVATE
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:ASM>>:-g3>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:-g3>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:-g3>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:ASM>>:-g0>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:-g0>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:-g0>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:-Os>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:-Os>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:>"
    "$<$<CONFIG:Debug>:-mcpu=cortex-m4>"
    "$<$<CONFIG:Debug>:-mfpu=fpv4-sp-d16>"
    "$<$<CONFIG:Debug>:-mfloat-abi=hard>"
    "$<$<NOT:$<CONFIG:Debug>>:-mcpu=cortex-m4>"
    "$<$<NOT:$<CONFIG:Debug>>:-mfpu=fpv4-sp-d16>"
    "$<$<NOT:$<CONFIG:Debug>>:-mfloat-abi=hard>"
)

target_link_libraries(
    ${TARGET_NAME} PRIVATE
)

target_link_directories(
    ${TARGET_NAME} PRIVATE
)

target_link_options(
    ${TARGET_NAME} PRIVATE
    "$<$<CONFIG:Debug>:-mcpu=cortex-m4>"
    "$<$<CONFIG:Debug>:-mfpu=fpv4-sp-d16>"
    "$<$<CONFIG:Debug>:-mfloat-abi=hard>"
    "$<$<NOT:$<CONFIG:Debug>>:-mcpu=cortex-m4>"
    "$<$<NOT:$<CONFIG:Debug>>:-mfpu=fpv4-sp-d16>"
    "$<$<NOT:$<CONFIG:Debug>>:-mfloat-abi=hard>"
    -T
    "$<$<CONFIG:Debug>:${PROJECT_SOURCE_DIR}/STM32L476RETX_FLASH.ld>"
    "$<$<NOT:$<CONFIG:Debug>>:${PROJECT_SOURCE_DIR}/STM32L476RETX_FLASH.ld>"
)

target_sources(
    ${TARGET_NAME} PRIVATE
    "Core\\Src\\main.c"
    "Core\\Src\\stm32l4xx_hal_msp.c"
    "Core\\Src\\stm32l4xx_hal_timebase_tim.c"
    "Core\\Src\\stm32l4xx_it.c"
    "Core\\Src\\syscalls.c"
    "Core\\Src\\sysmem.c"
    "Core\\Src\\system_stm32l4xx.c"
    "Core\\Startup\\startup_stm32l476retx.s"
    "Middlewares\\ST\\STM32_USB_Device_Library\\Class\\MSC\\Src\\usbd_msc_bot.c"
    "Middlewares\\ST\\STM32_USB_Device_Library\\Class\\MSC\\Src\\usbd_msc_data.c"
    "Middlewares\\ST\\STM32_USB_Device_Library\\Class\\MSC\\Src\\usbd_msc_scsi.c"
    "Middlewares\\ST\\STM32_USB_Device_Library\\Class\\MSC\\Src\\usbd_msc.c"
    "Middlewares\\ST\\STM32_USB_Device_Library\\Core\\Src\\usbd_core.c"
    "Middlewares\\ST\\STM32_USB_Device_Library\\Core\\Src\\usbd_ctlreq.c"
    "Middlewares\\ST\\STM32_USB_Device_Library\\Core\\Src\\usbd_ioreq.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_cortex.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_crc_ex.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_crc.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_dma_ex.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_dma.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_exti.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_flash_ex.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_flash_ramfunc.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_flash.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_gpio.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_i2c_ex.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_i2c.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_mmc_ex.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_mmc.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_pcd_ex.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_pcd.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_pwr_ex.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_pwr.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_rcc_ex.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_rcc.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_tim_ex.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_tim.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_ll_sdmmc.c"
    "Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_ll_usb.c"
    "USB_DEVICE\\App\\usb_device.c"
    "USB_DEVICE\\App\\usbd_desc.c"
    "USB_DEVICE\\App\\usbd_storage_if.c"
    "USB_DEVICE\\Target\\usbd_conf.c"
)

add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
)

add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
    $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.hex
)

add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
    $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.bin
)

endfunction()