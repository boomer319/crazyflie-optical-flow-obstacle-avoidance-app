cmd_src/modules/src/stabilizer.o := arm-none-eabi-gcc -Wp,-MD,src/modules/src/.stabilizer.o.d    -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/src -Isrc/modules/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/libdw1000/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/platform/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/deck/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/deck/drivers/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/bosch/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/esp32/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/kve   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/tdoa   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/FatFS   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/vl53l1   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/app_obstacle_avoidance/build/include/generated -fno-delete-null-pointer-checks -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -c -o src/modules/src/stabilizer.o /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/src/stabilizer.c

source_src/modules/src/stabilizer.o := /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/src/stabilizer.c

deps_src/modules/src/stabilizer.o := \
    $(wildcard include/config/deck/usd.h) \
  /usr/include/newlib/math.h \
  /usr/include/newlib/sys/reent.h \
  /usr/include/newlib/_ansi.h \
  /usr/include/newlib/newlib.h \
  /usr/include/newlib/_newlib_version.h \
  /usr/include/newlib/sys/config.h \
    $(wildcard include/config/h//.h) \
  /usr/include/newlib/machine/ieeefp.h \
  /usr/include/newlib/sys/features.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stddef.h \
  /usr/include/newlib/sys/_types.h \
  /usr/include/newlib/machine/_types.h \
  /usr/include/newlib/machine/_default_types.h \
  /usr/include/newlib/sys/lock.h \
  /usr/include/newlib/sys/cdefs.h \
  /usr/include/newlib/_ansi.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdint.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdbool.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config/trace.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/usec_time.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/cfassert.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config/stm32fxxx.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/stm32f4xx.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/core_cm4.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_version.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/mpu_armv7.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/system_stm32f4xx.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config/stm32f4xx_conf.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_crc.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dbgmcu.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dma.h \
    $(wildcard include/config/it.h) \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_exti.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_flash.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_i2c.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_iwdg.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_pwr.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rcc.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rtc.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_sdio.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_spi.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_syscfg.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_tim.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_usart.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_wwdg.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_misc.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_cryp.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_hash.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rng.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_can.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dac.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dcmi.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_fsmc.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/console.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/eprintf.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdarg.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/system.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/log.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/param.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/param_logic.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/crtp.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/debug.h \
    $(wildcard include/config/debug/print/on/uart1.h) \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config/config.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/interface/motors.h \
    $(wildcard include/config/motors/esc/protocol/oneshot125.h) \
    $(wildcard include/config/motors/esc/protocol/oneshot42.h) \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/pm.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/interface/adc.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/semphr.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/syslink.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/deck/interface/deck.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/deck/interface/deck_core.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/estimator.h \
    $(wildcard include/config/estimator/kalman/enable.h) \
    $(wildcard include/config/estimator/oot.h) \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/imu_types.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/deck/interface/deck_constants.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/deck/interface/deck_digital.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/deck/interface/deck_analog.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/deck/interface/deck_spi.h \
  /usr/include/newlib/string.h \
  /usr/include/newlib/sys/_locale.h \
  /usr/include/newlib/strings.h \
  /usr/include/newlib/sys/string.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/stabilizer.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/sensors.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/commander.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/crtp_commander_high_level.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/math3d.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/crtp_localization_service.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/lighthouse/pulse_processor.h \
  /usr/include/newlib/stdlib.h \
  /usr/include/newlib/machine/stdlib.h \
  /usr/include/newlib/alloca.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/lighthouse/ootx_decoder.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_calibration.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_geometry.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/arm_math.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/float.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include-fixed/limits.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/controller.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/power_distribution.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/collision_avoidance.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/health.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/supervisor.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/deck/drivers/interface/usddeck.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/quatcompress.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/statsCnt.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/static_mem.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/rateSupervisor.h \

src/modules/src/stabilizer.o: $(deps_src/modules/src/stabilizer.o)

$(deps_src/modules/src/stabilizer.o):
