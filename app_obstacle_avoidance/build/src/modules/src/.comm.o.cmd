cmd_src/modules/src/comm.o := arm-none-eabi-gcc -Wp,-MD,src/modules/src/.comm.o.d    -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/src -Isrc/modules/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/libdw1000/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/platform/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/deck/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/deck/drivers/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/bosch/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/esp32/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/kve   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/tdoa   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/FatFS   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/vl53l1   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/app_obstacle_avoidance/build/include/generated -fno-delete-null-pointer-checks -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -c -o src/modules/src/comm.o /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/src/comm.c

source_src/modules/src/comm.o := /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/src/comm.c

deps_src/modules/src/comm.o := \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdbool.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config/trace.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/usec_time.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdint.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/crtp.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/console.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/eprintf.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdarg.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/crtpservice.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/param_task.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/log.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/eskylink.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/interface/uart_syslink.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/syslink.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/radiolink.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/syslink.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/usblink.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/platformservice.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/crtp.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/crtp_localization_service.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/imu_types.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/lighthouse/pulse_processor.h \
  /usr/include/newlib/stdlib.h \
  /usr/include/newlib/machine/ieeefp.h \
  /usr/include/newlib/_ansi.h \
  /usr/include/newlib/newlib.h \
  /usr/include/newlib/_newlib_version.h \
  /usr/include/newlib/sys/config.h \
    $(wildcard include/config/h//.h) \
  /usr/include/newlib/sys/features.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stddef.h \
  /usr/include/newlib/sys/reent.h \
  /usr/include/newlib/_ansi.h \
  /usr/include/newlib/sys/_types.h \
  /usr/include/newlib/machine/_types.h \
  /usr/include/newlib/machine/_default_types.h \
  /usr/include/newlib/sys/lock.h \
  /usr/include/newlib/sys/cdefs.h \
  /usr/include/newlib/machine/stdlib.h \
  /usr/include/newlib/alloca.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/lighthouse/ootx_decoder.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_calibration.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_geometry.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/arm_math.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /usr/include/newlib/string.h \
  /usr/include/newlib/sys/_locale.h \
  /usr/include/newlib/strings.h \
  /usr/include/newlib/sys/string.h \
  /usr/include/newlib/math.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/float.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include-fixed/limits.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/stabilizer_types.h \

src/modules/src/comm.o: $(deps_src/modules/src/comm.o)

$(deps_src/modules/src/comm.o):
