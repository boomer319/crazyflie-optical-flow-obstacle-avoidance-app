cmd_src/modules/src/estimator/estimator_kalman.o := arm-none-eabi-gcc -Wp,-MD,src/modules/src/estimator/.estimator_kalman.o.d    -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/src/estimator -Isrc/modules/src/estimator -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/libdw1000/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/platform/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/deck/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/deck/drivers/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/bosch/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/esp32/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/outlierfilter   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/cpx   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/p2pDTR   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/controller   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/estimator   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/kve   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/tdoa   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/FatFS   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/vl53l1   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/app_obstacle_avoidance_by_imitating_cf/build/include/generated -fno-delete-null-pointer-checks -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -c -o src/modules/src/estimator/estimator_kalman.o /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/src/estimator/estimator_kalman.c

source_src/modules/src/estimator/estimator_kalman.o := /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/src/estimator/estimator_kalman.c

deps_src/modules/src/estimator/estimator_kalman.o := \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core/kalman_core.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/cf_math.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/arm_math.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdint.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /usr/include/newlib/string.h \
  /usr/include/newlib/_ansi.h \
  /usr/include/newlib/newlib.h \
  /usr/include/newlib/_newlib_version.h \
  /usr/include/newlib/sys/config.h \
    $(wildcard include/config/h//.h) \
  /usr/include/newlib/machine/ieeefp.h \
  /usr/include/newlib/sys/features.h \
  /usr/include/newlib/sys/reent.h \
  /usr/include/newlib/_ansi.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stddef.h \
  /usr/include/newlib/sys/_types.h \
  /usr/include/newlib/machine/_types.h \
  /usr/include/newlib/machine/_default_types.h \
  /usr/include/newlib/sys/lock.h \
  /usr/include/newlib/sys/cdefs.h \
  /usr/include/newlib/sys/_locale.h \
  /usr/include/newlib/strings.h \
  /usr/include/newlib/sys/string.h \
  /usr/include/newlib/math.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/float.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include-fixed/limits.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/cfassert.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdbool.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/imu_types.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_supervisor.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config/trace.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/usec_time.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/cfassert.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/semphr.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/hal/interface/sensors.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/static_mem.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/estimator/estimator.h \
    $(wildcard include/config/estimator/kalman/enable.h) \
    $(wildcard include/config/estimator/ukf/enable.h) \
    $(wildcard include/config/estimator/oot.h) \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/estimator/estimator_kalman.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/system.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/log.h \
    $(wildcard include/config/debug/log/enable.h) \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/param.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/param_logic.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/crtp.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/physicalConstants.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/supervisor.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/axis3fSubSampler.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/statsCnt.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/rateSupervisor.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core/mm_distance.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core/mm_absolute_height.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core/mm_position.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core/mm_pose.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core/mm_tdoa.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/outlierfilter/outlierFilterTdoa.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core/mm_flow.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core/mm_tof.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core/mm_yaw_error.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core/mm_sweep_angles.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/outlierfilter/outlierFilterLighthouse.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core/mm_tdoa_robust.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/kalman_core/mm_distance_robust.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/debug.h \
    $(wildcard include/config/debug/print/on/uart1.h) \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/config/config.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/modules/interface/console.h \
  /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/utils/interface/eprintf.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdarg.h \

src/modules/src/estimator/estimator_kalman.o: $(deps_src/modules/src/estimator/estimator_kalman.o)

$(deps_src/modules/src/estimator/estimator_kalman.o):
