cmd_src/modules/src/kalman_core/built-in.o :=  arm-none-eabi-gcc --specs=nosys.specs --specs=nano.specs -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -nostdlib  -r -o src/modules/src/kalman_core/built-in.o src/modules/src/kalman_core/kalman_core.o src/modules/src/kalman_core/mm_absolute_height.o src/modules/src/kalman_core/mm_distance.o src/modules/src/kalman_core/mm_distance_robust.o src/modules/src/kalman_core/mm_flow.o src/modules/src/kalman_core/mm_pose.o src/modules/src/kalman_core/mm_position.o src/modules/src/kalman_core/mm_sweep_angles.o src/modules/src/kalman_core/mm_tdoa.o src/modules/src/kalman_core/mm_tdoa_robust.o src/modules/src/kalman_core/mm_tof.o src/modules/src/kalman_core/mm_yaw_error.o
