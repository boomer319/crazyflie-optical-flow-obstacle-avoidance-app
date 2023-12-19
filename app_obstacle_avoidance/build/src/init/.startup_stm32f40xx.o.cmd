cmd_src/init/startup_stm32f40xx.o := arm-none-eabi-gcc -Wp,-MD,src/init/.startup_stm32f40xx.o.d   -D__firmware__ -fno-exceptions -D__ASSEMBLY__ -DCC_HAVE_ASM_GOTO   -c -o src/init/startup_stm32f40xx.o /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/init/startup_stm32f40xx.S

source_src/init/startup_stm32f40xx.o := /home/aurelio/verrueckter_schwarm/nanoflownet_flussnetz/crazyflie-optical-flow-obstacle-avoidance-app/crazyflie-firmware/src/init/startup_stm32f40xx.S

deps_src/init/startup_stm32f40xx.o := \

src/init/startup_stm32f40xx.o: $(deps_src/init/startup_stm32f40xx.o)

$(deps_src/init/startup_stm32f40xx.o):
