cmd_src/modules/src/built-in.o :=  arm-none-eabi-gcc --specs=nosys.specs --specs=nano.specs -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -nostdlib  -r -o src/modules/src/built-in.o src/modules/src/app_channel.o src/modules/src/app_handler.o src/modules/src/bootloader.o src/modules/src/attitude_pid_controller.o src/modules/src/collision_avoidance.o src/modules/src/commander.o src/modules/src/comm.o src/modules/src/console.o src/modules/src/controller_indi.o src/modules/src/controller_mellinger.o src/modules/src/controller.o src/modules/src/controller_pid.o src/modules/src/crtp_commander_generic.o src/modules/src/crtp_commander_high_level.o src/modules/src/crtp_commander.o src/modules/src/crtp_commander_rpyt.o src/modules/src/crtp_localization_service.o src/modules/src/crtp.o src/modules/src/crtpservice.o src/modules/src/esp_deck_flasher.o src/modules/src/estimator_complementary.o src/modules/src/estimator_kalman.o src/modules/src/estimator.o src/modules/src/eventtrigger.o src/modules/src/extrx.o src/modules/src/health.o src/modules/src/kalman_supervisor.o src/modules/src/log.o src/modules/src/mem.o src/modules/src/msp.o src/modules/src/outlierFilter.o src/modules/src/param_logic.o src/modules/src/param_task.o src/modules/src/peer_localization.o src/modules/src/pid.o src/modules/src/planner.o src/modules/src/platformservice.o src/modules/src/position_controller_indi.o src/modules/src/position_controller_pid.o src/modules/src/position_estimator_altitude.o src/modules/src/power_distribution_stock.o src/modules/src/pptraj_compressed.o src/modules/src/pptraj.o src/modules/src/queuemonitor.o src/modules/src/range.o src/modules/src/sensfusion6.o src/modules/src/sound_cf2.o src/modules/src/stabilizer.o src/modules/src/static_mem.o src/modules/src/supervisor.o src/modules/src/sysload.o src/modules/src/system.o src/modules/src/tdoaEngineInstance.o src/modules/src/worker.o src/modules/src/kalman_core/built-in.o src/modules/src/lighthouse/built-in.o
