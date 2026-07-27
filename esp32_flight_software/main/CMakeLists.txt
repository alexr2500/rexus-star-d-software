idf_component_register(
    SRCS "main.c" "mode_manager.c" "state_machine.c" "error_manager.c"
         "health_monitor.c" "telemetry.c" "temperature_control.c"
         "sensors.c" "watchdog.c"
    INCLUDE_DIRS "."
    REQUIRES drivers links
)