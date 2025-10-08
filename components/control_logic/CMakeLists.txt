idf_component_register(SRCS "control_logic.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES common_interfaces
                       REQUIRES can_bus_comm
                       REQUIRES bike_sensors
                       REQUIRES esp_timer)