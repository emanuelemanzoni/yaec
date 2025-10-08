idf_component_register(SRCS "main_v0.cpp"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS ""
                       REQUIRES bike_sensors
                       REQUIRES can_bus_comm
                       REQUIRES common_interfaces
                       REQUIRES control_logic
                       REQUIRES ui)
