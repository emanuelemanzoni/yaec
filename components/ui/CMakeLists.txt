idf_component_register(
    # List all C and C++ source files in this component
    SRCS "ui_task.cpp" "oled_display.c"

    # Directories for header files
    INCLUDE_DIRS "." "include"

    # This component requires the standard ESP-IDF driver and SPI flash components
    PRIV_REQUIRES 
        driver 
        spi_flash
        common_interfaces
        bike_sensors
)