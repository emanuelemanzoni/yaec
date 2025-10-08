idf_component_register(SRCS "sensors.cpp"
                       INCLUDE_DIRS "include"
                       INCLUDE_DIRS ""
                       REQUIRES driver
                       REQUIRES esp_adc
                       REQUIRES common_interfaces)