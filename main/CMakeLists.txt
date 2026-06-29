file(GLOB_RECURSE app_sources "*.c")
idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "." "drivers" "modules" "interface"
    REQUIRES driver esp_timer freertos
)
