idf_component_register(
    SRCS
    "main.c"
    "clock.c"
    "motor.c"
    "encoder.c"
    "ota.c"
    INCLUDE_DIRS "."
)
set_target_properties(${COMPONENT_LIB} PROPERTIES COMPILE_FLAGS "-save-temps")
