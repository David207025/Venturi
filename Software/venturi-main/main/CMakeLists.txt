idf_component_register(
        SRCS
        "venturi-main.cpp"
        "VenturiTools/VenturiTools.cpp"
        "VenturiEngine/VenturiEngine.cpp"
        INCLUDE_DIRS
        "."
        PRIV_REQUIRES
        driver
        esp_driver_touch_sens
        esp_driver_uart
        esp_driver_tsens
        esp_driver_gptimer
        esp_timer
        fatfs
        sdmmc
        vfs
)

target_compile_options(${COMPONENT_LIB} PRIVATE
        -O3
        -funroll-loops
        -fno-tree-vectorize
)

component_compile_options(-Wno-missing-field-initializers -Wno-unknown-pragmas)