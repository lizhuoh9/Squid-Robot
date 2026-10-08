idf_component_register(
    SRCS
        "main.cpp" "Output.cpp" "Console.cpp" "CH9434A.cpp"
        "KalmanFilter.cpp" "DepthSensorManager.cpp" "UltrasonicManager.cpp"
        "ImuManager.cpp" "MotionLink.cpp" "MotionLock.cpp" "StatusDisplay.cpp"
        "ForwardControl.cpp" "TurnControl.cpp" "DepthController.cpp"
        "AutoNavigator.cpp" "SensorHub.cpp" "SdCard.cpp" "SDLogger.cpp"
        "CommandHandler.cpp" "OtaManager.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_adc fatfs sdmmc esp_wifi esp_netif esp_event nvs_flash
             esp_http_server app_update esp_timer lwip vfs
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-format-truncation)
