idf_build_get_property(component_targets __COMPONENT_TARGETS)
if ("___idf_espressif__esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir espressif__esp-dl COMPONENT_DIR)
elseif ("___idf_esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir esp-dl COMPONENT_DIR)
else()
    message(FATAL_ERROR "ESP-DL component was not resolved by the ESP-IDF Component Manager")
endif()
set(cmake_dir "${espdl_dir}/fbs_loader/cmake")
include(${cmake_dir}/utilities.cmake)

set(depth_model_dir "${CMAKE_CURRENT_LIST_DIR}/../../5_DepthLearning/artifacts/depth_model_based_v1")
set(depth_espdl_file "${depth_model_dir}/depth_model_espdl.espdl")
if (NOT EXISTS "${depth_espdl_file}")
    message(FATAL_ERROR "Missing ${depth_espdl_file}. Run learning/export_espdl.py before building.")
endif()
if (NOT EXISTS "${depth_model_dir}/DepthResidualModelConfig.h")
    message(FATAL_ERROR "Missing model normalization header. Re-export this model with --metadata-output.")
endif()

idf_component_register(
    SRCS
        "main.cpp"
        "Output.cpp"
        "Console.cpp"
        "CH9434A.cpp"
        "KalmanFilter.cpp"
        "DepthSensorManager.cpp"
        "UltrasonicManager.cpp"
        "ImuManager.cpp"
        "MotionLink.cpp"
        "MotionLock.cpp"
        "StatusDisplay.cpp"
        "ForwardControl.cpp"
        "TurnControl.cpp"
        "DepthController.cpp"
        "DepthResidualModel.cpp"
        "AutoNavigator.cpp"
        "SensorHub.cpp"
        "SdCard.cpp"
        "SDLogger.cpp"
        "CommandHandler.cpp"
        "OtaManager.cpp"
    INCLUDE_DIRS
        "."
        "${depth_model_dir}"
    REQUIRES
        driver
        esp_adc
        fatfs
        sdmmc
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        esp_http_server
        app_update
        esp_timer
        lwip
        vfs
        espressif__esp-dl
)

target_add_aligned_binary_data(${COMPONENT_LIB} ${depth_espdl_file} BINARY)

include(${espdl_dir}/cmake/compile_register.cmake)
esp_dl_compile_add_yml("${CMAKE_CURRENT_LIST_DIR}/depth_model.compile_req.yml")

# 长文件名/路径拼接可能被截断（非致命，仅裁剪超长名），关掉该 -Werror 类别。
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-format-truncation)
