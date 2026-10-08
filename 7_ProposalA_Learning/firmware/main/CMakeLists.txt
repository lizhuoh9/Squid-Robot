idf_component_get_property(espdl_dir espressif__esp-dl COMPONENT_DIR)
set(cmake_dir "${espdl_dir}/fbs_loader/cmake")
include(${cmake_dir}/utilities.cmake)
set(proposal_model_dir "${CMAKE_CURRENT_LIST_DIR}/../../artifacts/hybrid_mpc_v1")
idf_component_register(
    SRCS
        "main.cpp" "Output.cpp" "Console.cpp" "CH9434A.cpp"
        "KalmanFilter.cpp" "DepthSensorManager.cpp" "UltrasonicManager.cpp"
        "ImuManager.cpp" "MotionLink.cpp" "MotionLock.cpp" "StatusDisplay.cpp"
        "ForwardControl.cpp" "TurnControl.cpp" "DepthController.cpp"
        "AutoNavigator.cpp" "SensorHub.cpp" "SdCard.cpp" "SDLogger.cpp"
        "CommandHandler.cpp" "OtaManager.cpp" "RamCapture.cpp" "ProposalStudent.cpp"
    INCLUDE_DIRS "." "${proposal_model_dir}"
    REQUIRES driver esp_adc fatfs sdmmc esp_wifi esp_netif esp_event nvs_flash
             esp_http_server app_update esp_timer lwip vfs json espressif__esp-dl
)
target_add_aligned_binary_data(${COMPONENT_LIB} "${proposal_model_dir}/proposal_student.espdl" BINARY)
include(${espdl_dir}/cmake/compile_register.cmake)
esp_dl_compile_add_yml("${CMAKE_CURRENT_LIST_DIR}/proposal_student.compile_req.yml")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-format-truncation)
