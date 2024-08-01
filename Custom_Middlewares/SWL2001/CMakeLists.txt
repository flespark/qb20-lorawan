cmake_minimum_required(VERSION 3.22)

if ("${DRIVER_NAME}" STREQUAL "")
    get_filename_component(DRIVER_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    string(APPEND DRIVER_NAME "_driver")
endif()

set(DRIVER_PROJECT_NAME ${DRIVER_NAME})

project(${DRIVER_PROJECT_NAME})

add_library(${DRIVER_PROJECT_NAME} INTERFACE)

target_compile_definitions(${DRIVER_PROJECT_NAME} INTERFACE
    SX126X
    SX1262
    RP2_103 # Regional Parameter
    # REGION_AS_923
    # REGION_AU_915
    # REGION_CN_470
    # REGION_CN_470_RP_1_0
    # REGION_EU_868
    # REGION_IN_865
    # REGION_KR_920
    # REGION_RU_864
    REGION_US_915
    NUMBER_OF_STACKS=1
    MODEM_HAL_DBG_TRACE=1
    MODEM_HAL_DEEP_DBG_TRACE=0
)

target_compile_options(${DRIVER_PROJECT_NAME} INTERFACE
	-mabi=aapcs
	-Wpedantic
    -Wno-unused-parameter
    -Wfatal-errors
	-ffast-math
    -fstack-usage
    -fno-builtin
)

file(GLOB DRIVER_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/radio_hal/radio_utilities.c
    ${CMAKE_CURRENT_SOURCE_DIR}/radio_hal/ral_sx126x_bsp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/radio_hal/sx126x_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_hal_l4/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_modem_hal/smtc_modem_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lorawan_api/lorawan_api.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lorawan_manager/lorawan_cid_request_management.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lorawan_manager/lorawan_dwn_ack_management.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lorawan_manager/lorawan_join_management.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lorawan_manager/lorawan_send_management.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lorawan_packages/lorawan_certification/lorawan_certification.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/lr1mac_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/lr1mac_utilities.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/lr1_stack_mac_layer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/services/smtc_duty_cycle.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/services/smtc_lbt.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/smtc_real/src/region_as_923.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/smtc_real/src/region_au_915.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/smtc_real/src/region_cn_470.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/smtc_real/src/region_cn_470_rp_1_0.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/smtc_real/src/region_eu_868.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/smtc_real/src/region_in_865.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/smtc_real/src/region_kr_920.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/smtc_real/src/region_ru_864.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/smtc_real/src/region_us_915.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/smtc_real/src/smtc_real.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/modem_supervisor/modem_supervisor_light.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/modem_supervisor/modem_tx_protocol_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/modem_utilities/fifo_ctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/modem_utilities/modem_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/modem_utilities/modem_event_utilities.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/radio_drivers/sx126x_driver/src/lr_fhss_mac.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/radio_drivers/sx126x_driver/src/sx126x.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/radio_drivers/sx126x_driver/src/sx126x_lr_fhss.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/radio_planner/src/radio_planner.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/smtc_modem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/smtc_modem_crypto/smtc_modem_crypto.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/smtc_modem_crypto/soft_secure_element/aes.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/smtc_modem_crypto/soft_secure_element/cmac.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/smtc_modem_crypto/soft_secure_element/soft_se.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/smtc_modem_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/smtc_ralf/src/ralf_sx126x.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/smtc_ral/src/ral_sx126x.c
)

set(DRIVER_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/smtc_hal_l4
    ${CMAKE_CURRENT_SOURCE_DIR}/radio_hal
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_api
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lorawan_api
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lorawan_manager
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lorawan_packages
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lorawan_packages/lorawan_certification
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/services
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/lr1mac/src/smtc_real/src
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/modem_supervisor
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/modem_utilities
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/radio_drivers/sx126x_driver/src
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/radio_planner/src
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/smtc_modem_crypto
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/smtc_modem_crypto/smtc_secure_element
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/smtc_modem_crypto/soft_secure_element
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/smtc_ralf/src
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_core/smtc_ral/src
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm_lib/smtc_modem_hal
)

target_sources(${DRIVER_PROJECT_NAME} INTERFACE ${DRIVER_SOURCES})

target_include_directories(${DRIVER_PROJECT_NAME} INTERFACE ${DRIVER_INCLUDE_DIRS})

target_link_libraries(${DRIVER_PROJECT_NAME} INTERFACE
    $<LINK_ONLY:stm32cubemx>

    # Add user defined libraries
)