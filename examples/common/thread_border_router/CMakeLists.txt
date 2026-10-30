# 移除 esp_ot_cli_extension 依赖（CLI 功能已禁用）
set(requires openthread protocol_examples_common vfs esp_wifi esp_eth esp_rcp_update esp_event app_update)

if(CONFIG_OPENTHREAD_CLI_OTA)
    list(APPEND requires esp_http_client esp_br_http_ota)
endif()

idf_component_register(SRC_DIRS "src"
                       INCLUDE_DIRS "include"
                       REQUIRES "${requires}"
                       # HYP F-RCP-003: the RCP recovery hooks hash the stored H2 images
                       # and bound their reads with esp_timer.
                       PRIV_REQUIRES mbedtls esp_timer
)
