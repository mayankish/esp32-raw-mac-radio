idf_component_register(
    SRCS "main.c" "data_contract.c" "raw_link.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_event esp_netif nvs_flash mdns lwip
)
