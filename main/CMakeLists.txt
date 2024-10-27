idf_component_register(
  SRCS "network.c" "wifi.c" "esp32-rgb-main.c" 
  INCLUDE_DIRS "." 
  PRIV_REQUIRES esp_netif driver esp_wifi vfs esp_timer nvs_flash
)
