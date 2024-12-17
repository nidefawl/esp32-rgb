idf_component_register(
  # SRCS "network.c" "wifi.c" "esp32-rgb-main.c" 
  # SRCS "example_nvs_wifi_connect.c" "example_echo_ws_server.c"
  SRCS "esp32-rgb-main.c" "http-server.c"
  INCLUDE_DIRS "." 
  PRIV_REQUIRES esp_netif driver esp_wifi vfs esp_timer nvs_flash
  nvs_wifi_connect
  led_strip_async
  esp_http_server 
  esp_wifi
  mdns
EMBED_FILES 
  index.html
  style.css
)
