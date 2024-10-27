/* Common functions for protocol examples, to establish Wi-Fi or Ethernet connection.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */

#pragma once
#include "sdkconfig.h"

#include "esp_err.h"
#if !CONFIG_IDF_TARGET_LINUX
#include "esp_netif.h"
#if CONFIG_EXAMPLE_CONNECT_ETHERNET
#include "esp_eth.h"
#endif
#endif // !CONFIG_IDF_TARGET_LINUX

#ifdef __cplusplus
extern "C" {
#endif

#if !CONFIG_IDF_TARGET_LINUX
#if CONFIG_EXAMPLE_CONNECT_WIFI
#define NETIF_WIFI_DESC_NAME "netif_wifi"
#endif

#if CONFIG_EXAMPLE_CONNECT_ETHERNET
#define NETIF_ETH_DESC_NAME "netif_eth"
#endif

/* Example default interface, prefer the ethernet one if running in example-test (CI) configuration */
#if CONFIG_EXAMPLE_CONNECT_ETHERNET
#define EXAMPLE_INTERFACE get_example_netif_from_desc(NETIF_ETH_DESC_NAME)
#define get_example_netif() get_example_netif_from_desc(NETIF_ETH_DESC_NAME)
#elif CONFIG_EXAMPLE_CONNECT_WIFI
#define EXAMPLE_INTERFACE get_example_netif_from_desc(NETIF_WIFI_DESC_NAME)
#define get_example_netif() get_example_netif_from_desc(NETIF_WIFI_DESC_NAME)
#endif

/**
 * @brief Configure Wi-Fi or Ethernet, connect, wait for IP
 *
 * This all-in-one helper function is used in protocols examples to
 * reduce the amount of boilerplate in the example.
 *
 * It is not intended to be used in real world applications.
 * See examples under examples/wifi/getting_started/ and examples/ethernet/
 * for more complete Wi-Fi or Ethernet initialization code.
 *
 * Read "Establishing Wi-Fi or Ethernet Connection" section in
 * examples/protocols/README.md for more information about this function.
 *
 * @return ESP_OK on successful connection
 */
esp_err_t network_connect(void);

/**
 * Counterpart to network_connect, de-initializes Wi-Fi or Ethernet
 */
esp_err_t network_disconnect(void);

/**
 * @brief Returns esp-netif pointer created by network_connect() described by
 * the supplied desc field
 *
 * @param desc Textual interface of created network interface, for example "sta"
 * indicate default WiFi station, "eth" default Ethernet interface.
 *
 */
esp_netif_t *get_example_netif_from_desc(const char *desc);

#if CONFIG_EXAMPLE_PROVIDE_WIFI_CONSOLE_CMD
/**
 * @brief Register wifi connect commands
 *
 * Provide a simple wifi_connect command in esp_console.
 * This function can be used after esp_console is initialized.
 */
void example_register_wifi_connect_commands(void);
#endif

#if CONFIG_EXAMPLE_CONNECT_ETHERNET
/**
 * @brief Get the example Ethernet driver handle
 *
 * @return esp_eth_handle_t
 */
esp_eth_handle_t get_example_eth_handle(void);
#endif // CONFIG_EXAMPLE_CONNECT_ETHERNET

#else
static inline esp_err_t example_connect(void) {return ESP_OK;}
#endif // !CONFIG_IDF_TARGET_LINUX

#ifdef __cplusplus
}
#endif
