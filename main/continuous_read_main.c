/*
 * Sensor DHT11 con ESP32 - ThingsBoard & OLED + Portal Web Avanzado
 * Con configuración persistente en NVS
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "u8g2.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "cJSON.h"

// Incluir archivos HTML
#include "html_forms.h"
#include "html_responses.h"

// --- TAG para logs ---
static const char *TAG = "ESP32_DHT11";

// --- Configuración ---
typedef struct {
    // DHT11
    int dht_gpio;
    float temperature;
    float humidity;
    
    // WiFi Configuration (se guarda en NVS)
    char wifi_ssid[32];
    char wifi_password[64];
    int32_t wifi_max_retries;
    int32_t wifi_retry_delay_ms;
    
    // Default WiFi (fallback)
    char default_ssid[32];
    char default_password[64];
    
    // AP Configuration
    char ap_password[32];
    char ap_ssid[32];
    
    // MQTT - ThingsBoard
    char mqtt_broker[64];
    uint32_t mqtt_port;
    char mqtt_token[64];
    char mqtt_telemetry_topic[64];
    
    // OLED
    uint8_t oled_address;
    int i2c_sda_pin;
    int i2c_scl_pin;
} app_config_t;

static app_config_t config = {
    .dht_gpio = 4,
    .temperature = 0,
    .humidity = 0,
    .wifi_max_retries = 5,
    .wifi_retry_delay_ms = 1000,
    .default_ssid = "SBC",
    .default_password = "SBCwifi$",
    .ap_password = "config123",
    .mqtt_broker = "demo.thingsboard.io",
    .mqtt_port = 1883,
    .mqtt_token = "HGf7saV16hOPmVOmkTwb",
    .mqtt_telemetry_topic = "v1/devices/me/telemetry",
    .oled_address = 0x3C,
    .i2c_sda_pin = 21,
    .i2c_scl_pin = 22
};

// --- Variables globales ---
static esp_mqtt_client_handle_t mqtt_client = NULL;
static u8g2_t u8g2;
static bool wifi_connected = false;
static bool mqtt_connected = false;
static httpd_handle_t server = NULL;
static int current_retry_count = 0;

// --- NVS Keys ---
#define NVS_NAMESPACE "wifi_config"
#define NVS_KEY_SSID "ssid"
#define NVS_KEY_PASSWORD "password"
#define NVS_KEY_RETRIES "max_retries"
#define NVS_KEY_DELAY "retry_delay_ms"

// --- Prototipos ---
static void wifi_init(void);
static void wifi_connect_sta(void);
static void wifi_start_ap(void);
static void start_webserver(void);
static void stop_webserver(void);
static esp_err_t init_oled(void);
static esp_err_t init_mqtt(void);
static bool read_dht11_data(void);
static void display_dht11_data(void);
static void send_dht11_data(void);
static bool wait_for_connection(bool check_wifi, bool check_mqtt, int timeout_ms);
static void main_application_loop(void);
static char* create_telemetry_json(void);

// --- NVS Functions ---
static esp_err_t save_wifi_config(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    ESP_LOGI(TAG, "💾 Guardando configuración WiFi en NVS...");
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error abriendo NVS: %s", esp_err_to_name(err));
        return err;
    }
    
    // Guardar SSID
    err = nvs_set_str(nvs_handle, NVS_KEY_SSID, config.wifi_ssid);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error guardando SSID: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    // Guardar password
    err = nvs_set_str(nvs_handle, NVS_KEY_PASSWORD, config.wifi_password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error guardando password: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    // Guardar max retries
    err = nvs_set_i32(nvs_handle, NVS_KEY_RETRIES, config.wifi_max_retries);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error guardando retries: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    // Guardar retry delay
    err = nvs_set_i32(nvs_handle, NVS_KEY_DELAY, config.wifi_retry_delay_ms);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error guardando delay: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "✅ Configuración WiFi guardada en NVS:");
        ESP_LOGI(TAG, "   SSID: %s", config.wifi_ssid);
        ESP_LOGI(TAG, "   Max Retries: %d", config.wifi_max_retries);
        ESP_LOGI(TAG, "   Retry Delay: %d ms", config.wifi_retry_delay_ms);
    } else {
        ESP_LOGE(TAG, "❌ Error guardando en NVS: %s", esp_err_to_name(err));
    }
    
    return err;
}

static esp_err_t load_wifi_config(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    size_t required_size;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo abrir NVS (puede ser la primera ejecución)");
        return err;
    }
    
    // Leer SSID
    required_size = sizeof(config.wifi_ssid);
    err = nvs_get_str(nvs_handle, NVS_KEY_SSID, config.wifi_ssid, &required_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se encontró SSID guardado");
        nvs_close(nvs_handle);
        return err;
    }
    
    // Leer password
    required_size = sizeof(config.wifi_password);
    err = nvs_get_str(nvs_handle, NVS_KEY_PASSWORD, config.wifi_password, &required_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se encontró password guardado");
        nvs_close(nvs_handle);
        return err;
    }
    
    // Leer max retries
    err = nvs_get_i32(nvs_handle, NVS_KEY_RETRIES, &config.wifi_max_retries);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se encontraron retries, usando valor por defecto: 5");
        config.wifi_max_retries = 5;
    }
    
    // Leer retry delay
    err = nvs_get_i32(nvs_handle, NVS_KEY_DELAY, &config.wifi_retry_delay_ms);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se encontró delay, usando valor por defecto: 1000 ms");
        config.wifi_retry_delay_ms = 1000;
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "✅ Configuración WiFi cargada de NVS:");
    ESP_LOGI(TAG, "   SSID: %s", config.wifi_ssid);
    ESP_LOGI(TAG, "   Max Retries: %d", config.wifi_max_retries);
    ESP_LOGI(TAG, "   Retry Delay: %d ms", config.wifi_retry_delay_ms);
    
    return ESP_OK;
}

static esp_err_t clear_wifi_config(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) return err;
    
    nvs_erase_all(nvs_handle);
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "🗑️ Configuración WiFi eliminada de NVS");
        // Restaurar valores por defecto
        strcpy(config.wifi_ssid, config.default_ssid);
        strcpy(config.wifi_password, config.default_password);
        config.wifi_max_retries = 5;
        config.wifi_retry_delay_ms = 1000;
    }
    return err;
}

// --- WiFi Event Handler ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi STA iniciado - Conectando a: %s", config.wifi_ssid);
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        mqtt_connected = false; // MQTT también se desconecta cuando WiFi falla
        
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGW(TAG, "WiFi desconectado. Razón: %d", event->reason);
        
        if (current_retry_count < config.wifi_max_retries) {
            current_retry_count++;
            ESP_LOGI(TAG, "🔄 Reintentando conexión (%d/%d) en %d ms...", 
                    current_retry_count, config.wifi_max_retries, config.wifi_retry_delay_ms);
            vTaskDelay(config.wifi_retry_delay_ms / portTICK_PERIOD_MS);
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "❌ Máximo de reintentos (%d) alcanzado. Cambiando a modo AP.", config.wifi_max_retries);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_connected = true;
        current_retry_count = 0;
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "✅ WiFi CONECTADO - SSID: %s, IP: " IPSTR, 
                config.wifi_ssid, IP2STR(&event->ip_info.ip));
        
        // Reiniciar MQTT cuando WiFi se reconecta
        if (mqtt_client) {
            ESP_LOGI(TAG, "🔄 Reiniciando conexión MQTT después de reconexión WiFi");
            esp_mqtt_client_stop(mqtt_client);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_mqtt_client_start(mqtt_client);
        }
    }
}

// --- MQTT Event Handler ---
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                              int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    switch (event->event_id) {
        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG, "🔄 MQTT intentando conectar a %s:%" PRIu32 "...", 
            config.mqtt_broker, config.mqtt_port);
            break;
            
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI(TAG, "✅ MQTT CONECTADO a ThingsBoard!");
            ESP_LOGI(TAG, "   Broker: %s:%" PRIu32, config.mqtt_broker, config.mqtt_port);
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGE(TAG, "❌ MQTT DESCONECTADO");
            break;
            
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "✅ ThingsBoard CONFIRMA recepción (msg_id: %d)", event->msg_id);
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "❌ ERROR MQTT: %d", event->error_handle->error_type);
            mqtt_connected = false;
            break;
            
        default:
            break;
    }
}

// --- Generar SSID del AP ---
static void generate_ap_ssid(void) {
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    snprintf(config.ap_ssid, sizeof(config.ap_ssid), 
             "ESP32_%02X%02X%02X", mac[3], mac[4], mac[5]);
}

// --- Inicializar WiFi ---
static void wifi_init(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
                       &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, 
                       &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
}

// --- Conectar como Station ---
static void wifi_connect_sta(void) {
    ESP_LOGI(TAG, "🔌 Iniciando conexión WiFi...");
    ESP_LOGI(TAG, "   SSID: %s", config.wifi_ssid);
    ESP_LOGI(TAG, "   Max Reintentos: %d", config.wifi_max_retries);
    ESP_LOGI(TAG, "   Delay Reintentos: %d ms", config.wifi_retry_delay_ms);
    
    wifi_connected = false;
    mqtt_connected = false;
    current_retry_count = 0;
    esp_wifi_stop();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, config.wifi_ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, config.wifi_password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// --- Iniciar Access Point ---
static void wifi_start_ap(void) {
    ESP_LOGI(TAG, "📡 Iniciando modo Access Point...");
    
    esp_wifi_stop();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.ap.ssid, config.ap_ssid, sizeof(wifi_config.ap.ssid) - 1);
    strncpy((char*)wifi_config.ap.password, config.ap_password, sizeof(wifi_config.ap.password) - 1);
    wifi_config.ap.ssid_len = strlen(config.ap_ssid);
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "✅ Access Point creado:");
    ESP_LOGI(TAG, "   SSID: %s", config.ap_ssid);
    ESP_LOGI(TAG, "   Password: %s", config.ap_password);
    ESP_LOGI(TAG, "   IP: 192.168.4.1");
}

// --- Web Server Handlers ---
static esp_err_t config_form_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, HTML_CONFIG_FORM, strlen(HTML_CONFIG_FORM));
    return ESP_OK;
}

static void url_decode(char *dst, const char *src) {
    char a, b;
    while (*src) {
        if ((*src == '%') && (a = src[1]) && (b = src[2]) && isxdigit(a) && isxdigit(b)) {
            if (a >= 'a') a -= 'a'-'A';
            if (a >= 'A') a -= ('A' - 10); else a -= '0';
            if (b >= 'a') b -= 'a'-'A';
            if (b >= 'A') b -= ('A' - 10); else b -= '0';
            *dst++ = 16*a+b;
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
}

static esp_err_t save_config_handler(httpd_req_t *req) {
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send(req, HTML_ERROR_PAGE, strlen(HTML_ERROR_PAGE));
        return ESP_FAIL;
    }
    
    content[ret] = '\0';
    
    // Parsear los datos del formulario
    char *ssid_start = strstr(content, "ssid=");
    char *pass_start = strstr(content, "password=");
    char *retries_start = strstr(content, "retries=");
    char *delay_start = strstr(content, "delay=");
    
    if (ssid_start && pass_start && retries_start && delay_start) {
        char ssid[32] = {0}, password[64] = {0};
        char retries_str[10] = {0}, delay_str[10] = {0};
        char decoded_ssid[32] = {0}, decoded_password[64] = {0};
        
        // Extraer SSID
        ssid_start += 5;
        char *ssid_end = strchr(ssid_start, '&');
        if (ssid_end) {
            strncpy(ssid, ssid_start, ssid_end - ssid_start);
        }
        
        // Extraer password
        pass_start += 9;
        char *pass_end = strchr(pass_start, '&');
        if (pass_end) {
            strncpy(password, pass_start, pass_end - pass_start);
        } else {
            // Si no hay &, tomar hasta el final
            strncpy(password, pass_start, sizeof(password) - 1);
        }
        
        // Extraer retries
        retries_start += 8;
        char *retries_end = strchr(retries_start, '&');
        if (retries_end) {
            strncpy(retries_str, retries_start, retries_end - retries_start);
        } else {
            strncpy(retries_str, retries_start, sizeof(retries_str) - 1);
        }
        
        // Extraer delay
        delay_start += 6;
        char *delay_end = strchr(delay_start, '&');
        if (delay_end) {
            strncpy(delay_str, delay_start, delay_end - delay_start);
        } else {
            strncpy(delay_str, delay_start, sizeof(delay_str) - 1);
        }
        
        // Decodificar URL
        url_decode(decoded_ssid, ssid);
        url_decode(decoded_password, password);
        
        // Actualizar configuración
        strncpy(config.wifi_ssid, decoded_ssid, sizeof(config.wifi_ssid) - 1);
        strncpy(config.wifi_password, decoded_password, sizeof(config.wifi_password) - 1);
        config.wifi_max_retries = atoi(retries_str);
        config.wifi_retry_delay_ms = atoi(delay_str);
        
        ESP_LOGI(TAG, "📝 Nueva configuración recibida:");
        ESP_LOGI(TAG, "   SSID: %s", config.wifi_ssid);
        ESP_LOGI(TAG, "   Max Reintentos: %d", config.wifi_max_retries);
        ESP_LOGI(TAG, "   Delay Reintentos: %d ms", config.wifi_retry_delay_ms);
        
        // Guardar en NVS (siempre se guarda ahora)
        save_wifi_config();
        
        httpd_resp_send(req, HTML_SAVE_SUCCESS, strlen(HTML_SAVE_SUCCESS));
        
        // Reiniciar después de guardar
        ESP_LOGI(TAG, "🔄 Reiniciando en 3 segundos...");
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        esp_restart();
        
    } else {
        ESP_LOGE(TAG, "❌ Error parseando datos del formulario");
        httpd_resp_send(req, HTML_ERROR_PAGE, strlen(HTML_ERROR_PAGE));
    }
    return ESP_OK;
}

static esp_err_t reset_config_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "🗑️ Solicitado reset de configuración");
    clear_wifi_config();
    httpd_resp_send(req, HTML_RESET_SUCCESS, strlen(HTML_RESET_SUCCESS));
    
    // Reiniciar después de resetear
    ESP_LOGI(TAG, "🔄 Reiniciando en 3 segundos...");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    esp_restart();
    
    return ESP_OK;
}

// --- Inicializar servidor web ---
static void start_webserver(void) {
    if (server) {
        ESP_LOGI(TAG, "Servidor web ya está ejecutándose");
        return;
    }
    
    httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
    http_config.server_port = 80;
    http_config.lru_purge_enable = true;
    http_config.max_uri_handlers = 10;
    
    if (httpd_start(&server, &http_config) == ESP_OK) {
        httpd_uri_t uri_get = { .uri = "/", .method = HTTP_GET, .handler = config_form_handler };
        httpd_uri_t uri_save = { .uri = "/save", .method = HTTP_POST, .handler = save_config_handler };
        httpd_uri_t uri_reset = { .uri = "/reset", .method = HTTP_POST, .handler = reset_config_handler };
        
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_save);
        httpd_register_uri_handler(server, &uri_reset);
        
        ESP_LOGI(TAG, "🌐 Servidor web iniciado: http://192.168.4.1");
    } else {
        ESP_LOGE(TAG, "❌ Error iniciando servidor web");
    }
}

static void stop_webserver(void) {
    if (server) {
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "Servidor web detenido");
    }
}

// --- Funciones OLED (simplificadas) ---
uint8_t u8g2_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    static uint8_t buffer[32];
    static uint8_t buf_idx;
    uint8_t *data;
    
    switch(msg) {
        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;
            while(arg_int--) buffer[buf_idx++] = *data++;
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_idx = 0;
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (config.oled_address << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write(cmd, buffer, buf_idx, true);
            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
            i2c_cmd_link_delete(cmd);
            break;
    }
    return 1;
}

uint8_t u8g2_gpio_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch(msg) {
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(pdMS_TO_TICKS(arg_int));
            break;
    }
    return 1;
}

static esp_err_t init_oled(void) {
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config.i2c_sda_pin,
        .scl_io_num = config.i2c_scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0));
    
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_i2c_byte_cb, u8g2_gpio_delay_cb);
    u8g2_SetI2CAddress(&u8g2, config.oled_address << 1);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
    u8g2_DrawStr(&u8g2, 10, 30, "Iniciando...");
    u8g2_SendBuffer(&u8g2);
    
    ESP_LOGI(TAG, "✅ OLED inicializado");
    return ESP_OK;
}

static void display_dht11_data(void) {
    char temp_str[20], hum_str[20];
    
    u8g2_ClearBuffer(&u8g2);
    
    snprintf(temp_str, sizeof(temp_str), "Temp: %.1fC", config.temperature);
    snprintf(hum_str, sizeof(hum_str), "Hum:  %.1f%%", config.humidity);
    
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&u8g2, 10, 20, temp_str);
    u8g2_DrawStr(&u8g2, 10, 40, hum_str);
    
    if (wifi_connected) {
        if (mqtt_connected) {
            u8g2_DrawStr(&u8g2, 10, 60, "TB: CONECTADO");
        } else {
            u8g2_DrawStr(&u8g2, 10, 60, "TB: DESCONECTADO");
        }
    } else {
        u8g2_DrawStr(&u8g2, 10, 60, "Modo AP");
    }
    
    u8g2_SendBuffer(&u8g2);
}

// --- Funciones DHT11 (implementación REAL) ---
static bool read_dht11_data(void) {
    uint8_t data[5] = {0};
    
    // Configurar GPIO como salida
    gpio_set_direction(config.dht_gpio, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(config.dht_gpio, GPIO_FLOATING);
    
    // Señal de inicio - 18ms LOW
    gpio_set_level(config.dht_gpio, 0);
    esp_rom_delay_us(18000);
    
    // 20-40us HIGH
    gpio_set_level(config.dht_gpio, 1);
    esp_rom_delay_us(40);
    
    // Cambiar a entrada
    gpio_set_direction(config.dht_gpio, GPIO_MODE_INPUT);
    
    // Esperar respuesta del sensor
    int timeout = 10000;
    while (gpio_get_level(config.dht_gpio) == 1) {
        if (timeout-- <= 0) {
            ESP_LOGE(TAG, "❌ Timeout esperando respuesta DHT11");
            return false;
        }
        esp_rom_delay_us(1);
    }
    
    timeout = 10000;
    while (gpio_get_level(config.dht_gpio) == 0) {
        if (timeout-- <= 0) {
            ESP_LOGE(TAG, "❌ Timeout en pulso bajo inicial");
            return false;
        }
        esp_rom_delay_us(1);
    }
    
    timeout = 10000;
    while (gpio_get_level(config.dht_gpio) == 1) {
        if (timeout-- <= 0) {
            ESP_LOGE(TAG, "❌ Timeout en pulso alto inicial");
            return false;
        }
        esp_rom_delay_us(1);
    }
    
    // Leer 40 bits de datos
    for (int i = 0; i < 40; i++) {
        // Esperar pulso bajo
        timeout = 10000;
        while (gpio_get_level(config.dht_gpio) == 0) {
            if (timeout-- <= 0) {
                ESP_LOGE(TAG, "❌ Timeout en bit %d (bajo)", i);
                return false;
            }
            esp_rom_delay_us(1);
        }
        
        // Medir duración del pulso alto
        uint32_t high_time = 0;
        timeout = 10000;
        while (gpio_get_level(config.dht_gpio) == 1 && high_time < 100) {
            high_time++;
            esp_rom_delay_us(1);
        }
        
        // Bit 0: 26-28us, Bit 1: 70us
        data[i / 8] <<= 1;
        if (high_time > 30) {  // Si el pulso alto > 30us, es un 1
            data[i / 8] |= 1;
        }
    }
    
    // Verificar checksum
    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        // DHT11: humidity entero, temperature entero
        config.humidity = data[0];
        config.temperature = data[2];
        
        ESP_LOGI(TAG, "📊 DHT11 - Temp: %.1f°C, Hum: %.1f%%", 
                config.temperature, config.humidity);
        return true;
    } else {
        ESP_LOGE(TAG, "❌ Checksum error DHT11");
        return false;
    }
}

// --- Crear JSON para telemetría (usando cJSON) ---
static char* create_telemetry_json(void) {
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "❌ Error creando objeto JSON");
        return NULL;
    }
    
    // Agregar datos de telemetría según formato ThingsBoard
    cJSON_AddNumberToObject(root, "temperature", config.temperature);
    cJSON_AddNumberToObject(root, "humidity", config.humidity);
    
    // Convertir a string
    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    
    return json_string;
}

// --- Funciones MQTT para ThingsBoard ---
static esp_err_t init_mqtt(void) {
    ESP_LOGI(TAG, "🔌 Inicializando MQTT para ThingsBoard...");
    ESP_LOGI(TAG, "   Broker: %s:%" PRIu32, config.mqtt_broker, config.mqtt_port);
    ESP_LOGI(TAG, "   Token: %s", config.mqtt_token);
    ESP_LOGI(TAG, "   Topic: %s", config.mqtt_telemetry_topic);
    
    // Construir URI MQTT completa - CORREGIDO
    char mqtt_uri[100];
    snprintf(mqtt_uri, sizeof(mqtt_uri), "mqtt://%s:%" PRIu32, config.mqtt_broker, config.mqtt_port);
    
    // El resto del código permanece igual...
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = mqtt_uri,
        .credentials.username = config.mqtt_token,
        .session.keepalive = 60,
        .network.disable_auto_reconnect = false,
        .network.reconnect_timeout_ms = 5000,
        .task.stack_size = 6144,
        .buffer.size = 2048,
    };
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "❌ Error creando cliente MQTT");
        return ESP_FAIL;
    }
    
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_err_t ret = esp_mqtt_client_start(mqtt_client);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error iniciando MQTT: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "✅ Cliente MQTT iniciado, esperando conexión...");
    return ESP_OK;
}

static void send_dht11_data(void) {
    ESP_LOGI(TAG, "🔍 DIAGNÓSTICO - mqtt_connected: %d, wifi_connected: %d", 
             mqtt_connected, wifi_connected);
    
    if (!mqtt_connected) {
        ESP_LOGE(TAG, "❌ NO se pueden enviar datos - MQTT desconectado");
        return;
    }
    
    if (!wifi_connected) {
        ESP_LOGE(TAG, "❌ NO se pueden enviar datos - WiFi desconectado");
        return;
    }
    
    // Crear payload JSON usando cJSON
    char *payload = create_telemetry_json();
    if (payload == NULL) {
        ESP_LOGE(TAG, "❌ Error creando payload JSON");
        return;
    }
    
    ESP_LOGI(TAG, "📤 ENVIANDO a ThingsBoard:");
    ESP_LOGI(TAG, "   Topic: %s", config.mqtt_telemetry_topic);
    ESP_LOGI(TAG, "   Payload: %s", payload);
    
    // Enviar telemetría a ThingsBoard
    int msg_id = esp_mqtt_client_publish(mqtt_client, 
                                        config.mqtt_telemetry_topic,
                                        payload, 
                                        0,  // QoS 0 (ThingsBoard recomienda QoS 1 para mensajes críticos)
                                        0,  // No retain
                                        10000); // Timeout 10 segundos
    
    // Liberar memoria del JSON
    cJSON_free(payload);
    
    if (msg_id == -1) {
        ESP_LOGE(TAG, "❌ ERROR CRÍTICO: No se pudo publicar mensaje MQTT");
        ESP_LOGE(TAG, "   - Posible falta de memoria");
        ESP_LOGE(TAG, "   - O conexión MQTT perdida");
    } else {
        ESP_LOGI(TAG, "✅ TELEMETRÍA ENVIADA - msg_id: %d", msg_id);
        ESP_LOGI(TAG, "✅ Temp: %.1f°C, Hum: %.1f%%", 
                config.temperature, config.humidity);
    }
}

// --- Funciones auxiliares ---
static bool wait_for_connection(bool check_wifi, bool check_mqtt, int timeout_ms) {
    int waited = 0;
    const int interval = 100;
    
    ESP_LOGI(TAG, "⏳ Esperando conexión...");
    
    while (waited < timeout_ms) {
        bool connected = true;
        if (check_wifi && !wifi_connected) connected = false;
        if (check_mqtt && !mqtt_connected) connected = false;
        
        if (connected) {
            ESP_LOGI(TAG, "✅ Conexión establecida después de %d ms", waited);
            return true;
        }
        
        vTaskDelay(interval / portTICK_PERIOD_MS);
        waited += interval;
    }
    
    ESP_LOGE(TAG, "❌ Timeout de conexión después de %d ms", timeout_ms);
    return false;
}

// --- Bucle principal de aplicación ---
static void main_application_loop(void) {
    ESP_LOGI(TAG, "🚀 Iniciando aplicación principal DHT11 + ThingsBoard");
    
    // Inicializar MQTT para ThingsBoard
    if (init_mqtt() != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error inicializando MQTT, modo local solamente");
    }
    
    // Esperar conexión MQTT con más tiempo
    ESP_LOGI(TAG, "⏳ Esperando conexión MQTT a ThingsBoard (máximo 30 segundos)...");
    int mqtt_timeout = 0;
    while (!mqtt_connected && mqtt_timeout < 30000) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        mqtt_timeout += 1000;
        if (mqtt_timeout % 5000 == 0) {
            ESP_LOGI(TAG, "⏳ Esperando MQTT... %d/30000 ms", mqtt_timeout);
        }
    }
    
    if (!mqtt_connected) {
        ESP_LOGW(TAG, "⚠️ MQTT no conectado después de 30 segundos");
        ESP_LOGW(TAG, "⚠️ Continuando en modo local sin ThingsBoard");
    } else {
        ESP_LOGI(TAG, "🎉 MQTT CONECTADO - Iniciando envío de telemetría a ThingsBoard");
    }
    
    // Bucle principal
    int cycle_count = 0;
    while (wifi_connected) {
        cycle_count++;
        ESP_LOGI(TAG, "--- CICLO %d ---", cycle_count);
        
        if (read_dht11_data()) {
            display_dht11_data();
            
            // Solo enviar si MQTT está conectado
            if (mqtt_connected) {
                send_dht11_data();
            } else {
                ESP_LOGW(TAG, "📊 Datos locales - Temp: %.1f°C, Hum: %.1f%%", 
                        config.temperature, config.humidity);
                
                // Intentar reconectar MQTT si está desconectado
                if (wifi_connected && !mqtt_connected) {
                    ESP_LOGI(TAG, "🔄 Intentando reconexión MQTT...");
                    esp_mqtt_client_reconnect(mqtt_client);
                }
            }
        } else {
            ESP_LOGE(TAG, "❌ Error leyendo sensor DHT11");
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5 segundos entre lecturas
    }
    
    ESP_LOGI(TAG, "🔁 Saliendo del bucle - WiFi desconectado");
}

// --- Función principal ---
void app_main(void) {
    ESP_LOGI(TAG, "🔧 Inicializando sistema ESP32 DHT11 + ThingsBoard...");
    
    // 1. Inicialización de sistemas básicos
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    
    // 2. Inicializar hardware
    init_oled();
    wifi_init();
    generate_ap_ssid();
    
    // 3. Cargar configuración WiFi de NVS
    if (load_wifi_config() != ESP_OK) {
        ESP_LOGW(TAG, "Usando configuración WiFi por defecto");
        strcpy(config.wifi_ssid, config.default_ssid);
        strcpy(config.wifi_password, config.default_password);
    }
    
    // 4. Mostrar información inicial en OLED
    display_dht11_data();
    u8g2_DrawStr(&u8g2, 10, 50, "Conectando...");
    u8g2_SendBuffer(&u8g2);
    
    // 5. Intentar conexión WiFi con configuración cargada
    ESP_LOGI(TAG, "📶 Intentando conexión WiFi con configuración guardada...");
    wifi_connect_sta();
    
    // 6. Esperar conexión WiFi
    if (wait_for_connection(true, false, 15000)) {
        // Modo normal: WiFi conectado
        ESP_LOGI(TAG, "🎉 MODO NORMAL - Conectado a WiFi");
        display_dht11_data();
        u8g2_DrawStr(&u8g2, 10, 50, "WiFi OK!");
        u8g2_SendBuffer(&u8g2);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        
        main_application_loop();
    }
    
    // 7. Modo configuración AP (si falla la conexión WiFi)
    ESP_LOGI(TAG, "🔁 Cambiando a MODO CONFIGURACIÓN AP");
    wifi_start_ap();
    start_webserver();
    
    // Mostrar estado en OLED
    display_dht11_data();
    u8g2_DrawStr(&u8g2, 10, 50, "Modo AP");
    u8g2_DrawStr(&u8g2, 10, 60, "192.168.4.1");
    u8g2_SendBuffer(&u8g2);
    
    ESP_LOGI(TAG, "🌐 Portal web disponible en: http://192.168.4.1");
    ESP_LOGI(TAG, "📱 Conéctate al WiFi: %s", config.ap_ssid);
    ESP_LOGI(TAG, "🔑 Password: %s", config.ap_password);
    
    // 8. Bucle principal del modo AP
    while (1) {
        // Si por algún motivo se conecta a WiFi, cambiar a modo normal
        if (wifi_connected) {
            ESP_LOGI(TAG, "🔀 Cambiando a MODO NORMAL (WiFi conectado)");
            stop_webserver();
            main_application_loop();
            
            // Si salimos del bucle de aplicación, volver al modo AP
            ESP_LOGI(TAG, "🔁 Volviendo a MODO CONFIGURACIÓN AP");
            wifi_start_ap();
            start_webserver();
        }
        
        // Leer y mostrar datos del sensor en modo AP
        if (read_dht11_data()) {
            display_dht11_data();
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Actualizar cada 5 segundos en modo AP
    }
}