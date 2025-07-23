#include <stdio.h>

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/cyw43_arch.h" 
#include "hardware/adc.h"
#include "lwip/tcp.h"

#include "lib/ssd1306/ssd1306.h"
#include "lib/ssd1306/display.h"
#include "lib/led/led.h"
#include "lib/button/button.h"
#include "lib/matrix_leds/neopixel.h"
#include "lib/buzzer/buzzer.h"
#include "lib/sensors/aht20.h"
#include "lib/sensors/bmp280.h"
#include "config/wifi_config.h"
#include "public/html_data.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Constantes de configuração do I2C para sensores
#define I2C_PORT_SENSORS i2c0    // Sensores AHT20 e BMP280
#define I2C_SDA_SENSORS_PIN    0
#define I2C_SCL_SENSORS_PIN    1

// Pinos joystick
#define VRX_PIN 27  
#define VRY_PIN 26

#define SEA_LEVEL_PRESSURE 1021.0f // Pressão atmosférica ao nível do mar em hPa

// Estrutura para armazenar os dados dos sensores
typedef struct {
    float temperature;
    float pressure;
    float altitude;
    float humidity;
    float max_temperature; // Temperatura máxima registrada
    float min_temperature; // Temperatura mínima registrada
    float max_pressure;    // Pressão máxima registrada
    float min_pressure;    // Pressão mínima registrada
    float max_humidity;     // Umidade máxima registrada
    float min_humidity;     // Umidade mínima registrada
    float offset_temperature; // Offset de temperatura para compensação
} SensorData_t;

// Handles do FreeRTOS
static SemaphoreHandle_t xMutexSensorData;
static SemaphoreHandle_t xMutexDisplay;
SemaphoreHandle_t xWifiReadySemaphore; // Novo semáforo para sinalizar que o Wi-Fi está pronto


// Inicialização padrão dos limites climáticos
static volatile SensorData_t xSensorData = {
    .temperature = 0.0f,
    .pressure = 0.0f,
    .altitude = 0.0f,
    .humidity = 0.0f,
    .max_temperature = 60.0f,
    .min_temperature = 0.0f,
    .max_pressure = 1050.0f,
    .min_pressure = 950.0f,
    .max_humidity = 100.0f,
    .min_humidity = 20.0f,
    .offset_temperature = 0.0f
};

// Variável global para o display SSD1306
static ssd1306_t ssd;

// Variáveis globais para debounce dos botões
volatile uint32_t last_time_debounce_button_a = 0;
volatile uint32_t last_time_debounce_button_sw = 0;
const uint32_t delay_debounce = 200; // 200 ms

// Variaveis do programa
volatile bool reset_limits= false;  // Variável para reiniciar os limites
volatile bool modo_simulacao = false; // Variável para modo de simulação

// Estrutura de dados
struct http_state
{
    char response[10000];
    size_t len;
    size_t sent;
};

// Prototipos das funções
void button_callback(uint gpio, uint32_t events);
static float calculate_altitude(float pressure);
void vSensorTask(void *pvParameters);
void vDisplayTask(void *pvParameters);
void vWebServerTask(void *pvParameters);
void vMatrizLedsTask(void *pvParameters);
void vLedsBuzzerTask(void *pvParameters);

// Funções auxiliares do Web Server
static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err);
static void start_http_server(void);


int main()
{
    stdio_init_all();

    // Inicia o Display SSD1306 pois 
    init_display(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);   // Envia os dados para o display

    
    // Inicializa Botões    
    button_init_predefined(true, true, true);
    button_setup_irq(true,true,true,button_callback);

    // Cria mutexes
    xMutexSensorData = xSemaphoreCreateMutex();
    xMutexDisplay = xSemaphoreCreateMutex();

    xWifiReadySemaphore = xSemaphoreCreateBinary(); // Cria um semáforo binário


    // Cria tasks
    xTaskCreate(vSensorTask,  "SensorTask",  configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vDisplayTask, "DisplayTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vWebServerTask, "WebServerTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vMatrizLedsTask, "MatrizLedsTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vLedsBuzzerTask, "LedsBuzzerTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

    vTaskStartScheduler();
    panic_unsupported();

}

// Callback para os botões
void button_callback(uint gpio, uint32_t events){
    uint32_t current_time = to_ms_since_boot(get_absolute_time());


        if(gpio == BUTTON_A && (current_time - last_time_debounce_button_a > delay_debounce)){
            printf("\nBOTAO A PRESSIONADO\n");

            // Reinicia os limites
            reset_limits = true;

            last_time_debounce_button_a = current_time; // Atualiza o tempo do último debounce do botão A
        }
        else if(gpio == BUTTON_SW && (current_time - last_time_debounce_button_sw > delay_debounce)){
            printf("\nBOTAO SW PRESSIONADO\n");

            // Alterna entre modo de simulação e normal
            modo_simulacao = !modo_simulacao;

            last_time_debounce_button_sw = current_time; // Atualiza o tempo do último debounce do botão SW
        }
        else if(gpio == BUTTON_B){// Habilita o modo de gravação 
            printf("\nHABILITANDO O MODO GRAVAÇÃO\n");

            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "  HABILITANDO", 5, 25);
            ssd1306_draw_string(&ssd, " MODO GRAVACAO", 5, 38);
            ssd1306_send_data(&ssd);

            reset_usb_boot(0, 0);
        }
}

// Função para calcular altitude a partir da pressão
static float calculate_altitude(float pressure) {
    return 44330.0f * (1.0f - powf(pressure / SEA_LEVEL_PRESSURE, 0.1903f));
}

// Task de leitura dos sensores
void vSensorTask(void *pvParameters) {
    (void) pvParameters;

    // Inicializa I2C para sensores
    i2c_init(I2C_PORT_SENSORS, 400 * 1000); // 400 kHz
    gpio_set_function(I2C_SDA_SENSORS_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_SENSORS_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_SENSORS_PIN);
    gpio_pull_up(I2C_SCL_SENSORS_PIN);

    // Inicializa BMP280
    bmp280_init(I2C_PORT_SENSORS);
    struct bmp280_calib_param bmp_params;
    bmp280_get_calib_params(I2C_PORT_SENSORS, &bmp_params);

    // Inicializa AHT20
    aht20_reset(I2C_PORT_SENSORS);
    aht20_init(I2C_PORT_SENSORS);

    //Inicializa joysticks para modo de simulação
    adc_init();
    adc_gpio_init(VRY_PIN);  // Eixo Y
    adc_gpio_init(VRX_PIN);  // Eixo X

    //Variaveis para armazenar os dados dos sensores
    int32_t raw_temp_bmp, raw_pressure;
    AHT20_Data aht_data;
    float temp_bmp = 0.0f;
    float pressure = 0.0f; // Pressão em hPa
    float altitude = 0.0f; // Altitude em metros

    xSemaphoreTake(xWifiReadySemaphore, portMAX_DELAY);
    xSemaphoreGive(xWifiReadySemaphore); // Dá o semáforo de volta para que outras tasks também possam usá-lo

    while (true) {

        if(reset_limits) {
            // Reseta os limites na estrutura principal
            if (xSemaphoreTake(xMutexSensorData, portMAX_DELAY) == pdTRUE) {
                xSensorData.max_temperature = 60.0f;
                xSensorData.min_temperature = 0.0f;
                xSensorData.max_pressure = 1050.0f;
                xSensorData.min_pressure = 950.0f;
                xSensorData.max_humidity = 100.0f;
                xSensorData.min_humidity = 20.0f;
                xSensorData.offset_temperature = 0.0f;                 
                reset_limits = false; // Reseta a flag
                xSemaphoreGive(xMutexSensorData);
            }
        }

        if(modo_simulacao) {
            // Modo de simulação: lê valores dos joysticks
            adc_select_input(1); // Seleciona o canal para o eixo X
            //temp_bmp = adc_read(); // Simula temperatura BMP280
            aht_data.temperature = adc_read() / 4095.0f * (150.0f + 50.0f) - 50.0f; // Simula temperatura AHT20

            adc_select_input(0); // Seleciona o canal para o eixo Y
            pressure = (adc_read() / 4095.0f) * (1500.0f - 600.0f) + 600.0f;
            aht_data.humidity = adc_read() / 4095.0f * 100; // Simula umidade AHT20

            altitude = calculate_altitude(pressure); // Calcula altitude com a pressão simulada
        } else {
            // Modo normal: lê sensores reais
            // Leitura BMP280
            bmp280_read_raw(I2C_PORT_SENSORS, &raw_temp_bmp, &raw_pressure);
            temp_bmp = bmp280_convert_temp(raw_temp_bmp, &bmp_params) / 100.0f;
            pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &bmp_params) / 100.0f; // Converte Pa para hPa
            altitude = calculate_altitude(pressure); 

            // Leitura AHT20
            if (!aht20_read(I2C_PORT_SENSORS, &aht_data)) {
                // erro, define zeros
                aht_data.temperature = 0.0f;
                aht_data.humidity = 0.0f;
                printf("Erro na leitura do AHT20!\n\n\n");
            }
        }

        // Protege acesso aos dados compartilhados
        if (xSemaphoreTake(xMutexSensorData, portMAX_DELAY) == pdTRUE) {
            xSensorData.temperature = aht_data.temperature + xSensorData.offset_temperature; // Temperatura AHT20
            xSensorData.pressure = pressure;
            xSensorData.altitude = altitude;
            xSensorData.humidity = aht_data.humidity;
            xSemaphoreGive(xMutexSensorData);

            printf("Temp: %.2f C, Humidity: %.2f %%, Pressure: %.2f hPa, Altitude: %.2f m\n",
                   xSensorData.temperature,
                   xSensorData.humidity,
                   xSensorData.pressure, 
                   xSensorData.altitude);
            printf("Max Temp: %.2f C, Min Temp: %.2f C, Offset: %.2f C\n",
                   xSensorData.max_temperature,
                   xSensorData.min_temperature,
                   xSensorData.offset_temperature);
            printf("Max Humidity: %.2f %%, Min Humidity: %.2f %%\n",
                   xSensorData.max_humidity,
                   xSensorData.min_humidity);
            printf("Max Pressure: %.2f hPa, Min Pressure: %.2f hPa\n",
                   xSensorData.max_pressure,
                   xSensorData.min_pressure);
            
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Task de exibição no display
void vDisplayTask(void *pvParameters) {
    (void) pvParameters;

    // Buffers com nomes mais descritivos para exibição no display
    char tempLine[30];
    char altitudeLine[30];
    char humidityLine[30];
    char pressureLine[30];

    xSemaphoreTake(xWifiReadySemaphore, portMAX_DELAY);
    xSemaphoreGive(xWifiReadySemaphore); // Libera para outras tasks

    while (true) {
        SensorData_t data;

        // Lê dados protegidos por mutex
        if (xSemaphoreTake(xMutexSensorData, portMAX_DELAY) == pdTRUE) {
            data = xSensorData;
            xSemaphoreGive(xMutexSensorData);
        }

        // Formata os dados em strings para o display
        snprintf(tempLine, sizeof(tempLine),    "Temp:%.1f C", data.temperature);
        snprintf(altitudeLine, sizeof(altitudeLine), "Alt:%.2f m", data.altitude);
        snprintf(humidityLine, sizeof(humidityLine), "Umidade:%.1f %%", data.humidity);
        snprintf(pressureLine, sizeof(pressureLine), "Pressao:%.1f hPa", data.pressure); 

        // Atualiza o display com as informações
        if (xSemaphoreTake(xMutexDisplay, portMAX_DELAY) == pdTRUE) {
            ssd1306_fill(&ssd, false);  // Limpa o display

            // Cabeçalho centralizado
            draw_centered_text(&ssd, "Estacao Meteor.", 0); 

            // Dados organizados verticalmente
            ssd1306_draw_string(&ssd, tempLine,0, 12);
            ssd1306_draw_string(&ssd, altitudeLine, 0, 22);
            ssd1306_draw_string(&ssd, humidityLine, 0, 38);
            ssd1306_draw_string(&ssd, pressureLine, 0, 48);

            ssd1306_send_data(&ssd);
            xSemaphoreGive(xMutexDisplay);
        }

        vTaskDelay(pdMS_TO_TICKS(500));  // Atualiza a cada 500ms
    }
}

// Task para LEDs e buzzer
void vLedsBuzzerTask(void *pvParameters) {
    (void) pvParameters;

    init_buzzer(BUZZER_A_PIN, 4.0f);
    init_leds();

    xSemaphoreTake(xWifiReadySemaphore, portMAX_DELAY);
    xSemaphoreGive(xWifiReadySemaphore); // Libera para outras tasks

    // Adiciona um atraso para os sensores estabilizarem
    vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 5 segundos antes de começar os alertas

    while (true) {
        SensorData_t data;

        if (xSemaphoreTake(xMutexSensorData, portMAX_DELAY) == pdTRUE) {
            data = xSensorData;
            xSemaphoreGive(xMutexSensorData);
        }

         // Verifica os limites
        bool alerta_temp = data.temperature < data.min_temperature || data.temperature > data.max_temperature;
        bool alerta_umid = data.humidity < data.min_humidity || data.humidity > data.max_humidity;
        bool alerta_press = data.pressure < data.min_pressure || data.pressure > data.max_pressure;

        // Define cor do LED conforme o tipo de alerta
        if (alerta_temp && alerta_umid && alerta_press) {
            turn_on_leds(); // Todos fora do limite
        } else if (alerta_temp && alerta_press) {
            set_led_yellow(); // Temperatura e pressão fora do limite
        } else if (alerta_umid && alerta_press) {
            set_led_cyan();   // Umidade e pressão fora do limite
        } else if (alerta_temp && alerta_umid) {
            set_led_magenta(); // Temperatura e umidade fora do limite
        } else if (alerta_temp) {
            set_led_red();    // Só temperatura fora
        } else if (alerta_umid) {
            set_led_blue();   // Só umidade fora
        } else if (alerta_press) {
            set_led_green();  // Pressão fora
        } else {
            turn_off_leds();  // Tudo normal
        }  

        // Se qualquer alerta, toque o buzzer
        if (alerta_temp || alerta_umid || alerta_press) {
            play_tone(BUZZER_A_PIN, 300);
            vTaskDelay(pdMS_TO_TICKS(250));
            stop_tone(BUZZER_A_PIN);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task Matriz de Leds
void vMatrizLedsTask(void *pvParameters) {
    (void) pvParameters;

    npInit(LED_PIN);

    SensorData_t data;

    while (true) {
        // Lê dados protegidos por mutex
        if (xSemaphoreTake(xMutexSensorData, portMAX_DELAY) == pdTRUE) {
            data = xSensorData; // copia struct com segurança
            xSemaphoreGive(xMutexSensorData);
        }

        npClear();  // Limpa todos os LEDs

        // --- Temperatura (colunas 0 e 1) ---
        float tempMin = -20.0f;
        float tempMax = 60.0f;
        int maxAltura = 5; // altura máxima da coluna (5 LEDs)

        // Normaliza e calcula altura proporcional
        int alturaTemp = (int)(((data.temperature - tempMin) / (tempMax - tempMin)) * maxAltura);
        if (alturaTemp > maxAltura) alturaTemp = maxAltura;
        if (alturaTemp < 0) alturaTemp = 0;

        npFillCollumns(0, alturaTemp, 255, 0, 0);  // Coluna 0 vermelha
        npFillCollumns(1, alturaTemp, 255, 0, 0);  // Coluna 1 vermelha

        // --- Umidade (colunas 2 e 3) ---
        int alturaUmid = (int)((data.humidity / 100.0f) * maxAltura);
        if (alturaUmid > maxAltura) alturaUmid = maxAltura;
        if (alturaUmid < 0) alturaUmid = 0;

        npFillCollumns(2, alturaUmid, 0, 0, 255);  // Coluna 2 azul
        npFillCollumns(3, alturaUmid, 0, 0, 255);  // Coluna 3 azul

        // --- Pressão (coluna 4) ---
        float pressMin = 800.0f;
        float pressMax = 1200.0f;
        // Ajusta pressão para hPa se necessário (ex: se data.pressure em Pa)
        float pressure_hPa = data.pressure;

        int alturaPressao = (int)(((pressure_hPa - pressMin) / (pressMax - pressMin)) * maxAltura);
        if (alturaPressao > maxAltura) alturaPressao = maxAltura;
        if (alturaPressao < 0) alturaPressao = 0;

        npFillCollumns(4, alturaPressao, 0, 255, 0);  // Coluna 4 verde

        npWrite();  // Atualiza a matriz

        vTaskDelay(pdMS_TO_TICKS(1000));  // Atualiza a cada 1s
    }
}


// Task que inicia o servidor web
void vWebServerTask(void *pvParameters){
    (void)pvParameters; // Evita aviso de parâmetro não utilizado


    if (xSemaphoreTake(xMutexDisplay,portMAX_DELAY) == pdTRUE){

        // Inicializa o display para mostrar o status do Wi-Fi
        draw_centered_text(&ssd, "Conectando", 24); // Cabeçalho centralizado
        draw_centered_text(&ssd, "ao WI-FI", 34); // Cabeçalho centralizado

        ssd1306_send_data(&ssd);

        // Inicializa a biblioteca CYW43 para Wi-Fi
        if (cyw43_arch_init())
        {
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "WiFi => FALHA", 0, 0);
            ssd1306_send_data(&ssd);

            vTaskDelete(NULL);  // Encerrar esta task
        }

        cyw43_arch_enable_sta_mode();
        if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
        {
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "WiFi => ERRO", 0, 0);
            ssd1306_send_data(&ssd);

            vTaskDelete(NULL);  // Encerrar esta task
        }

        uint8_t *ip = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
        char ip_str[24];
        snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
        printf("IP: %s\n",ip_str);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "WiFi => OK", 0, 0);
        ssd1306_draw_string(&ssd, ip_str, 0, 10);
        ssd1306_send_data(&ssd);
        start_http_server();
        vTaskDelay(pdMS_TO_TICKS(5000)); // pra dar tempo de ver o ip
        xSemaphoreGive(xMutexDisplay);
        xSemaphoreGive(xWifiReadySemaphore); // Sinaliza que o Wi-Fi está pronto!
    }


    while (1){
        // Mantém o servidor HTTP ativo
        cyw43_arch_poll();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Aguarda 1 segundo
    }
     cyw43_arch_deinit();// Esperamos que nunca chegue aqui
}

// Função de recebimento HTTP
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (!p) {
        tcp_close(tpcb);
        return ERR_OK;
    }

    char *req = (char *)p->payload;
    struct http_state *hs = malloc(sizeof(struct http_state));
    if (!hs) {
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_MEM;
    }
    hs->sent = 0;


    printf("Requisição:\n %s\n", req);
    // --- GET /estado ---
    if (strstr(req, "GET /estado")) {
        SensorData_t data;
        data = xSensorData;

        char json_payload[256];
        int json_len = snprintf(json_payload, sizeof(json_payload),
            "{"
            "\"temperature\":%.2f,"
            "\"humidity\":%.2f,"
            "\"pressure\":%.2f,"
            "\"altitude\":%.2f,"
            "\"max_temperature\":%.2f,"
            "\"min_temperature\":%.2f,"
            "\"max_pressure\":%.2f,"
            "\"min_pressure\":%.2f,"
            "\"max_humidity\":%.2f,"
            "\"min_humidity\":%.2f,"
            "\"offset_temperature\":%.2f"
            "}",
            data.temperature,
            data.humidity,
            data.pressure,
            data.altitude,
            data.max_temperature,
            data.min_temperature,
            data.max_pressure,
            data.min_pressure,
            data.max_humidity,
            data.min_humidity,
            data.offset_temperature
        );

        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s",
            json_len, json_payload
        );
    }
    // --- POST /limites ---
    else if (strstr(req, "POST /limites")) {
        char *body = strstr(req, "\r\n\r\n");
        if (body) {
            body += 4;
            float min_t, max_t, offset_t;
            float min_h, max_h;
            float min_p, max_p;

            if (sscanf(body,
                "{"
                "\"min_temperature\":%f,"
                "\"max_temperature\":%f,"
                "\"offset_temperature\":%f,"
                "\"min_humidity\":%f,"
                "\"max_humidity\":%f,"
                "\"min_pressure\":%f,"
                "\"max_pressure\":%f"
                "}",
                &min_t, &max_t, &offset_t,
                &min_h, &max_h,
                &min_p, &max_p) == 7)
            {
                xSensorData.min_temperature = min_t;
                xSensorData.max_temperature = max_t;
                xSensorData.offset_temperature = offset_t;

                xSensorData.min_humidity = min_h;
                xSensorData.max_humidity = max_h;
                xSensorData.min_pressure = min_p;
                xSensorData.max_pressure = max_p;
            }

            const char *txt = "Limites e offset atualizados";
            hs->len = snprintf(hs->response, sizeof(hs->response),
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/plain\r\n"
                "Access-Control-Allow-Origin: *\r\n"
                "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
                "Content-Length: %d\r\n"
                "Connection: close\r\n"
                "\r\n"
                "%s",
                (int)strlen(txt), txt
            );
        }
    }
    // --- fallback: página HTML ---
    else {
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s",
            (int)strlen(html_data), html_data
        );
    }

    tcp_arg(tpcb, hs);
    tcp_sent(tpcb, http_sent);
    tcp_write(tpcb, hs->response, hs->len, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    printf("Response: %s\n", hs->response);

    pbuf_free(p);
    return ERR_OK;
} 



// Função de callback para enviar dados HTTP
static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    struct http_state *hs = (struct http_state *)arg;
    hs->sent += len;
    if (hs->sent >= hs->len)
    {
        tcp_close(tpcb);
        free(hs);
    }
    return ERR_OK;
}

// Função de callback para aceitar conexões TCP
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    tcp_recv(newpcb, http_recv);
    return ERR_OK;
}

// Função para iniciar o servidor HTTP
static void start_http_server(void)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb)
    {
        printf("Erro ao criar PCB TCP\n");
        return;
    }
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK)
    {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
    printf("Servidor HTTP rodando na porta 80...\n");
}