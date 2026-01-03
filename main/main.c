#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "ssd1306.h"
#include "bmp3.h"
#include "bmp3_defs.h"
#include "esp_rom_sys.h"



static const char *TAG = "TimerProjetc";

/* ============================
   CONFIGURAÇÕES DE HARDWARE
   ============================ */

// I2C
#define I2C_BUS_NUM 0
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

//Timer

static TimerHandle_t timer_1s = NULL; //Manipular o Timer
static TaskHandle_t sensor_task_handle = NULL; //Manipular task sensor

/*
static TaskHandle_t exibir_task_handle = NULL; //Manipular task exibir

Esse Handle acima foi muito útil para manipular a task exibir_task no inicio do projeto que servia como gatilho
para o timer, mas a medida que o projeto foi crescendo, quem faz o papel de gatilho para acionar a exibir_task é o sensor_task
por isso retirado, mas é importante deixar para conhecimento
Sempre um sistema em cadeia, timer aciona o sensor e o sensor chama o display, sem criar handle extra
*/



static QueueHandle_t bmp388_queue = NULL; //Queue Global


/* ============================
   DISPLAY (HANDLE GLOBAL)
   ============================ */

ssd1306_handle_t oled_handle = NULL; 


/* ============================
   CONFIGURAÇÃO BMP388
   ============================ */


static struct bmp3_dev bmp388; //"oBjeto do sensor"
static i2c_master_bus_handle_t i2c_bus = NULL;  //Handle que manipula o i2c para a API
static i2c_master_dev_handle_t bmp388_i2c = NULL; //Handle do sensor dentro do barramento


// Função de escrita para API 
int8_t bmp388_i2c_write(uint8_t reg_addr, //valor do registrador a ser escrito
                        const uint8_t *data, //vetor que não pode ser alterado de dados
                        uint32_t len, //tamanho do buffer
                        void *intf_ptr) //ponteiro generico 

{
    uint8_t buffer[len + 1]; //vetor que guarda o endereço do registrador

    buffer[0] = reg_addr; 

    memcpy(&buffer[1], data, len);// (Endereço destino, Endereço fonte, qntd de bytes a ser enviado)

    //envia transição completa por i2c(retorna sucesso ou erro)
    return i2c_master_transmit( bmp388_i2c, //Handle dispositivo i2c
                                buffer, //ponteiro para os dados
                                len + 1, //tamamho total
                                -1); //timeout infinito
}

//Função de Leitura para API
int8_t bmp388_i2c_read(uint8_t reg_addr,
                       uint8_t *data,
                       uint32_t len,
                       void *intf_ptr)
{
    i2c_master_transmit(bmp388_i2c, &reg_addr, 1, -1);
    return i2c_master_receive(bmp388_i2c, data, len, -1);
}


//Função de delay para API
void bmp388_delay_us(uint32_t period, void *intf_ptr)
{
    esp_rom_delay_us(period);
}


esp_err_t bmp388_init_sensor(void)
{
    struct bmp3_dev *dev = &bmp388; //ponteiro para struct global 
    int8_t rslt; //varaivel que pega o retorno do driver(se foi sucesso ou falha)

    dev->intf = BMP3_I2C_INTF; //sensor usando I2C
    dev->read = bmp388_i2c_read; //"Driver quando quiser ler use esse função"
    dev->write = bmp388_i2c_write; // "Driver quando quiser escrever use esse função"
    dev->delay_us = bmp388_delay_us; //"Driver quando quiser delay use esse função"
    dev->intf_ptr = NULL;
    

    rslt = bmp3_init(dev);
    if (rslt != BMP3_OK) {
        ESP_LOGE(TAG, "Erro bmp3_init: %d", rslt);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BMP388 inicializado com sucesso");
    return ESP_OK;
}

void bmp388_config_sensor(void)
{
    // Estrutura que contém TODAS as configurações possíveis do BMP388
    struct bmp3_settings settings = {0};

    // Máscara que informa ao driver QUAIS campos serão aplicados
    uint16_t settings_sel = 0;

    /* =========================
       HABILITA PRESSÃO E TEMPERATURA
       ========================= */

    // Liga o sensor de pressão
    settings.press_en = BMP3_ENABLE;

    // Liga o sensor de temperatura 
    settings.temp_en  = BMP3_ENABLE;

    // Diz ao driver que essas duas opções devem ser aplicadas
    settings_sel |= BMP3_SEL_PRESS_EN;
    settings_sel |= BMP3_SEL_TEMP_EN;

    /* =========================
       OVERSAMPLING
       ========================= */

    // Oversampling da pressão (4x = mais estabilidade)
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;

    // Oversampling da temperatura (1x já é suficiente)
    settings.odr_filter.temp_os  = BMP3_NO_OVERSAMPLING;

    // Diz ao driver que os oversamplings devem ser aplicados
    settings_sel |= BMP3_SEL_PRESS_OS;
    settings_sel |= BMP3_SEL_TEMP_OS;

    /* =========================
       FILTRO IIR
       ========================= */

    // Filtro interno para suavizar leituras
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;

    // Informa que o filtro IIR será configurado
    settings_sel |= BMP3_SEL_IIR_FILTER;

    /* =========================
       ODR (frequência interna do sensor)
       ========================= */

    // Sensor atualiza os dados internamente a 25 Hz
    settings.odr_filter.odr = BMP3_ODR_25_HZ;

    // Informa que o ODR deve ser aplicado
    settings_sel |= BMP3_SEL_ODR;

    /* =========================
       APLICA CONFIGURAÇÕES
       ========================= */

    // Envia tudo para o sensor via driver
    bmp3_set_sensor_settings(settings_sel, &settings, &bmp388);

    /* =========================
       MODO DE OPERAÇÃO
       ========================= */

    // Coloca o BMP388 em modo NORMAL (medição contínua)
   // Coloca o sensor em operação
    bmp3_set_op_mode(&settings, &bmp388);
}


typedef struct{
    float temperatura;
    float pressao;

}bmp388_data_t;


void sensor_task(void *pvParameters){

    struct bmp3_data data;
    bmp388_data_t dados_app;

    // Inicializa o driver BMP388 (interface, chip id, etc)
    if (bmp388_init_sensor() != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao iniciar BMP388");
        vTaskDelete(NULL);
    }

     // Configura medições, oversampling, filtros
    bmp388_config_sensor();
    

    while(1){
        //Aguarda notificação do timer
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

         // Lê sensor
        int8_t rslt = bmp3_get_sensor_data(
            BMP3_PRESS | BMP3_TEMP,
            &data,
            &bmp388
        );

         if (rslt == BMP3_OK) {
            dados_app.temperatura = data.temperature;
            dados_app.pressao = data.pressure;

            //envia para a fila
            xQueueOverwrite(bmp388_queue, &dados_app);

            ESP_LOGI(TAG,
                     "BMP388 T=%.2f C  P=%.2f Pa",
                     data.temperature,
                     data.pressure);
        } else {
            ESP_LOGE(TAG, "Erro leitura BMP388: %d", rslt);
        }
    }



}


/* ============================================================
   Inicialização do Display OLED
   ============================================================ */
void oled_init(){


    esp_err_t i2c_ret;

    
    // Configuração do barramento I2C
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_BUS_NUM,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true }
    };

    i2c_master_bus_handle_t bus_handle = NULL;
    i2c_ret = i2c_new_master_bus(&bus_cfg, &bus_handle);

     if (i2c_ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao iniciar I2C: %d", i2c_ret);
        return;
    }

     // Config padrão do display
    ssd1306_config_t disp_cfg = I2C_SSD1306_128x64_CONFIG_DEFAULT;

    i2c_ret = ssd1306_init(bus_handle, &disp_cfg, &oled_handle);

    if (i2c_ret != ESP_OK || oled_handle == NULL) {
        ESP_LOGE(TAG, "Falha ao iniciar SSD1306: %d",  i2c_ret);
        return;
    }
}


void exibir_task(void *pvParameters)
{
    bmp388_data_t dados; 
    char texto[50];
    

    while (1) {

        xQueueReceive(bmp388_queue,&dados, portMAX_DELAY); // Recebendo os dados da fila global do sensor

        // espera notificação do timer
        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (oled_handle == NULL) {
            ESP_LOGE(TAG, "OLED nao inicializado");
            continue;
        }

        snprintf(texto, sizeof(texto),"T: %.2f C\nP: %.0f Pa",dados.temperatura,dados.pressao);

        

        ssd1306_clear_display(oled_handle, true);
        ssd1306_display_text(oled_handle, 0, texto, false);
        ssd1306_display_pages(oled_handle);

        ESP_LOGI(TAG, "Display atualizado");
    }
}


   
// Função chamada automaticamente pelo FreeRTOS
// toda vez que o software timer expira (a cada 1s)
void timer_callback(TimerHandle_t xTimer)
{
    if(sensor_task_handle != NULL ){
        // Envia uma notificação para a task
        xTaskNotifyGive(sensor_task_handle);
    }
   
}





void app_main(void)
{
    

    // Inicializa o barramento I2C e o display OLED
    oled_init();

    //Cria queue bmp388
    bmp388_queue = xQueueCreate(1, sizeof(bmp388_data_t));

    

    //Cria a task responsavel por ler o sensor
    xTaskCreate(
    sensor_task,                // Função que será executada pela task
    "SENSOR",                   // Nome da task (apenas para debug/log)
    4096,                       // Tamanho da stack da task (em bytes)
    NULL,                       // Parâmetro passado para a task (não usado)
    6,                          // prioridade maior que a do display
    &sensor_task_handle         // Handle da task (usado para notificação pelo timer)
    );


    // Cria a task responsável por atualizar o display
    xTaskCreate(
        exibir_task,            // Função que será executada pela task
        "OLED",                 // Nome da task (apenas para debug/log)
        2048,                   // Tamanho da stack da task (em bytes)
        NULL,                   // Parâmetro passado para a task (não usado)
        5,                      // Prioridade da task (quanto maior, maior prioridade)
        NULL                    // Handle da task (usado para notificação pelo timer)
    );

    // Cria um software timer do FreeRTOS
    timer_1s = xTimerCreate(
        "timer_1s",             // Nome do timer (usado apenas para debug)
        pdMS_TO_TICKS(1000),    // Período do timer (1000 ms = 1 segundo)
        pdTRUE,                 // pdTRUE → timer periódico (se fosse pdFALSE seria one-shot)
        NULL,                   // ID do timer (não utilizado neste projeto)
        timer_callback          // Função callback chamada a cada disparo do timer
    );

    // Verifica se o timer foi criado corretamente
    if (timer_1s == NULL) {
        ESP_LOGE(TAG, "Erro ao criar timer");
        return;
    }

    // Inicia o timer
    // O segundo parâmetro é o tempo máximo que a função pode esperar
    // para enviar o comando de start para o serviço de timers
    xTimerStart(timer_1s, 0);

    // Log informando que o sistema está funcionando
    ESP_LOGI(TAG, "Sistema iniciado com timer e task");
}
