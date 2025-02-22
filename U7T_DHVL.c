// Bibliotecas incluídas para o Projeto
#include <stdio.h>
#include <math.h>
#include <string.h>  
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "inc/ssd1306.h"
#include "inc/ssd1306_i2c.h"

// Definição dos Pinos e Constantes do Sistema
#define BUTTON_PIN 5 // Pino do botão (Botão A)
 
#define LED_PIN  13 // Pino do LED
#define PWM_WRAP 12500 // Valor de envoltória do PWM

#define MIC_CHANNEL 2 // Canal ADC do microfone
#define MIC_ADC_PIN 28 // Pino do ADC
#define ADC_CLOCK_DIV 96.f // Divisor de clock do ADC
#define SAMPLES 200 // Número de amostras 
#define ADC_RESOLUTION (1 << 12)  // Resolução do ADC (12-bit ADC -> 4096 níveis)
#define V_REF 3.3f  // Tensão de referência do ADC
#define MIC_OFFSET 1.65f  // Offset do sinal do microfone (meio da escala de 3.3V)

// Definição de variáveis globais
volatile bool state = false; // Estado atual do sistema (NIGHT ou DAY)

uint dma_channel; // Canal DMA
dma_channel_config dma_cfg; // Configuração do DMA
uint16_t adc_buffer[SAMPLES]; // Buffer de armazenamento de amostras de ADC

const uint I2C_SDA = 14; // Pino SDA do I2C
const uint I2C_SCL = 15; // Pino SCL do I2C
uint8_t ssd[ssd1306_buffer_length]; // Buffer do display OLED

// Funções de Configuração e Inicialização

// Função para configuração do botão
void setup_button(){
    gpio_init(BUTTON_PIN); // Inicialização do pino 
    gpio_set_dir(BUTTON_PIN, GPIO_IN); // Definição do pino como entrada
    gpio_pull_up(BUTTON_PIN); // Ativação do pull-up
}

// Função para configuração do PWM para controle do LED
void setup_pwm() {
    gpio_set_function(LED_PIN, GPIO_FUNC_PWM); // Definição do pino como saída PWM
    uint slice_num = pwm_gpio_to_slice_num(LED_PIN);  // Número do "slice" PWM 

    pwm_set_wrap(slice_num, PWM_WRAP); // Valor máximo da contagem PWM
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0); // Inicialização do canal PWM com nível 0
    pwm_set_enabled(slice_num, true);   // Habilita o PWM
}

// Função para configuração do ADC
void setup_adc(){
    adc_gpio_init(MIC_ADC_PIN); // Inicialização do pino como ADC
    adc_init(); // Inicialização do módulo ADC

    adc_select_input(MIC_CHANNEL); // Seleção do canal de entrada do ADC
    adc_fifo_setup(true, true, 1, false, false); // Configuração do FIFO
    adc_set_clkdiv(ADC_CLOCK_DIV); // Definição do divisor de Clock
}

// Função para configuração do DMA para obteção de dados do ADC
void setup_dma(){
    // Requisição de um canal DMA disponível
    dma_channel = dma_claim_unused_channel(true);

    // Configuração padrão para o canal DMA 
    dma_cfg = dma_channel_get_default_config(dma_channel);

    // Definição do tamanho da transferência de dados (16 bits)
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16);

    // Desativação do incremento do endereço de leitura
    channel_config_set_read_increment(&dma_cfg, false);

    // Ativação do incremento do endereço de escrita 
    channel_config_set_write_increment(&dma_cfg, true);

    // Definição do gatilho da transferência DMA como o ADC
    channel_config_set_dreq(&dma_cfg, DREQ_ADC);
}

// Função para configuração do display OLED
void setup_oled(){
    // Inicialização da comunicação I2C
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);

    // Definição dos pinos SDA e SCL
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    // Ativação do pull-up
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicialização do display OLED
    ssd1306_init();

    // Limpeza do buffer do display
    memset(ssd, 0, ssd1306_buffer_length);
}

// Funções Auxiliares

// Função para ajustar o brilho do LED
void set_led_brightness(float percent) {
    uint slice_num = pwm_gpio_to_slice_num(LED_PIN); // Número do slice PWM
    uint level = (uint)(PWM_WRAP * percent);  // Cálculo proporcional do Duty Cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, level); // Ajuste do brilho
}

// Função para coletar amostras do microfone via DMA
void sample_mic() {
    adc_fifo_drain(); // Esvaziamendo do FIFO do ADC
    adc_run(false); // Desativação do ADC

    dma_channel_configure(dma_channel, &dma_cfg, adc_buffer, &(adc_hw->fifo), SAMPLES, true); // Configuração do canal DMA
    
    adc_run(true); // Ativação do ADC
    dma_channel_wait_for_finish_blocking(dma_channel); // Aguardo da conclusção da transferência DMA
    adc_run(false); // Desativação do ADC
}

// Função para calcular a potência RMS do sinal do microfone
float mic_rms() {
    float sum_sq = 0.0f;

    // Percorre as amostras coletadas 
    for (uint i = 0; i < SAMPLES; ++i) {
        float voltage = (adc_buffer[i] * V_REF / ADC_RESOLUTION) - MIC_OFFSET;  // Conversão ADC para tensão (V)
        sum_sq += voltage * voltage;  // Soma dos quadrados
    }

    float rms = sqrt(sum_sq / SAMPLES);  // Cálculo RMS
    return rms;
}

// Função para converter o sinal RMS para dB
float convert_to_dB(float v_rms) {
    float ref_voltage = 0.001f;  // 1mV de referência para conversão dB
    if (v_rms <= 0) return -100.0f;  // Tratamento de Excessão (v_rms <= 0)
    
    return 20.0f * log10f(v_rms / ref_voltage);  // Cálculo para conversão para dB
}

// Função para atualizar a exibição do display OLED
void display_state(uint8_t *ssd, bool state) {
    // Definição da área de renderização 
    struct render_area frame_area = {
        start_column : 0,
        end_column : ssd1306_width - 1,
        start_page : 0,
        end_page : ssd1306_n_pages - 1
    };

    // Cálculo do tamanho do buffer necessário
    calculate_render_area_buffer_length(&frame_area);

    // Limpeza do buffer do display
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);

    // Texto a ser exibido no display
    const char *text = state ? "NIGHT" : "DAY"; 

    // Cálculo do comprimento do texto(em pixels) e centralização dele
    int text_length = strlen(text) * 6;  
    int x_position = (ssd1306_width - text_length) / 2;  
    int y_position = (ssd1306_height - 8) / 2;

    // Escrita do texto no buffer 
    ssd1306_draw_string(ssd, x_position, y_position, text);

    // Atualização do display 
    render_on_display(ssd, &frame_area);
}

// Função Principal
int main() {
    // Inicializa a interface padrão de entrada/saída
    stdio_init_all();

    // Configuração dos periféricos
    setup_button();
    setup_pwm();
    setup_adc(); 
    setup_dma();
    setup_oled();

    // Loop principal do programa
    while (true) {
        // Atualiza o display OLED com o estado atual (NIGHT ou DAY)
        display_state(ssd, state);

        // Verifica o pressionamento do botão
        if(!gpio_get(BUTTON_PIN)){
            sleep_ms(50); // Debounce
            while (!gpio_get(BUTTON_PIN));
            state = !state; 
            printf("Modo: %s\n", state ? "NIGHT (30dB)" : "DAY (60dB)");
        }

        // Captura das amostras do microfone via DMA
        sample_mic();
        float rms_voltage = mic_rms();  // Obtém o valor RMS
        float dB = convert_to_dB(rms_voltage);  // Converte para dB

        printf("Volume (dB): %.2f dB\n", dB);

        // Definição o limite de detecção de som alto de acordo com o estado atual
        float threshold = state ? 30.0f : 60.0f;
        // Acionamento do LED caso o som ultrapasse o limite
        if (dB >= threshold) {
            printf("Alerta !!! Som em nível prejudicial !!!\n");
            set_led_brightness(0.9f);
            sleep_ms(2000);
            set_led_brightness(0.0f);
        }

        // Intervalo de 1 segundo entre leituras do microfone
        sleep_ms(1000);

    }
}


