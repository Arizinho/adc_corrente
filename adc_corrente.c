#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include <stdio.h>
#include <math.h>

#define SAMPLES_PER_CYCLE 256
#define SAMPLE_RATE 60 * SAMPLES_PER_CYCLE 
#define CYCLES 3
#define TOTAL_SAMPLES (SAMPLES_PER_CYCLE * CYCLES)

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 16
#define UART_RX_PIN 17

#define TRIGGER_GPIO 15

volatile bool trigger_medida = false;

void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == TRIGGER_GPIO && (events & GPIO_IRQ_EDGE_RISE)) {
        trigger_medida = true;
    }
}

const float WC = 2.0f * M_PI * 106.0f;
const float A  = 2.0f * SAMPLE_RATE;
const float B  = WC * sqrtf(2.0f);
const float C  = WC * WC;
const float raizde2 = sqrtf(2.0f);

const float K1 = C / (A*A + A*B + C);
const float K2 = 2.0f * (C - A*A) / (A*A + A*B + C);
const float K3 = (A*A + C - A*B) / (A*A + A*B + C);

const float hc[16] = {(4.0/32.0) * cos(2.0*M_PI*1.0/32.0), (4.0/32.0) * cos(2.0*M_PI*2.0/32.0), (4.0/32.0) * cos(2.0*M_PI*3.0/32.0), (4.0/32.0) * cos(2.0*M_PI*4.0/32.0),
                     (4.0/32.0) * cos(2.0*M_PI*5.0/32.0), (4.0/32.0) * cos(2.0*M_PI*6.0/32.0), (4.0/32.0) * cos(2.0*M_PI*7.0/32.0), (4.0/32.0) * cos(2.0*M_PI*8.0/32.0),
                     (4.0/32.0) * cos(2.0*M_PI*9.0/32.0), (4.0/32.0) * cos(2.0*M_PI*10.0/32.0), (4.0/32.0) * cos(2.0*M_PI*11.0/32.0), (4.0/32.0) * cos(2.0*M_PI*12.0/32.0),
                     (4.0/32.0) * cos(2.0*M_PI*13.0/32.0), (4.0/32.0) * cos(2.0*M_PI*14.0/32.0), (4.0/32.0) * cos(2.0*M_PI*15.0/32.0), (4.0/32.0) * cos(2.0*M_PI*16.0/32.0)};

const float hs[16] = {(4.0/32.0) * sin(2.0*M_PI*1.0/32.0), (4.0/32.0) * sin(2.0*M_PI*2.0/32.0), (4.0/32.0) * sin(2.0*M_PI*3.0/32.0), (4.0/32.0) * sin(2.0*M_PI*4.0/32.0),
                     (4.0/32.0) * sin(2.0*M_PI*5.0/32.0), (4.0/32.0) * sin(2.0*M_PI*6.0/32.0), (4.0/32.0) * sin(2.0*M_PI*7.0/32.0), (4.0/32.0) * sin(2.0*M_PI*8.0/32.0),
                     (4.0/32.0) * sin(2.0*M_PI*9.0/32.0), (4.0/32.0) * sin(2.0*M_PI*10.0/32.0), (4.0/32.0) * sin(2.0*M_PI*11.0/32.0), (4.0/32.0) * sin(2.0*M_PI*12.0/32.0),
                     (4.0/32.0) * sin(2.0*M_PI*13.0/32.0), (4.0/32.0) * sin(2.0*M_PI*14.0/32.0), (4.0/32.0) * sin(2.0*M_PI*15.0/32.0), (4.0/32.0) * sin(2.0*M_PI*16.0/32.0)};

int main() {
    //stdio_init_all();

    sleep_ms(2000); // Aguarda inicialização da porta serial

    // --- Trigger no GPIO 15 ---
    gpio_init(TRIGGER_GPIO);
    gpio_set_dir(TRIGGER_GPIO, GPIO_IN);
    gpio_pull_down(TRIGGER_GPIO);

    gpio_set_irq_enabled_with_callback(
        TRIGGER_GPIO,
        GPIO_IRQ_EDGE_RISE,
        true,
        &gpio_callback
    );

    // Inicializa o ADC
    adc_init();
    adc_gpio_init(26);  // ADC0 no GPIO26
    adc_gpio_init(27);  // ADC1 no GPIO27
    adc_gpio_init(28);  // ADC2 no GPIO28

    //Inicializa comunicação UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    char buffer_uart[80];

    // Configura clock do ADC para 15.36kHz * 3 canais = 46.08MHz
    adc_set_clkdiv(48000000.0 / (SAMPLE_RATE * 3));  

    // Configura modo round-robin: alterna entre ch0 e ch1
    adc_select_input(0);
    adc_set_round_robin((1 << 0) | (1 << 1) | (1 << 2)); // canais 0, 1 e 2
    adc_fifo_setup(
        true,  // habilita FIFO
        true,  // habilita req de DMA
        1,     // DREQ a cada amostra
        false,
        false
    );
    adc_fifo_drain(); // Limpa FIFO

    // DMA channel
    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_ADC);

    // Buffer para leituras intercaladas (ch0, ch1, ch2, ch0, ch1, ch2, ...)
    static uint16_t raw_data[TOTAL_SAMPLES * 3];

    // Inicia transferência DMA
    dma_channel_configure(
        dma_chan,
        &c,
        raw_data,            // destino
        &adc_hw->fifo,       // fonte
        TOTAL_SAMPLES * 3,   // 3 canais intercalados
        false
    );
    

    sleep_ms(100);

    int N = TOTAL_SAMPLES/CYCLES;
    trigger_medida = false;
    while (1) {
        while (!trigger_medida) {
            tight_loop_contents(); // dica pro SDK, não adiciona atraso perceptível
        }

        trigger_medida = false;

        // reconfigura o número de transferências
        dma_channel_set_trans_count(dma_chan, TOTAL_SAMPLES * 3, false);
        // aponta o buffer e dispara
        dma_channel_set_write_addr(dma_chan, raw_data, true);

        adc_run(true);
        dma_channel_wait_for_finish_blocking(dma_chan);
        adc_run(false);
        adc_fifo_drain();

        uint count1 = 0, count2 = 0, count3 = 0;
        float soma_ia = 0, soma_ib = 0, soma_ic = 0;

        float Y_cia = 0, Y_sia = 0, Y_cib = 0, Y_sib = 0, Y_cic = 0, Y_sic = 0;

        // estados do filtro tensão ia
        float y_ia1 = 0, y_ia2 = 0;
        float ia_ant1 = 0, ia_ant2 = 0;

        // estados do filtro tensão ib
        float y_ib1 = 0, y_ib2 = 0;
        float ib_ant1 = 0, ib_ant2 = 0;

        // estados do filtro tensão ic
        float y_ic1 = 0, y_ic2 = 0;
        float ic_ant1 = 0, ic_ant2 = 0;

        // Primeiro calcula os offsets médios
        float soma_ch0 = 0, soma_ch1 = 0, soma_ch2 = 0;
        for (int i = 0; i < TOTAL_SAMPLES; i++) {
            soma_ch0 += raw_data[3*i];     // canal tensão ic
            soma_ch1 += raw_data[3*i + 1]; // canal tensão ib
            soma_ch2 += raw_data[3*i + 2]; // canal tensão ia
        }
        float offset_ia = soma_ch2 / TOTAL_SAMPLES;
        float offset_ib = soma_ch1 / TOTAL_SAMPLES;
        float offset_ic = soma_ch0 / TOTAL_SAMPLES;

        for (int i = 0; i < TOTAL_SAMPLES; i++) {
            // pega valores intercalados
            float leitura_ia   = ((float)raw_data[3*i + 2] - offset_ia) * 0.027739142f;
            float leitura_ib   = ((float)raw_data[3*i + 1] - offset_ib) * 0.027739142f;
            float leitura_ic   = ((float)raw_data[3*i] - offset_ic) * 0.027739142f;

            // filtro ia
            float y_ia = -K2*y_ia1 - K3*y_ia2
                    + K1*leitura_ia
                    + 2*K1*ia_ant1
                    + K1*ia_ant2;
            ia_ant2 = ia_ant1; ia_ant1 = leitura_ia;
            y_ia2 = y_ia1; y_ia1 = y_ia;

            // filtro ib
            float y_ib = -K2*y_ib1 - K3*y_ib2
                    + K1*leitura_ib
                    + 2*K1*ib_ant1
                    + K1*ib_ant2;
            ib_ant2 = ib_ant1; ib_ant1 = leitura_ib;
            y_ib2 = y_ib1; y_ib1 = y_ib;

            // filtro ic
            float y_ic = -K2*y_ic1 - K3*y_ic2
                    + K1*leitura_ic
                    + 2*K1*ic_ant1
                    + K1*ic_ant2;
            ic_ant2 = ic_ant1; ic_ant1 = leitura_ic;
            y_ic2 = y_ic1; y_ic1 = y_ic;


            // acumula só último ciclo
            if (i >= (CYCLES-1)*N) {
                count1++;
                if (count1 >= 8 && count2 < 16) {
                    Y_cia += y_ia * hc[15-count2];
                    Y_sia += y_ia * hs[15-count2];
                    Y_cib += y_ib * hc[15-count2];
                    Y_sib += y_ib * hs[15-count2];
                    Y_cic += y_ic * hc[15-count2];
                    Y_sic += y_ic * hs[15-count2];
                    count1 = 0;
                    count2++;
                }
                soma_ia += y_ia*y_ia;
                soma_ib += y_ib*y_ib;
                soma_ic += y_ic*y_ic;
                // soma_i += z*z;
                // soma_p += y*z;
                //printf("%.2f;", leitura_ia);
            }
        }

        float angulo_ia = atan2f(Y_sia, Y_cia);
        float angulo_ib = atan2f(Y_sib, Y_cib);
        float angulo_ic = atan2f(Y_sic, Y_cic);

        // float modulo_ia = sqrtf(Y_cia*Y_cia + Y_sia*Y_sia);
        // float modulo_ib = sqrtf(Y_cib*Y_cib + Y_sib*Y_sib);
        // float modulo_ic = sqrtf(Y_cic*Y_cic + Y_sic*Y_sic);

        // float Ia_rms = modulo_ia/raizde2;
        // float Ib_rms = modulo_ib/raizde2;
        // float Ic_rms = modulo_ic/raizde2;

        float Ia_rms = sqrtf(soma_ia / N);
        float Ib_rms = sqrtf(soma_ib / N);
        float Ic_rms = sqrtf(soma_ic / N);

        // Formata e envia os dados via UART
        snprintf(buffer_uart, sizeof(buffer_uart), "Ia%.4f Ib%.4f Ic%.4f pA%.4f pB%.4f pC%.4f;", Ia_rms, Ib_rms, Ic_rms, angulo_ia, angulo_ib, angulo_ic);
        uart_puts(UART_ID, buffer_uart);
    }
}
