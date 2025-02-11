#include <driver/i2s.h>
#include <opus.h>

#include "main.h"
#define USE_DFR1154 1

#if USE_DFR1154
#include "driver/i2s_pdm.h"
#include "driver/i2s_std.h"
#define OPUS_OUT_BUFFER_SIZE 1276  // 1276 bytes is recommended by opus_encode
#define SAMPLE_RATE 8000
#define BUFFER_SAMPLES 320

#define MCLK_PIN 0
#define DAC_BCLK_PIN 45
#define DAC_LRCLK_PIN 46
#define DAC_DATA_PIN 42

#define PDM_CLK_PIN 38
#define PDM_DATA_PIN 39
i2s_chan_handle_t rx_chan;        // I2S rx channel handler
i2s_chan_handle_t tx_chan;        // I2S tx channel handler

#else
#define OPUS_OUT_BUFFER_SIZE 1276  // 1276 bytes is recommended by opus_encode
#define SAMPLE_RATE 8000
#define BUFFER_SAMPLES 320

#define MCLK_PIN 0
#define DAC_BCLK_PIN 15
#define DAC_LRCLK_PIN 16
#define DAC_DATA_PIN 17
#define ADC_BCLK_PIN 38
#define ADC_LRCLK_PIN 39
#define ADC_DATA_PIN 40
#endif

#define OPUS_ENCODER_BITRATE 30000
#define OPUS_ENCODER_COMPLEXITY 0


void oai_init_audio_capture() {
#if USE_DFR1154
  i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));
  i2s_std_config_t tx_std_cfg={
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = GPIO_NUM_NC,
      .bclk = GPIO_NUM_45,
      .ws   = GPIO_NUM_46,
      .dout = GPIO_NUM_42,
      .din  = GPIO_NUM_NC,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv   = false,
      },
    }
  };
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));
  i2s_channel_enable(tx_chan);

#else
  i2s_config_t i2s_config_out = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S_MSB,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = BUFFER_SAMPLES,
      .use_apll = 1,
      .tx_desc_auto_clear = true,
  };
  
  if (i2s_driver_install(I2S_NUM_0, &i2s_config_out, 0, NULL) != ESP_OK) {
    printf("Failed to configure I2S driver for audio output");
    return;
  }

  i2s_pin_config_t pin_config_out = {
      .mck_io_num = MCLK_PIN,
      .bck_io_num = DAC_BCLK_PIN,
      .ws_io_num = DAC_LRCLK_PIN,
      .data_out_num = DAC_DATA_PIN,
      .data_in_num = I2S_PIN_NO_CHANGE,
  };
  if (i2s_set_pin(I2S_NUM_0, &pin_config_out) != ESP_OK) {
    printf("Failed to set I2S pins for audio output");
    return;
  }
  i2s_zero_dma_buffer(I2S_NUM_0);
#endif

#if USE_DFR1154
  i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));
  i2s_pdm_rx_config_t pdm_rx_cfg = {
      .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
      /* The data bit-width of PDM mode is fixed to 16 */
      .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .clk = GPIO_NUM_38,
          .din = GPIO_NUM_39,
          .invert_flags = {
              .clk_inv = false,
          },
      },
  };
  // Enable all slots for example
  pdm_rx_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
  //pdm_rx_cfg.slot_cfg.slot_mask = I2S_PDM_LINE_SLOT_ALL;
  ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_chan, &pdm_rx_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
#else
  i2s_config_t i2s_config_in = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S_MSB,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = BUFFER_SAMPLES,
      .use_apll = 1,
  };

  if (i2s_driver_install(I2S_NUM_1, &i2s_config_in, 0, NULL) != ESP_OK) {
    printf("Failed to configure I2S driver for audio input");
    return;
  }

  i2s_pin_config_t pin_config_in = {
      .mck_io_num = MCLK_PIN,
      .bck_io_num = ADC_BCLK_PIN,
      .ws_io_num = ADC_LRCLK_PIN,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = ADC_DATA_PIN,
  };
  if (i2s_set_pin(I2S_NUM_1, &pin_config_in) != ESP_OK) {
    printf("Failed to set I2S pins for audio input");
    return;
  }
#endif
}

opus_int16 *output_buffer = NULL;
OpusDecoder *opus_decoder = NULL;

void oai_init_audio_decoder() {
  int decoder_error = 0;
  opus_decoder = opus_decoder_create(SAMPLE_RATE, 2, &decoder_error);
  if (decoder_error != OPUS_OK) {
    printf("Failed to create OPUS decoder");
    return;
  }

  output_buffer = (opus_int16 *)malloc(BUFFER_SAMPLES * sizeof(opus_int16));
}

void oai_audio_decode(uint8_t *data, size_t size) {
  int decoded_size =
      opus_decode(opus_decoder, data, size, output_buffer, BUFFER_SAMPLES, 0);

  if (decoded_size > 0) {
    size_t bytes_written = 0;
    #if USE_DFR1154
      i2s_channel_write(tx_chan, output_buffer, BUFFER_SAMPLES * sizeof(opus_int16), &bytes_written, portMAX_DELAY);
    #else
      i2s_write(I2S_NUM_0, output_buffer, BUFFER_SAMPLES * sizeof(opus_int16), &bytes_written, portMAX_DELAY);
    #endif
  }
}

OpusEncoder *opus_encoder = NULL;
opus_int16 *encoder_input_buffer = NULL;
uint8_t *encoder_output_buffer = NULL;

void oai_init_audio_encoder() {
  int encoder_error;
  opus_encoder = opus_encoder_create(SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP,
                                     &encoder_error);
  if (encoder_error != OPUS_OK) {
    printf("Failed to create OPUS encoder");
    return;
  }

  if (opus_encoder_init(opus_encoder, SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP) !=
      OPUS_OK) {
    printf("Failed to initialize OPUS encoder");
    return;
  }

  opus_encoder_ctl(opus_encoder, OPUS_SET_BITRATE(OPUS_ENCODER_BITRATE));
  opus_encoder_ctl(opus_encoder, OPUS_SET_COMPLEXITY(OPUS_ENCODER_COMPLEXITY));
  opus_encoder_ctl(opus_encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
  encoder_input_buffer = (opus_int16 *)malloc(BUFFER_SAMPLES);
  encoder_output_buffer = (uint8_t *)malloc(OPUS_OUT_BUFFER_SIZE);
}

void oai_send_audio(PeerConnection *peer_connection) {
  size_t bytes_read = 0;
#if USE_DFR1154
  i2s_channel_read(rx_chan, encoder_input_buffer, BUFFER_SAMPLES, &bytes_read, portMAX_DELAY);
#else
  i2s_read(I2S_NUM_1, encoder_input_buffer, BUFFER_SAMPLES, &bytes_read,
           portMAX_DELAY);
#endif


  auto encoded_size =
      opus_encode(opus_encoder, encoder_input_buffer, BUFFER_SAMPLES / 2,
                  encoder_output_buffer, OPUS_OUT_BUFFER_SIZE);
  
  peer_connection_send_audio(peer_connection, encoder_output_buffer,
                             encoded_size);
                
}
