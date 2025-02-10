#if (CONFIG_OPENAI_WITH_GMF == 0)
#include <driver/i2s.h>
#include <opus.h>

#include "main.h"

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

#define OPUS_ENCODER_BITRATE 30000
#define OPUS_ENCODER_COMPLEXITY 0

void oai_init_audio_capture() {
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
    i2s_write(I2S_NUM_0, output_buffer, BUFFER_SAMPLES * sizeof(opus_int16),
              &bytes_written, portMAX_DELAY);
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

  i2s_read(I2S_NUM_1, encoder_input_buffer, BUFFER_SAMPLES, &bytes_read,
           portMAX_DELAY);

  auto encoded_size =
      opus_encode(opus_encoder, encoder_input_buffer, BUFFER_SAMPLES / 2,
                  encoder_output_buffer, OPUS_OUT_BUFFER_SIZE);

  peer_connection_send_audio(peer_connection, encoder_output_buffer,
                             encoded_size);
}
#endif
