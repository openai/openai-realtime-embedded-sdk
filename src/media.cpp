#include <opus.h>
#include <stdio.h>

#include "main.h"
#include "platform.h"

#define OPUS_OUT_BUFFER_SIZE 1276  // 1276 bytes is recommended by opus_encode

#define OPUS_ENCODER_BITRATE 30000
#define OPUS_ENCODER_COMPLEXITY 0

const auto kCaptureFrameSize = kCaptureSampleRate * 20 / 1000;
const auto kPlaybackFrameSize = kPlaybackSampleRate * 20 / 1000;

static OpusDecoder *opus_decoder = NULL;
static OpusEncoder *opus_encoder = NULL;

static opus_int16 output_buffer[kPlaybackFrameSize * kPlaybackChannelCount];
static opus_int16 input_buffer[kCaptureFrameSize * kCaptureChannelCount];
static uint8_t encoder_output_buffer[OPUS_OUT_BUFFER_SIZE];

void oai_init_audio_decoder() {
  int decoder_error = 0;
  opus_decoder = opus_decoder_create(kPlaybackSampleRate, kPlaybackChannelCount,
                                     &decoder_error);
  if (decoder_error != OPUS_OK) {
    printf("Failed to create OPUS decoder");
    return;
  }
}

void oai_audio_decode(uint8_t *data, size_t size) {
  int decoded_size = opus_decode(opus_decoder, data, size, output_buffer,
                                 sizeof(output_buffer), 0);

  if (decoded_size > 0) {
    size_t bytes_written = 0;
    oai_platform_audio_write((char *)output_buffer, sizeof(output_buffer),
                             &bytes_written);
  }
}

void oai_init_audio_encoder() {
  int encoder_error;
  opus_encoder = opus_encoder_create(kCaptureSampleRate, kCaptureChannelCount,
                                     OPUS_APPLICATION_VOIP, &encoder_error);
  if (encoder_error != OPUS_OK) {
    printf("Failed to create OPUS encoder");
    return;
  }

  opus_encoder_ctl(opus_encoder, OPUS_SET_BITRATE(OPUS_ENCODER_BITRATE));
  opus_encoder_ctl(opus_encoder, OPUS_SET_COMPLEXITY(OPUS_ENCODER_COMPLEXITY));
  opus_encoder_ctl(opus_encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
}

void oai_send_audio(PeerConnection *peer_connection) {
  size_t bytes_read = 0;

  oai_platform_audio_read((char *)input_buffer, sizeof(input_buffer),
                          &bytes_read);

  auto encoded_size =
      opus_encode(opus_encoder, input_buffer, kCaptureFrameSize,
                  encoder_output_buffer, sizeof(encoder_output_buffer));

  peer_connection_send_audio(peer_connection, encoder_output_buffer,
                             encoded_size);
}
