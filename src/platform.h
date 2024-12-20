#pragma once

#include <stdlib.h>

#include "peer_connection.h"

inline constexpr int kCaptureSampleRate = 8000;
inline constexpr int kCaptureChannelCount = 1;
inline constexpr int kPlaybackSampleRate = 8000;
inline constexpr int kPlaybackChannelCount = 2;

void oai_platform_init(void);
void oai_platform_restart(void);
void oai_platform_init_audio_capture(void);
void oai_platform_audio_write(char *output_buffer, size_t output_buffer_size,
                              size_t *bytes_written);
void oai_platform_audio_read(char *input_buffer, size_t input_buffer_size,
                             size_t *bytes_read);
void oai_platform_send_audio_task(PeerConnection *peer_connection);
