#include <pulse/error.h>
#include <pulse/pulseaudio.h>
#include <pulse/simple.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "media.h"
#include "peer_connection.h"
#include "platform.h"

static pa_simple *capture = NULL;
static pa_simple *playback = NULL;

void oai_platform_init(void) {}

void oai_platform_restart() {
  abort();
}

void oai_wifi(void) {}

void oai_platform_init_audio_capture() {
  int error;

  pa_sample_spec capture_spec = {.format = PA_SAMPLE_S16LE,
                                 .rate = kCaptureSampleRate,
                                 .channels = kCaptureChannelCount};

  pa_buffer_attr attr = {
      .maxlength = (uint32_t)-1,
      .tlength = (uint32_t)-1,
      .prebuf = (uint32_t)-1,
      .minreq = (uint32_t)-1,
      .fragsize = (uint32_t)-1,
  };

  // Create capture stream
  capture = pa_simple_new(NULL,              // Use default server
                          "OAI",             // Application name
                          PA_STREAM_RECORD,  // Stream direction
                          NULL,              // Use default device
                          "Capture",         // Stream description
                          &capture_spec,     // Sample format
                          NULL,              // Use default channel map
                          &attr,             // Buffer attributes
                          &error             // Error code
  );

  if (!capture) {
    fprintf(stderr, "Failed to create capture stream: %s\n",
            pa_strerror(error));
    return;
  }

  pa_sample_spec playback_spec = {.format = PA_SAMPLE_S16LE,
                                  .rate = kPlaybackSampleRate,
                                  .channels = kPlaybackChannelCount};

  playback = pa_simple_new(NULL,                // Use default server
                           "OAI",               // Application name
                           PA_STREAM_PLAYBACK,  // Stream direction
                           NULL,                // Use default device
                           "Playback",          // Stream description
                           &playback_spec,      // Sample format
                           NULL,                // Use default channel map
                           &attr,               // Buffer attributes
                           &error               // Error code
  );

  if (!playback) {
    fprintf(stderr, "Failed to create playback stream: %s\n",
            pa_strerror(error));
    pa_simple_free(capture);
    return;
  }
}

void oai_platform_audio_write(char *output_buffer, size_t output_buffer_size,
                              size_t *bytes_written) {
  int error;
  if (pa_simple_write(playback, output_buffer, output_buffer_size, &error) <
      0) {
    fprintf(stderr, "Read failed: %s\n", pa_strerror(error));
    return;
  }

  *bytes_written = output_buffer_size;
}

void oai_platform_audio_read(char *input_buffer, size_t input_buffer_size,
                             size_t *bytes_read) {
  int error;
  if (pa_simple_read(capture, input_buffer, input_buffer_size, &error) < 0) {
    fprintf(stderr, "Read failed: %s\n", pa_strerror(error));
    return;
  }

  *bytes_read = input_buffer_size;
}

static TaskHandle_t task_handle;

static void oai_send_audio_task(void *user_data) {
  while (1) {
    oai_send_audio((PeerConnection *)user_data);
  }
}

void oai_platform_send_audio_task(PeerConnection *peer_connection) {
  xTaskCreatePinnedToCore(oai_send_audio_task, "audio_publisher", 20000,
                          peer_connection, -7, &task_handle, 0);
}
