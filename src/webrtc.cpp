#include <esp_event.h>
#include <esp_log.h>
#include <opus.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main.h"
#include "peer_connection.h"
#include "platform.h"

#define TICK_INTERVAL 15
#define GREETING                                                    \
  "{\"type\": \"response.create\", \"response\": {\"modalities\": " \
  "[\"audio\", \"text\"], \"instructions\": \"Say 'How can I help?.'\"}}"

static PeerConnection *peer_connection = NULL;

static void oai_ondatachannel_onmessage_task(char *msg, size_t len,
                                             void *userdata, uint16_t sid) {
#ifdef LOG_DATACHANNEL_MESSAGES
  ESP_LOGI(LOG_TAG, "DataChannel Message: %s", msg);
#endif
}

static void oai_ondatachannel_onopen_task(void *userdata) {
  if (peer_connection_create_datachannel(peer_connection, DATA_CHANNEL_RELIABLE,
                                         0, 0, (char *)"oai-events",
                                         (char *)"") != -1) {
    ESP_LOGI(LOG_TAG, "DataChannel created");
    peer_connection_datachannel_send(peer_connection, (char *)GREETING,
                                     strlen(GREETING));
  } else {
    ESP_LOGE(LOG_TAG, "Failed to create DataChannel");
  }
}

static void oai_onconnectionstatechange_task(PeerConnectionState state,
                                             void *user_data) {
  ESP_LOGI(LOG_TAG, "PeerConnectionState: %s",
           peer_connection_state_to_string(state));

  if (state == PEER_CONNECTION_DISCONNECTED ||
      state == PEER_CONNECTION_CLOSED) {
    oai_platform_restart();
  } else if (state == PEER_CONNECTION_CONNECTED) {
    oai_init_audio_encoder();
    oai_platform_send_audio_task(peer_connection);
  }
}

static void oai_on_icecandidate_task(char *description, void *user_data) {
  char local_buffer[MAX_HTTP_OUTPUT_BUFFER + 1] = {0};
  oai_http_request(description, local_buffer);
  peer_connection_set_remote_description(peer_connection, local_buffer);
}

void oai_webrtc() {
  PeerConfiguration peer_connection_config = {
      .ice_servers = {},
      .audio_codec = CODEC_OPUS,
      .video_codec = CODEC_NONE,
      .datachannel = DATA_CHANNEL_STRING,
      .onaudiotrack = [](uint8_t *data, size_t size, void *userdata) -> void {
        oai_audio_decode(data, size);
      },
      .onvideotrack = NULL,
      .on_request_keyframe = NULL,
      .user_data = NULL,
  };

  peer_connection = peer_connection_create(&peer_connection_config);
  if (peer_connection == NULL) {
    ESP_LOGE(LOG_TAG, "Failed to create peer connection");
    oai_platform_restart();
  }

  peer_connection_oniceconnectionstatechange(peer_connection,
                                             oai_onconnectionstatechange_task);
  peer_connection_onicecandidate(peer_connection, oai_on_icecandidate_task);
  peer_connection_ondatachannel(peer_connection,
                                oai_ondatachannel_onmessage_task,
                                oai_ondatachannel_onopen_task, NULL);

  peer_connection_create_offer(peer_connection);

  while (1) {
    peer_connection_loop(peer_connection);
    vTaskDelay(pdMS_TO_TICKS(TICK_INTERVAL));
  }
}
