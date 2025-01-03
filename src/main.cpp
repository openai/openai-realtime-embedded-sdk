#include "main.h"

#include <esp_event.h>
#include <esp_log.h>
#include <peer.h>

#include "platform.h"

#ifndef LINUX_BUILD
#define MAIN extern "C" void app_main(void)
#else
#define MAIN int main(void)
#endif

MAIN {
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  peer_init();
  oai_platform_init_audio_capture();
  oai_init_audio_decoder();
  oai_wifi();
  oai_webrtc();
}
