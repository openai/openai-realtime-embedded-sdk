#if (CONFIG_OPENAI_WITH_GMF == 1)
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "bsp/esp-bsp.h"
#include "driver/i2s_pdm.h"
#include "driver/sdmmc_host.h"
#include "esp_audio_enc_default.h"
#include "esp_audio_simple_player.h"
#include "esp_audio_simple_player_advance.h"
#include "esp_codec_dev.h"
#include "esp_err.h"
#include "esp_gmf_audio_enc.h"
#include "esp_gmf_audio_helper.h"
#include "esp_gmf_data_bus.h"
#include "esp_gmf_element.h"
#include "esp_gmf_err.h"
#include "esp_gmf_fifo.h"
#include "esp_gmf_io_codec_dev.h"
#include "esp_gmf_io_i2s_pdm.h"
#include "esp_gmf_oal_mem.h"
#include "esp_gmf_oal_thread.h"
#include "esp_gmf_pipeline.h"
#include "esp_gmf_pool.h"
#include "esp_log.h"
#include "esp_opus_enc.h"
#include "main.h"

static const char *TAG = "gmf";

#define BUFFER_SAMPLES (320 * 1)
#define SAMPLE_RATE (8000 * 1)

static esp_gmf_fifo_handle_t oai_plr_dec_fifo = NULL;
static esp_gmf_fifo_handle_t oai_rec_enc_fifo = NULL;

static esp_codec_dev_handle_t oai_plr_handle = NULL;
static esp_codec_dev_handle_t oai_rec_handle = NULL;

static int16_t *oai_encoder_input_buffer = NULL;

static esp_gmf_err_t oai_record_event_callback(esp_gmf_event_pkt_t *event,
                                               void *ctx) {
  ESP_LOGI(TAG, "RECV el:%s-%p, type:%x, sub:%s, payload:%p, size:%d,%p",
           "OBJ_GET_TAG(event->from)", event->from, (int)event->type,
           esp_gmf_event_get_state_str((esp_gmf_event_state_t)event->sub),
           event->payload, event->payload_size, ctx);
  return ESP_GMF_ERR_OK;
}

static int oai_player_event_callback(esp_asp_event_pkt_t *event, void *ctx) {
  if (event->type == ESP_ASP_EVENT_TYPE_MUSIC_INFO) {
    esp_asp_music_info_t info;
    memcpy(&info, event->payload, event->payload_size);
    ESP_LOGI(TAG, "Get info, rate:%d, channels:%d, bits:%d", info.sample_rate,
             info.channels, info.bits);
  } else if (event->type == ESP_ASP_EVENT_TYPE_STATE) {
    esp_asp_state_t st = ESP_ASP_STATE_NONE;
    memcpy(&st, event->payload, event->payload_size);
    ESP_LOGI(TAG, "Get State, %d,%s", st,
             esp_audio_simple_player_state_to_str(st));
    if (ctx &&
        ((st == ESP_ASP_STATE_STOPPED) || (st == ESP_ASP_STATE_FINISHED) ||
         (st == ESP_ASP_STATE_ERROR))) {
      xSemaphoreGive((SemaphoreHandle_t)ctx);
    }
  }
  return 0;
}

static int oai_record_read_enc(uint8_t *data, int data_size) {
  esp_gmf_data_bus_block_t blk;

  int ret = esp_gmf_fifo_acquire_read(oai_rec_enc_fifo, &blk, data_size,
                                      portMAX_DELAY);
  memcpy(data, blk.buf, data_size);
  esp_gmf_fifo_release_read(oai_rec_enc_fifo, &blk, 0);

  return ret;
}

static void oai_record_write_enc(uint8_t *data, int len) {
  esp_gmf_data_bus_block_t blk;

  esp_gmf_fifo_acquire_write(oai_rec_enc_fifo, &blk, len, portMAX_DELAY);
  memcpy((void *)blk.buf, (void *)data, len);
  blk.valid_size = len;
  if (len == 0) {
    blk.is_last = true;
  }
  esp_gmf_fifo_release_write(oai_rec_enc_fifo, &blk, portMAX_DELAY);
}

static int oai_encoder_acquire_write(void *handle,
                                     esp_gmf_data_bus_block_t *blk,
                                     uint32_t wanted_size, int block_ticks) {
  if (blk->buf) {
    return wanted_size;
  }
  return wanted_size;
}

static int oai_encoder_release_write(void *handle,
                                     esp_gmf_data_bus_block_t *blk,
                                     int block_ticks) {
  int ret = 0;
  if (blk->valid_size) {
    oai_record_write_enc(blk->buf, blk->valid_size);
    ret = blk->valid_size;
  }
  return ret;
}

static int oai_player_read_dec(uint8_t *data, int data_size, void *ctx) {
  esp_gmf_data_bus_block_t blk;

  int ret = esp_gmf_fifo_acquire_read(oai_plr_dec_fifo, &blk, data_size,
                                      portMAX_DELAY);
  memcpy(data, blk.buf, data_size);
  esp_gmf_fifo_release_read(oai_plr_dec_fifo, &blk, 0);

  return ret;
}

static void oai_player_write_dec(uint8_t *data, int len) {
  esp_gmf_data_bus_block_t blk;

  esp_gmf_fifo_acquire_write(oai_plr_dec_fifo, &blk, len, portMAX_DELAY);
  memcpy((void *)blk.buf, (void *)data, len);
  blk.valid_size = len;
  if (len == 0) {
    blk.is_last = true;
  }
  esp_gmf_fifo_release_write(oai_plr_dec_fifo, &blk, portMAX_DELAY);
}

static int oai_player_write_out(uint8_t *data, int data_size, void *ctx) {
  esp_codec_dev_handle_t dev = (esp_codec_dev_handle_t)ctx;
  esp_codec_dev_write(dev, data, data_size);
  return data_size;
}

static esp_gmf_err_t oai_record_pipeline_create(
    esp_gmf_pipeline_handle_t *pipe_rec, esp_gmf_pool_handle_t pool) {
  esp_gmf_err_t ret = ESP_GMF_ERR_OK;
  esp_gmf_pipeline_handle_t pipe = NULL;
  esp_gmf_task_handle_t work_rec_task = NULL;

  do {
    ret = esp_gmf_pool_new_pipeline(
        pool, "codec_dev_rx", (const char *[]){"encoder"}, 1, NULL, &pipe);
    if (ret != ESP_GMF_ERR_OK) {
      ESP_LOGE(TAG, "esp_gmf_pool_new_pipeline failed(0x%x)", ret);
      break;
    }

    esp_gmf_port_handle_t out_port;
    out_port = (esp_gmf_port_handle_t)NEW_ESP_GMF_PORT_OUT_BYTE(
        (void *)oai_encoder_acquire_write, (void *)oai_encoder_release_write,
        NULL, &out_port, 0, ESP_GMF_MAX_DELAY);
    ret = esp_gmf_pipeline_reg_el_port(pipe, "encoder", ESP_GMF_IO_DIR_WRITER,
                                       out_port);
    if (ret != ESP_GMF_ERR_OK) {
      ESP_LOGE(TAG, "esp_gmf_pipeline_reg_el_port failed(0x%x)", ret);
      break;
    }

    esp_gmf_task_cfg_t cfg_rec;
    cfg_rec.name = "rec_task";
    cfg_rec.thread.prio = DEFAULT_ESP_GMF_TASK_PRIO;
    cfg_rec.thread.stack_in_ext = false;
    cfg_rec.thread.stack = 30 * 1024;
    cfg_rec.thread.core = 1;

    ret = esp_gmf_task_init(&cfg_rec, &work_rec_task);
    if (ret != ESP_GMF_ERR_OK) {
      ESP_LOGE(TAG, "esp_gmf_task_init failed(0x%x)", ret);
      break;
    }

    esp_gmf_pipeline_bind_task(pipe, work_rec_task);
    esp_gmf_pipeline_loading_jobs(pipe);
    esp_gmf_pipeline_set_event(pipe, oai_record_event_callback, NULL);

    esp_gmf_element_handle_t enc_handle = NULL;
    esp_gmf_pipeline_get_el_by_name(pipe, "encoder", &enc_handle);

    esp_gmf_info_sound_t info;
    info.sample_rates = SAMPLE_RATE;
    info.channels = 1;
    info.bits = 16;
    esp_gmf_audio_helper_reconfig_enc_by_type(
        ESP_AUDIO_TYPE_OPUS, &info,
        (esp_audio_enc_config_t *)OBJ_GET_CFG(enc_handle));

    esp_audio_enc_config_t *cfg =
        (esp_audio_enc_config_t *)OBJ_GET_CFG(enc_handle);
    esp_opus_enc_config_t *opus_enc_cfg = (esp_opus_enc_config_t *)cfg->cfg;
    opus_enc_cfg->bitrate = 10000 * 3;
    opus_enc_cfg->enable_vbr = true;

    ret = esp_gmf_pipeline_report_info(pipe, ESP_GMF_INFO_SOUND, &info,
                                       sizeof(info));
    if (ret != ESP_GMF_ERR_OK) {
      ESP_LOGE(TAG, "esp_gmf_pipeline_report_info failed(0x%x)", ret);
      break;
    }

    *pipe_rec = pipe;
    return ret;
  } while (0);

  if (pipe) {
    esp_gmf_pipeline_stop(pipe);
    esp_gmf_pipeline_destroy(pipe);
  }
  if (work_rec_task) {
    esp_gmf_task_deinit(work_rec_task);
  }
  return ret;
}

void pool_register_codec_dev_io(esp_gmf_pool_handle_t pool, void *play_dev,
                                void *record_dev) {
  if (play_dev != NULL) {
    codec_dev_io_cfg_t tx_codec_dev_cfg = ESP_GMF_IO_CODEC_DEV_CFG_DEFAULT();
    tx_codec_dev_cfg.dir = ESP_GMF_IO_DIR_WRITER;
    tx_codec_dev_cfg.dev = play_dev;
    tx_codec_dev_cfg.name = "codec_dev_tx";
    esp_gmf_io_handle_t tx_dev = NULL;
    esp_gmf_io_codec_dev_init(&tx_codec_dev_cfg, &tx_dev);
    esp_gmf_pool_register_io(pool, tx_dev, NULL);
  }
  if (record_dev != NULL) {
    codec_dev_io_cfg_t rx_codec_dev_cfg = ESP_GMF_IO_CODEC_DEV_CFG_DEFAULT();
    rx_codec_dev_cfg.dir = ESP_GMF_IO_DIR_READER;
    rx_codec_dev_cfg.dev = record_dev;
    rx_codec_dev_cfg.name = "codec_dev_rx";
    esp_gmf_io_handle_t rx_dev = NULL;
    esp_gmf_io_codec_dev_init(&rx_codec_dev_cfg, &rx_dev);
    esp_gmf_pool_register_io(pool, rx_dev, NULL);
  }
}

void pool_register_audio_codecs(esp_gmf_pool_handle_t pool) {
  esp_audio_enc_register_default();
  esp_audio_enc_config_t es_enc_cfg = DEFAULT_ESP_GMF_AUDIO_ENC_CONFIG();
  esp_gmf_element_handle_t enc_handle = NULL;
  esp_gmf_audio_enc_init(&es_enc_cfg, &enc_handle);
  esp_gmf_pool_register_element(pool, enc_handle, NULL);
}

void oai_init_audio_capture(void) {
  /* Initialize speaker */
  oai_plr_handle = bsp_audio_codec_speaker_init();
  assert(oai_plr_handle);
  /* Speaker output volume */
  esp_codec_dev_set_out_vol(oai_plr_handle, 60);
  ESP_LOGI(TAG, "Initialize speaker:%p", oai_plr_handle);

  /* Initialize microphone */
  oai_rec_handle = bsp_audio_codec_microphone_init();
  assert(oai_rec_handle);
  /* Microphone input gain */
  esp_codec_dev_set_in_gain(oai_rec_handle, 50.0);
  ESP_LOGI(TAG, "Initialize microphone:%p", oai_rec_handle);

  esp_codec_dev_sample_info_t fs_record = {
      .bits_per_sample = 16,
      .channel = 1,
      .channel_mask = 0,
      .sample_rate = SAMPLE_RATE,
      .mclk_multiple = I2S_MCLK_MULTIPLE_384,
  };
  esp_codec_dev_open(oai_rec_handle, &fs_record);

  esp_codec_dev_sample_info_t fs_play = {
      .bits_per_sample = 16,
      .channel = 2,
      .channel_mask = 0,
      .sample_rate = SAMPLE_RATE,
      .mclk_multiple = I2S_MCLK_MULTIPLE_384,
  };
  esp_codec_dev_open(oai_plr_handle, &fs_play);
}

void oai_init_audio_decoder(void) {
  esp_err_t err = ESP_GMF_ERR_OK;
  esp_asp_handle_t player_handle = NULL;

  do {
    err = esp_gmf_fifo_create(10, 1, &oai_plr_dec_fifo);
    if (err != ESP_GMF_ERR_OK) {
      ESP_LOGE(TAG, "oai_plr_dec_fifo init failed (0x%x)", err);
      break;
    }

    esp_asp_cfg_t player_cfg;
    player_cfg.in.cb = oai_player_read_dec;
    player_cfg.in.user_ctx = oai_plr_dec_fifo;
    player_cfg.out.cb = oai_player_write_out;
    player_cfg.out.user_ctx = oai_plr_handle;
    player_cfg.task_prio = 5;
    player_cfg.task_stack = 30 * 1024;
    player_cfg.task_core = 1;

    err = esp_audio_simple_player_new(&player_cfg, &player_handle);
    if (err != ESP_GMF_ERR_OK) {
      ESP_LOGE(TAG, "simple_player init failed (0x%x)", err);
      break;
    }

    err = esp_audio_simple_player_set_event(player_handle,
                                            oai_player_event_callback, NULL);
    if (err != ESP_GMF_ERR_OK) {
      ESP_LOGE(TAG, "set_event failed (0x%x)", err);
      break;
    }

    esp_asp_music_info_t music_info;
    music_info.sample_rate = SAMPLE_RATE;
    music_info.channels = 2;
    music_info.bits = 16;
    music_info.bitrate = 0;

    err = esp_audio_simple_player_run(player_handle, "raw://sdcard/openAI.opus",
                                      &music_info);
    if (err != ESP_GMF_ERR_OK) {
      ESP_LOGE(TAG, "run failed (0x%x)", err);
      break;
    }

    return;
  } while (0);

  if (player_handle) {
    esp_audio_simple_player_destroy(player_handle);
  }
  if (oai_plr_dec_fifo) {
    esp_gmf_fifo_destroy(oai_plr_dec_fifo);
  }
}

void oai_audio_decode(uint8_t *data, size_t size) {
  oai_player_write_dec(data, size);
}

void oai_init_audio_encoder(void) {
  esp_err_t err = ESP_GMF_ERR_OK;
  esp_gmf_pool_handle_t pool_handle = NULL;

  do {
    err = esp_gmf_fifo_create(10, 1, &oai_rec_enc_fifo);
    if (err != ESP_GMF_ERR_OK) {
      ESP_LOGE(TAG, "oai_rec_enc_fifo init failed (0x%x)", err);
      break;
    }

    esp_gmf_pipeline_handle_t record_pipeline = NULL;
    err = esp_gmf_pool_init(&pool_handle);
    if (err != ESP_GMF_ERR_OK) {
      ESP_LOGE(TAG, "esp_gmf_pool_init failed (0x%x)", err);
      break;
    }

    pool_register_audio_codecs(pool_handle);
    pool_register_codec_dev_io(pool_handle, oai_plr_handle, oai_rec_handle);
    ESP_GMF_POOL_SHOW_ITEMS(pool_handle);

    err = oai_record_pipeline_create(&record_pipeline, pool_handle);
    if (err != ESP_GMF_ERR_OK) {
      ESP_LOGE(TAG, "oai_record_pipeline_create failed (0x%x)", err);
      break;
    }

    err = esp_gmf_pipeline_run(record_pipeline);
    if (err != ESP_GMF_ERR_OK) {
      ESP_LOGE(TAG, "esp_gmf_pipeline_run failed (0x%x)", err);
      break;
    }

    oai_encoder_input_buffer = (int16_t *)malloc(BUFFER_SAMPLES);
    if (!oai_encoder_input_buffer) {
      ESP_LOGE(TAG, "Failed to allocate memory for encoder input buffer");
      break;
    }

    return;
  } while (0);

  if (oai_rec_enc_fifo) {
    esp_gmf_fifo_destroy(oai_rec_enc_fifo);
  }
  if (pool_handle) {
    esp_gmf_pool_deinit(pool_handle);
  }
  if (oai_encoder_input_buffer) {
    free(oai_encoder_input_buffer);
  }
}

void oai_send_audio(PeerConnection *peer_connection) {
  int encoded_size =
      oai_record_read_enc((uint8_t *)oai_encoder_input_buffer, BUFFER_SAMPLES);
  peer_connection_send_audio(
      peer_connection, (const uint8_t *)oai_encoder_input_buffer, encoded_size);
}
#endif
