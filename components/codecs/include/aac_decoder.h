// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#ifndef _AAC_DECODER_H_
#define _AAC_DECODER_H_

#include <audio_codec.h>
#include <esp_audio_pm.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AAC_CODEC_TASK_STACK_SIZE    4096
#define AAC_CODEC_TASK_PRIORITY      3

typedef struct aac_codec {
    audio_codec_t base;
    void *pvmp4_decoder;
    uint8_t *decode_buf;
    uint8_t *in_buf;
    short *out_buf;
    void *aacdata; ///aac_callbacks_t
    void *audio_info; //audio_info_t *audio_info;
    void *user_data; ///aac_buffer or m4a_buffer
    uint8_t _run;
    int _audio_type;
    int offset; //offset in ms
    esp_audio_pm_lock_handle_t pm_lock;
    bool is_ts_stream;
    int pcmcnt;
    int framecnt;
    int _samplerate_idx;
    int _channels;
    int numSamples;
    int sampleId;
    int offsetIdx;
    int streampos;
    int rawdatausedbyte;
    uint32_t mFixedHeader;
    int transportFmt;
    int frame_length;
    int bits;
    int _current_sampling_freq;
    int _current_channels;
} aac_codec_t;

void aac_codec_set_stack_size(aac_codec_t *codec, ssize_t stack_size);
aac_codec_t *aac_codec_create();
esp_err_t aac_codec_destroy(aac_codec_t *codec);
#ifdef __cplusplus
}
#endif

#endif /* _MP3_DECODER_H_ */
