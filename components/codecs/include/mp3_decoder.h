// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#ifndef _MP3_DECODER_H_
#define _MP3_DECODER_H_

#include <audio_codec.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MP3_CODEC_TASK_STACK_SIZE    (8 * 1024)
#define MP3_CODEC_TASK_PRIORITY      3

typedef struct mp3_codec {
    audio_codec_t base;

    void *pvmp3_decoder;
    uint8_t *decode_buf;

    uint8_t *in_buf;
    uint8_t *out_buf;
    uint8_t _run;
    int _skip_id3;
    int _current_sampling_freq;
    int _current_channels;
    int offset_in_ms; //offset in ms for seek
} mp3_codec_t;

mp3_codec_t *mp3_codec_create();
esp_err_t mp3_codec_destroy(mp3_codec_t *codec);
void mp3_codec_set_stack_size(mp3_codec_t *codec, ssize_t stack_size);

#ifdef __cplusplus
}
#endif

#endif /* _MP3_DECODER_H_ */
