// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _WAV_DECODER_H_
#define _WAV_DECODER_H_

#include <audio_codec.h>

#define WAV_DECODER_TASK_STACK_SIZE     4096
#define WAV_DECODER_TASK_PRIORITY       3

typedef struct wav_decoder {
    audio_codec_t base;
    void *handle;
    unsigned int inbuf_size;
    unsigned char *in_buf;
    int pcmcnt;
    int _current_sampling_freq;
    int _current_channels;
    bool header_parsed;
} wav_decoder_t;

//void wav_decoder_set_offset(wav_decoder_t *codec, int offset);
void wav_decoder_set_stack_size(wav_decoder_t *codec, ssize_t stack_size);
wav_decoder_t *wav_decoder_create();
esp_err_t wav_decoder_destroy(wav_decoder_t *codec);

#endif /* _WAV_DECODER_H_ */
