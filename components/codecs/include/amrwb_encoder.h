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

#ifndef _AMRWB_ENCODER_H_
#define _AMRWB_ENCODER_H_

#include "audio_codec.h"

#define AMRWB_ENCODER_TASK_STACK_SIZE    4*4096
#define AMRWB_ENCODER_TASK_PRIORITY      3

typedef struct  amrwb_encoder_t {
    audio_codec_t base;
    unsigned char *in_buf;
    unsigned char *out_buf;

    int mode;
    int allow_dtx;
    int eofFile;
    int Relens;

    void *handle; //Holds amr_codec interface functions and params.
    int pcmcnt;
    int framecnt;
    int total_bytes;
} amrwb_encoder_t;

void amrwb_encoder_set_offset(amrwb_encoder_t *codec, int offset);
void amrwb_encoder_set_stack_size(amrwb_encoder_t *codec, ssize_t stack_size);
amrwb_encoder_t *amrwb_encoder_create();
esp_err_t amrwb_encoder_destroy(amrwb_encoder_t *codec);

#endif