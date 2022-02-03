/*
*
* Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
*/

/*  ---------------------------------------------------------------------------------------
*   |                                                                                       |
*   |   The file includes functions and variables to configure AC101.                      |
*   |                                                                                       |
*   ----------------------------------------------------------------------------------------
*/
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <esp_codec.h>
#include <audio_board.h>

#define TAG "esp_codec_ac101"

#define AC101_DISABLE_MUTE 0x00   //disable mute
#define AC101_ENABLE_MUTE  0x01   //enable  mute

#define AC101_DEFAULT_VOL 45

#define AC101_I2C_MASTER_SPEED 100000  //set master clk speed to 100k

#define AC_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(TAG, format, ##__VA_ARGS__); \
        return b;\
    }

#define LOG_AC101(fmt, ...)   ESP_LOGW(TAG, fmt, ##__VA_ARGS__)

uint8_t curr_vol = 0;

/**
 * @brief Initialization function for i2c
 */
static esp_err_t audio_codec_i2c_init(int i2c_master_port)
{
    int res;
    i2c_config_t pf_i2c_pin = {0};

    res = audio_board_i2c_pin_config(i2c_master_port, &pf_i2c_pin);

    pf_i2c_pin.mode = I2C_MODE_MASTER;
	pf_i2c_pin.sda_pullup_en = GPIO_PULLUP_ENABLE,
	pf_i2c_pin.scl_pullup_en = GPIO_PULLUP_ENABLE,
    pf_i2c_pin.master.clk_speed = AC101_I2C_MASTER_SPEED;

    res |= i2c_param_config(i2c_master_port, &pf_i2c_pin);
    res |= i2c_driver_install(i2c_master_port, pf_i2c_pin.mode, 0, 0, 0);
    return res;
}

/**
 * @brief Write AC101 register
 *
 * @param slave_addr : slave address
 * @param reg_addr    : register address
 * @param data      : data to write
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
static esp_err_t ac101_write_reg(uint8_t slave_addr, uint8_t reg_addr, uint16_t data)
{
    int res = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	uint8_t send_buff[4];
	send_buff[0] = (slave_addr << 1);
	send_buff[1] = reg_addr;
	send_buff[2] = (data >> 8) & 0xff;
	send_buff[3] = data & 0xff;
    res |= i2c_master_start(cmd);
	ret |= i2c_master_write(cmd, send_buff, 4, ACK_CHECK_EN);
    res |= i2c_master_stop(cmd);
    res |= i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    AC_ASSERT(res, "ac101_write_reg error", -1);
    return res;
}

/**
 * @brief Read AC101 register
 *
 * @param reg_addr    : register address
 *
 * @return
 *     - (-1)     Error
 *     - (0)      Success
 */
static esp_err_t ac101_read_reg(uint8_t reg_addr, uint16_t *p_data)
{
    uint16_t val = 0;
    uint8_t data_rd[2];
    esp_err_t res;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    res  = i2c_master_start(cmd);
    res |= i2c_master_write_byte(cmd, (AC101_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    res |= i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    res |= i2c_master_start(cmd);
    res |= i2c_master_write_byte(cmd, (AC101_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    res |= i2c_master_read_byte(cmd, &data_rd, ACK_VAL);
    res |= i2c_master_stop(cmd);
    res |= i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    AC_ASSERT(res, "ac101_read_reg error", -1);
    val = (data_rd[0] << 8) + data_rd[1];
    *p_data = val;
    return res;
}

// /**
//  * @brief Configure AC101 ADC and DAC volume. Basicly you can consider this as ADC and DAC gain
//  *
//  * @param mode:             set ADC or DAC or all
//  * @param volume:           -96 ~ 0              for example ac101_set_adc_dac_volume(AC101_MODULE_ADC, 30, 6); means set ADC volume -30.5db
//  * @param dot:              whether include 0.5. for example ac101_set_adc_dac_volume(AC101_MODULE_ADC, 30, 4); means set ADC volume -30db
//  *
//  * @return
//  *     - (-1) Parameter error
//  *     - (0)   Success
//  */
// static esp_err_t ac101_set_adc_dac_volume(media_hal_codec_mode_t mode, float volume)
// {
//     esp_err_t res = 0;
//     uint8_t vol;
//     if ( volume < -96 || volume > 0 ) {
//         LOG_AC101("Warning: volume < -96! or > 0!\n");
//         if (volume < -96)
//             volume = -96;
//         else
//             volume = 0;
//     }
//     vol = (uint8_t) ((-1) * (volume * 2));

//     if (mode == MEDIA_HAL_CODEC_MODE_ENCODE || mode == MEDIA_HAL_CODEC_MODE_BOTH) {
//         res  = ac101_write_reg(AC101_ADDR, AC101_ADCCONTROL8, vol);
//         res |= ac101_write_reg(AC101_ADDR, AC101_ADCCONTROL9, vol);  //ADC Right Volume=0db
//     }
//     if (mode == MEDIA_HAL_CODEC_MODE_DECODE || mode == MEDIA_HAL_CODEC_MODE_BOTH) {
//         res  = ac101_write_reg(AC101_ADDR, AC101_DACCONTROL5, vol);
//         res |= ac101_write_reg(AC101_ADDR, AC101_DACCONTROL4, vol);
//     }
//     return res;
// }

esp_err_t ac101_set_state(media_hal_codec_mode_t mode, media_hal_sel_state_t media_hal_state)
{
    esp_err_t res = 0;
    uint8_t reg = 0;
    if(media_hal_state == MEDIA_HAL_START_STATE) {
        if (mode == MEDIA_HAL_CODEC_MODE_LINE_IN) {
            res |= ac101_write_reg(AC101_ADDR, AC101_ADC_SRC, 0x0408);
            res |= ac101_write_reg(AC101_ADDR, AC101_ADC_DIG_CTRL, 0x8000);
            res |= ac101_write_reg(AC101_ADDR, AC101_ADC_APC_CTRL, 0x3bc0);
        }
        if (mode == MEDIA_HAL_CODEC_MODE_ENCODE || mode == MEDIA_HAL_CODEC_MODE_BOTH || mode == MEDIA_HAL_CODEC_MODE_LINE_IN) {
            // I2S1_SDOUT_CTRL
            // res |= ac101_write_reg(AC101_ADDR, AC101_PLL_CTRL2, 0x8120);
            res |= ac101_write_reg(AC101_ADDR, AC101_MOD_CLK_ENA, 0x800c);
            res |= ac101_write_reg(AC101_ADDR, AC101_MOD_RST_CTRL, 0x800c);
            // res |= ac101_write_reg(AC101_ADDR, AC101_I2S_SR_CTRL, 0x3000);
        }
        if (mode == MEDIA_HAL_CODEC_MODE_DECODE || mode == MEDIA_HAL_CODEC_MODE_BOTH || mode == MEDIA_HAL_CODEC_MODE_LINE_IN) {
            //* Enable Headphone output
            res |= ac101_write_reg(AC101_ADDR, AC101_OMIXER_DACA_CTRL, 0xff80);
            res |= ac101_write_reg(AC101_ADDR, AC101_HPOUT_CTRL, 0xc3c1);
            res |= ac101_write_reg(AC101_ADDR, AC101_HPOUT_CTRL, 0xcb00);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            res |= ac101_write_reg(AC101_ADDR, AC101_HPOUT_CTRL, 0xfbc0);

            //* Enable Speaker output
            res |= ac101_write_reg(AC101_ADDR, AC101_SPKOUT_CTRL, 0xeabd);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            res |= ac101_control_volume(30);
        }
        return res;
    }
    if(media_hal_state == MEDIA_HAL_STOP_STATE) {
        res |= ac101_write_reg(AC101_ADDR, AC101_HPOUT_CTRL, 0x0001);   // disable earphone
        res |= ac101_write_reg(AC101_ADDR, AC101_SPKOUT_CTRL, 0xe880);   // disable speaker
        res |= ac101_control_volume(0);
        return res;
    }
    return res;
}

esp_err_t ac101_deinit(int port_num)
{
    esp_err_t ret = 0;
    //XXX: Simply returning since deinit causes noise
    /*
    ret = ac101_write_reg(AC101_ADDR, AC101_CHIPPOWER, 0xFF);  //reset and stop ac101
    gpio_set_level(GPIO_PA_EN, 0);
    i2c_driver_delete(port_num);
    */
    return ret;
}

esp_err_t ac101_powerup()
{
    esp_err_t ret = 0;
    gpio_set_level(GPIO_PA_EN, 1);
    return ret;
}

esp_err_t ac101_powerdown()
{
    esp_err_t ret = 0;
    gpio_set_level(GPIO_PA_EN, 0);
    return ret;
}

esp_err_t ac101_init(media_hal_config_t *media_hal_conf)
{
    ESP_LOGI(TAG, "Initialising esp_codec");
    media_hal_op_mode_t ac101_mode = media_hal_conf->op_mode;
    media_hal_adc_input_t ac101_adc_input = media_hal_conf->adc_input;
    media_hal_dac_output_t ac101_dac_output = media_hal_conf->dac_output;
    int port_num = media_hal_conf->port_num;

    esp_err_t res;

    audio_codec_i2c_init(port_num);   //set i2c pin and i2c clock frequency for esp32

#ifndef AC101_DISABLE_PA_PIN
    gpio_config_t  io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_SEL_PA_EN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_PA_EN, 1);
#endif

    res = ac101_write_reg(AC101_ADDR, AC101_CHIP_AUDIO_RS, 0x123);
    vTashDelay(1000 / portTICK_PERIOD_MS);

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "AC101 reset failed!");
        return res;
    } else {
        ESP_LOGW(TAG, "AC101 reset succeed");
    }
    
    res |= ac101_write_reg(AC101_ADDR, AC101_SPKOUT_CTRL, 0xe880);

	//Enable the PLL from 256*44.1KHz MCLK source
	res |= ac101_write_reg(AC101_ADDR, AC101_PLL_CTRL1, 0x014f);
	//res |= ac101_write_reg(AC101_ADDR, AC101_PLL_CTRL2, 0x83c0);
	res |= ac101_write_reg(AC101_ADDR, AC101_PLL_CTRL2, 0x8600);

	//Clocking system
	res |= ac101_write_reg(AC101_ADDR, AC101_SYSCLK_CTRL, 0x8b08);
	res |= ac101_write_reg(AC101_ADDR, AC101_MOD_CLK_ENA, 0x800c);
	res |= ac101_write_reg(AC101_ADDR, AC101_MOD_RST_CTRL, 0x800c);
	res |= ac101_write_reg(AC101_ADDR, AC101_I2S_SR_CTRL, 0x7000); //sample rate

	//AIF config
	res |= ac101_write_reg(AC101_ADDR, AC101_I2S1LCK_CTRL, 0x8850);	//BCLK/LRCK
	res |= ac101_write_reg(AC101_ADDR, AC101_I2S1_SDOUT_CTRL, 0xc000); //
	res |= ac101_write_reg(AC101_ADDR, AC101_I2S1_SDIN_CTRL, 0xc000);
	res |= ac101_write_reg(AC101_ADDR, AC101_I2S1_MXR_SRC, 0x2200); //

	res |= ac101_write_reg(AC101_ADDR, AC101_ADC_SRCBST_CTRL, 0xccc4);
	res |= ac101_write_reg(AC101_ADDR, AC101_ADC_SRC, 0x2020);
	res |= ac101_write_reg(AC101_ADDR, AC101_ADC_DIG_CTRL, 0x8000);
	res |= ac101_write_reg(AC101_ADDR, AC101_ADC_APC_CTRL, 0xbbc3);

	//Path Configuration
	res |= ac101_write_reg(AC101_ADDR, AC101_DAC_MXR_SRC, 0xcc00);
	res |= ac101_write_reg(AC101_ADDR, AC101_DAC_DIG_CTRL, 0x8000);
	res |= ac101_write_reg(AC101_ADDR, AC101_OMIXER_SR, 0x0081);
	res |= ac101_write_reg(AC101_ADDR, AC101_OMIXER_DACA_CTRL, 0xf080); //}

	//* Enable Speaker output
	res |= ac101_write_reg(AC101_ADDR, AC101_SPKOUT_CTRL, 0xeabd);

	ESP_LOGI(TAG, "init done");
	ac101_powerup();

    return res;
}














// 2022-02-03....
esp_err_t ac101_config_format(media_hal_codec_mode_t mode, media_hal_format_t fmt)
{
    esp_err_t res = 0;
    uint8_t reg = 0;
    if (mode == MEDIA_HAL_CODEC_MODE_ENCODE || mode == MEDIA_HAL_CODEC_MODE_BOTH) {
        res = ac101_read_reg(AC101_ADCCONTROL4, &reg);
        reg = reg & 0xfc;
        res |= ac101_write_reg(AC101_ADDR, AC101_ADCCONTROL4, reg | fmt);
    }
    if (mode == MEDIA_HAL_CODEC_MODE_DECODE || mode == MEDIA_HAL_CODEC_MODE_BOTH) {
        res = ac101_read_reg(AC101_DACCONTROL1, &reg);
        reg = reg & 0xf9;
        res |= ac101_write_reg(AC101_ADDR, AC101_DACCONTROL1, reg | (fmt << 1));
    }
    return res;
}

static uint16_t ac101_get_spk_volume(void) 
{
    uint16_t reg = 0;
    ac101_read_reg(AC101_SPKOUT_CTRL, &reg);
    reg &= 0x1f;    // volume : 0 ~ 31
    return (reg * 2);
}

static esp_err_t ac101_set_spk_volume(uint8_t volume) 
{
    esp_err_t res = 0;
    uint16_t reg = 0;

    if (volume > 0x3f) {
        volume = 0x3f;
    }
    volume = volume / 2;

    res |= ac101_read_reg(AC101_SPKOUT_CTRL, &reg);
    reg &= (~0x1f);
    volume &= 0x1f;
    reg |= volume;
    res |= ac101_write_reg(AC101_ADDR, AC101_SPKOUT_CTRL, reg);

    return res;
}

static uint16_t ac101_get_earph_volume(void) 
{
    uint16_t reg = 0;
    ac101_read_reg(AC101_HPOUT_CTRL, &reg);
    return (reg >> 4) & 0x3f;
}

static esp_err_t ac101_set_earph_volume(uint8_t volume) 
{
    esp_err_t res = 0;
    uint16_t tmp, reg = 0;

    if (volume > 0x3f) {
        volume = 0x3f;
    }

    res |= ac101_read_reg(AC101_HPOUT_CTRL, &reg);
    tmp =~(0x3f << 4);
    reg &= tmp;
    volume &= 0x3f;
    reg |= (volume << 4);
    res |= ac101_write_reg(AC101_ADDR, AC101_HPOUT_CTRL, reg);

    return res;
}

esp_err_t ac101_control_volume(uint8_t volume)
{
    esp_err_t res = 0;
    res |= ac101_set_spk_volume(volume);
    res |= ac101_set_earph_volume(volume);
    return res;
}

esp_err_t ac101_get_volume(uint8_t *volume)
{
    esp_err_t res = 0;
    *volume = (uint8_t) ac101_get_earph_volume();
    return res;
}

esp_err_t ac101_set_mute(bool bmute)
{
    esp_err_t res = 0;

    if (bmute) {
        res |= ac101_set_spk_volume(0);
        reg |= ac101_set_earph_volume(0);
    }
    return res;
}

/**
 * @brief Configure AC101 data sample bits
 *
 * @param mode:             set ADC or DAC or all
 * @param bitPerSample:   see BitsLength
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int ac101_set_bits_per_sample(es_module_t mode, es_bits_length_t bits_length)
{
    int res = 0;
    uint8_t reg = 0;
    int bits = (int) bits_length;

    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res = ac101_read_reg(AC101_ADCCONTROL4, &reg);
        reg = reg & 0xe3;
        res |=  ac101_write_reg(AC101_ADDR, AC101_ADCCONTROL4, reg | (bits << 2));
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res = ac101_read_reg(AC101_DACCONTROL1, &reg);
        reg = reg & 0xc7;
        res |= ac101_write_reg(AC101_ADDR, AC101_DACCONTROL1, reg | (bits << 3));
    }
    return res;
}

esp_err_t ac101_set_mic_gain(ac101_mic_gain_t gain)
{
    esp_err_t ret;
    int gain_n;
    gain_n = (int)gain / 3;
    ret = ac101_write_reg(AC101_ADDR, AC101_ADCCONTROL1, gain_n); //MIC PGA
    return ret;
}

void ac101_read_all_registers()
{
    for (int i = 0; i < 50; i++) {
        uint8_t reg = 0;
        ac101_read_reg(i, &reg);
        ets_printf("%x: %x\n", i, reg);
    }
}

esp_err_t ac101_write_register(uint8_t reg_addr, uint16_t data)
{
    return ac101_write_reg(AC101_ADDR, reg_addr, data);
}

esp_err_t ac101_set_i2s_clk(media_hal_codec_mode_t media_hal_codec_mode, media_hal_bit_length_t media_hal_bit_length)
{
    int clk_div = 2, tmp = 0;
    esp_err_t ret = ESP_OK;

    switch (media_hal_bit_length) {
        case MEDIA_HAL_BIT_LENGTH_16BITS:
            tmp = BIT_LENGTH_16BITS;
            clk_div = 3;
        break;
        case MEDIA_HAL_BIT_LENGTH_18BITS:
            tmp = BIT_LENGTH_18BITS;
        break;
        case MEDIA_HAL_BIT_LENGTH_20BITS:
            tmp = BIT_LENGTH_20BITS;
        break;
        case MEDIA_HAL_BIT_LENGTH_24BITS:
            tmp = BIT_LENGTH_24BITS;
        break;
        default:
            tmp = BIT_LENGTH_32BITS;
            clk_div = 3;
    }

    ret |= ac101_set_bits_per_sample(ES_MODULE_ADC_DAC, tmp);
    ret |= ac101_write_reg(AC101_ADDR, AC101_ADCCONTROL5, clk_div);  //ADCFsMode,singel SPEED,RATIO=256
    ret |= ac101_write_reg(AC101_ADDR, AC101_DACCONTROL2, clk_div);  //ADCFsMode,singel SPEED,RATIO=256
    return ret;
}
