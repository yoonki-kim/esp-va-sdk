#ifndef __AC101_H__
#define __AC101_H__

#include "esp_types.h"
#include "driver/i2c.h"
#include "media_hal.h"
#include "esxxx_common.h"

#define AC101_ADDR			0x1a				/*!< Device address*/

#define AC101_WRITE_BIT  			I2C_MASTER_WRITE 	/*!< I2C master write */
#define AC101_READ_BIT   			I2C_MASTER_READ  	/*!< I2C master read */
#define AC101_ACK_CHECK_EN   		0x1     			/*!< I2C master will check ack from slave*/
#define AC101_ACK_CHECK_DIS  		0x0     			/*!< I2C master will not check ack from slave */
#define AC101_ACK_VAL    			0x0         		/*!< I2C ack value */
#define AC101_NACK_VAL   			0x1         		/*!< I2C nack value */

#define AC101_CHIP_AUDIO_RS		0x00
#define AC101_PLL_CTRL1			0x01
#define AC101_PLL_CTRL2			0x02
#define AC101_SYSCLK_CTRL		0x03
#define AC101_MOD_CLK_ENA		0x04
#define AC101_MOD_RST_CTRL		0x05
#define AC101_I2S_SR_CTRL		0x06
#define AC101_I2S1LCK_CTRL		0x10
#define AC101_I2S1_SDOUT_CTRL	0x11
#define AC101_I2S1_SDIN_CTRL	0x12
#define AC101_I2S1_MXR_SRC		0x13
#define AC101_I2S1_VOL_CTRL1	0x14
#define AC101_I2S1_VOL_CTRL2	0x15
#define AC101_I2S1_VOL_CTRL3	0x16
#define AC101_I2S1_VOL_CTRL4	0x17
#define AC101_I2S1_MXR_GAIN		0x18
#define AC101_ADC_DIG_CTRL		0x40
#define AC101_ADC_VOL_CTRL		0x41
#define AC101_HMIC_CTRL1		0x44
#define AC101_HMIC_CTRL2		0x45
#define AC101_HMIC_STATUS		0x46
#define AC101_DAC_DIG_CTRL		0x48
#define AC101_DAC_VOL_CTRL		0x49
#define AC101_DAC_MXR_SRC		0x4c
#define AC101_DAC_MXR_GAIN		0x4d
#define AC101_ADC_APC_CTRL		0x50
#define AC101_ADC_SRC			0x51
#define AC101_ADC_SRCBST_CTRL	0x52
#define AC101_OMIXER_DACA_CTRL	0x53
#define AC101_OMIXER_SR			0x54
#define AC101_OMIXER_BST1_CTRL	0x55
#define AC101_HPOUT_CTRL		0x56
#define AC101_SPKOUT_CTRL		0x58
#define AC101_AC_DAC_DAPCTRL	0xa0
#define AC101_AC_DAC_DAPHHPFC 	0xa1
#define AC101_AC_DAC_DAPLHPFC 	0xa2
#define AC101_AC_DAC_DAPLHAVC 	0xa3
#define AC101_AC_DAC_DAPLLAVC 	0xa4
#define AC101_AC_DAC_DAPRHAVC 	0xa5
#define AC101_AC_DAC_DAPRLAVC 	0xa6
#define AC101_AC_DAC_DAPHGDEC 	0xa7
#define AC101_AC_DAC_DAPLGDEC 	0xa8
#define AC101_AC_DAC_DAPHGATC 	0xa9
#define AC101_AC_DAC_DAPLGATC 	0xaa
#define AC101_AC_DAC_DAPHETHD 	0xab
#define AC101_AC_DAC_DAPLETHD 	0xac
#define AC101_AC_DAC_DAPHGKPA 	0xad
#define AC101_AC_DAC_DAPLGKPA 	0xae
#define AC101_AC_DAC_DAPHGOPA 	0xaf
#define AC101_AC_DAC_DAPLGOPA 	0xb0
#define AC101_AC_DAC_DAPOPT   	0xb1
#define AC101_DAC_DAP_ENA     	0xb5

typedef enum{
	SAMPLE_RATE_8000	= 0x0000,
	SAMPLE_RATE_11052	= 0x1000,
	SAMPLE_RATE_12000	= 0x2000,
	SAMPLE_RATE_16000	= 0x3000,
	SAMPLE_RATE_22050	= 0x4000,
	SAMPLE_RATE_24000	= 0x5000,
	SAMPLE_RATE_32000	= 0x6000,
	SAMPLE_RATE_44100	= 0x7000,
	SAMPLE_RATE_48000	= 0x8000,
	SAMPLE_RATE_96000	= 0x9000,
	SAMPLE_RATE_192000	= 0xa000,
}ac_adda_fs_i2s1_t;

typedef enum{
	BCLK_DIV_1		= 0x0,
	BCLK_DIV_2		= 0x1,
	BCLK_DIV_4		= 0x2,
	BCLK_DIV_6		= 0x3,
	BCLK_DIV_8		= 0x4,
	BCLK_DIV_12		= 0x5,
	BCLK_DIV_16		= 0x6,
	BCLK_DIV_24		= 0x7,
	BCLK_DIV_32		= 0x8,
	BCLK_DIV_48		= 0x9,
	BCLK_DIV_64		= 0xa,
	BCLK_DIV_96		= 0xb,
	BCLK_DIV_128	= 0xc,
	BCLK_DIV_192	= 0xd,

}ac_i2s1_bclk_div_t;

typedef enum{
	LRCK_DIV_16		=0x0,
	LRCK_DIV_32		=0x1,
	LRCK_DIV_64		=0x2,
	LRCK_DIV_128	=0x3,
	LRCK_DIV_256	=0x4,
}ac_i2s1_lrck_div_t;

typedef enum {
    BIT_LENGTH_8_BITS = 0x00,
    BIT_LENGTH_16_BITS = 0x01,
    BIT_LENGTH_20_BITS = 0x02,
    BIT_LENGTH_24_BITS = 0x03,
} ac_bits_length_t;

typedef enum {
    AC_MODE_MIN = -1,
    AC_MODE_SLAVE = 0x00,
    AC_MODE_MASTER = 0x01,
    AC_MODE_MAX,
} ac_mode_sm_t;

typedef enum {
    AC_MODULE_MIN = -1,
    AC_MODULE_ADC = 0x01,
    AC_MODULE_DAC = 0x02,
    AC_MODULE_ADC_DAC = 0x03,
    AC_MODULE_LINE = 0x04,
    AC_MODULE_MAX
} ac_module_t;

typedef enum{
	SRC_MIC1	= 1,
	SRC_MIC2	= 2,
	SRC_LINEIN	= 3,
}ac_output_mixer_source_t;

typedef enum {
    GAIN_N45DB = 0,
    GAIN_N30DB = 1,
    GAIN_N15DB = 2,
    GAIN_0DB   = 3,
    GAIN_15DB  = 4,
    GAIN_30DB  = 5,
    GAIN_45DB  = 6,
    GAIN_60DB  = 7,
} ac_output_mixer_gain_t;

/**
 * @brief Configure AC101 clock
 */
typedef struct {
	ac_i2s1_bclk_div_t bclk_div;    /*!< bits clock divide */
	ac_i2s1_lrck_div_t lclk_div;    /*!< WS clock divide */
} ac_i2s_clock_t;


esp_err_t ac101_init(audio_hal_codec_config_t* codec_cfg);
esp_err_t ac101_deinit(void);
esp_err_t ac101_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state);
esp_err_t ac101_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t* iface);
esp_err_t ac101_set_voice_mute(bool enable);
esp_err_t ac101_set_voice_volume(int volume);
esp_err_t ac101_get_voice_volume(int* volume);
void ac101_pa_power(bool enable);






/*Enable pin for ac101*/
#define GPIO_PA_EN           GPIO_NUM_21
#define GPIO_SEL_PA_EN       GPIO_SEL_21

/* AC101 address */
// #define AC101_ADDR 0x20  // 0x22:CE=1;0x20:CE=0

/* AC101 register */
#define AC101_CONTROL1         0x00
#define AC101_CONTROL2         0x01
/* Power control registers */
#define AC101_CHIPPOWER        0x02
#define AC101_ADCPOWER         0x03
#define AC101_DACPOWER         0x04
#define AC101_CHIPLOPOW1       0x05
#define AC101_CHIPLOPOW2       0x06
#define AC101_ANAVOLMANAG      0x07
#define AC101_MASTERMODE       0x08
/* ADC */
#define AC101_ADCCONTROL1      0x09
#define AC101_ADCCONTROL2      0x0a
#define AC101_ADCCONTROL3      0x0b
#define AC101_ADCCONTROL4      0x0c
#define AC101_ADCCONTROL5      0x0d
#define AC101_ADCCONTROL6      0x0e
#define AC101_ADCCONTROL7      0x0f
#define AC101_ADCCONTROL8      0x10
#define AC101_ADCCONTROL9      0x11
#define AC101_ADCCONTROL10     0x12
#define AC101_ADCCONTROL11     0x13
#define AC101_ADCCONTROL12     0x14
#define AC101_ADCCONTROL13     0x15
#define AC101_ADCCONTROL14     0x16
/* DAC */
#define AC101_DACCONTROL1      0x17
#define AC101_DACCONTROL2      0x18
#define AC101_DACCONTROL3      0x19
#define AC101_DACCONTROL4      0x1a
#define AC101_DACCONTROL5      0x1b
#define AC101_DACCONTROL6      0x1c
#define AC101_DACCONTROL7      0x1d
#define AC101_DACCONTROL8      0x1e
#define AC101_DACCONTROL9      0x1f
#define AC101_DACCONTROL10     0x20
#define AC101_DACCONTROL11     0x21
#define AC101_DACCONTROL12     0x22
#define AC101_DACCONTROL13     0x23
#define AC101_DACCONTROL14     0x24
#define AC101_DACCONTROL15     0x25
#define AC101_DACCONTROL16     0x26
#define AC101_DACCONTROL17     0x27
#define AC101_DACCONTROL18     0x28
#define AC101_DACCONTROL19     0x29
#define AC101_DACCONTROL20     0x2a
#define AC101_DACCONTROL21     0x2b
#define AC101_DACCONTROL22     0x2c
#define AC101_DACCONTROL23     0x2d
#define AC101_DACCONTROL24     0x2e
#define AC101_DACCONTROL25     0x2f
#define AC101_DACCONTROL26     0x30
#define AC101_DACCONTROL27     0x31
#define AC101_DACCONTROL28     0x32
#define AC101_DACCONTROL29     0x33
#define AC101_DACCONTROL30     0x34


typedef enum {
    AC101_BIT_LENGTH_MIN = -1,
    AC101_BIT_LENGTH_16BITS = 0x03,
    AC101_BIT_LENGTH_18BITS = 0x02,
    AC101_BIT_LENGTH_20BITS = 0x01,
    AC101_BIT_LENGTH_24BITS = 0x00,
    AC101_BIT_LENGTH_32BITS = 0x04,
    AC101_BIT_LENGTH_MAX,
} ac101_bit_length_t;

typedef enum {
    AC101_SAMPLE_RATE_MIN = -1,
    AC101_SAMPLE_RATE_16K,
    AC101_SAMPLE_RATE_32K,
    AC101_SAMPLE_RATE_44_1K,
    AC101_SAMPLE_RATE_MAX,
} ac101_sample_rate_t;

/**
 * @brief Select adc channel for input mic signal
 */
typedef enum {
    AC101_ADC_INPUT_LINE1 = 0x00,       //mic input to adc channel 1
    AC101_ADC_INPUT_LINE2 = 0x50,       //mic input to adc channel 2
    AC101_ADC_INPUT_DIFFERENCE = 0xf0,
} ac101_adc_input_t;

/**
 * @brief Select dac channel for output voice signal
 */
typedef enum {
    AC101_DAC_OUTPUT_LINE1 = 0x14,  //voice output from dac channel 1
    AC101_DAC_OUTPUT_LINE2 = 0x28,  //voice output from dac channel 2
    AC101_DAC_OUTPUT_ALL = 0x3c,
    AC101_DAC_OUTPUT_MAX,
} ac101_dac_output_t;

/**
 * @brief Select microphone gain for ac101
 */
typedef enum {
    AC101_MIC_GAIN_0DB = 0,    //microphone gain set to 00DB
    AC101_MIC_GAIN_3DB = 3,    //microphone gain set to 03DB
    AC101_MIC_GAIN_6DB = 6,    //microphone gain set to 06DB
    AC101_MIC_GAIN_9DB = 9,    //microphone gain set to 09DB
    AC101_MIC_GAIN_12DB = 12,  //microphone gain set to 12DB
    AC101_MIC_GAIN_15DB = 15,  //microphone gain set to 15DB
    AC101_MIC_GAIN_18DB = 18,  //microphone gain set to 18DB
    AC101_MIC_GAIN_21DB = 21,  //microphone gain set to 21DB
    AC101_MIC_GAIN_24DB = 24,  //microphone gain set to 24DB
    AC101_MIC_GAIN_MAX,
} ac101_mic_gain_t;

/**
 * @brief Select AC101 working module
 */
typedef enum {
    AC101_MODULE_ADC = 0x01,  //select adc mode
    AC101_MODULE_DAC,         //select dac mode
    AC101_MODULE_ADC_DAC,     //select both, adc and dac mode
    AC101_MODULE_LINE,        //select line mode
} ac101_module_t;

/**
 * @brief Select mode for AC101
 */
typedef enum {
    AC101_MODE_SLAVE = 0x00,   //set ac101 in slave mode
    AC101_MODE_MASTER = 0x01,  //set ac101 in master mode
} ac101_mode_t;

/**
 * @brief Select i2s format for AC101
 */
typedef enum {
    AC101_I2S_NORMAL = 0,  //i2s format
    AC101_I2S_LEFT,        //left justified
    AC101_I2S_RIGHT,       //right justified
    AC101_I2S_DSP,         //pcm/dsp format
    AC101_I2S_MAX
} ac101_i2s_format_t;

typedef struct {
    ac101_mode_t esMode;
    ac101_dac_output_t dacOutput;
    ac101_adc_input_t adcInput;
} ac101_config_t;

typedef enum {
    AC101_SEL_DAC_CH_0,
    AC101_SEL_DAC_CH_1,
} ac101_sel_dac_ch_t;

/**
 * @brief Initialize ac101 audio codec
 *
 * @param ac101_mode set ac101 in master or slave mode
 * @param ac101_adc_input select adc input channel
 * @param ac101_dac_output select dac output channel
 *
 * @return     int, 0--success, others--fail
 */
esp_err_t ac101_init(media_hal_config_t *media_hal_conf);

/**
 * @brief De-initialize ac101 audio codec
 *
 * @param port_num i2c port number
 *
 * @return     int, 0--success, others--fail
 */
esp_err_t ac101_deinit(int port_num);

/**
 * @brief Configure ac101 data format either I2S, PCM or left/right justified
 *
 * @param mode select mode for ac101 either ADC, DAC, both ADC and DAC or LINE
 * @param fmt  select audio data format
 *
 * @return     int, 0--success, others--fail
 */
esp_err_t ac101_config_format(media_hal_codec_mode_t mode, media_hal_format_t fmt);

/**
 * @brief Set I2S clock for ac101. In slave mode the SCLK and LRCLK frequencies are auto-detected
 *
 * @param mode select mode for ac101 either ADC, DAC, both ADC and DAC or LINE
 * @param rate set frequency
 * @param media_hal_bit_length set bit length for each audio sample
 *
 * @return     int, 0--success, others--fail
 */
esp_err_t ac101_set_i2s_clk(media_hal_codec_mode_t mode, media_hal_bit_length_t media_hal_bit_length);

/**
 * @brief Set bit per sample of audio data
 *
 * @param mode select mode for ac101 either ADc, DAC, both ADC and DAC or LINE
 * @param bits_per_sample set bit length for each audio sample
 *
 * @return     int, 0--success, others--fail
 */
esp_err_t ac101_set_bits_per_sample(es_module_t mode, es_bits_length_t bits_per_sample);

/**
 * @brief Start/stop selected mode of ac101
 *
 * @param mode select mode for ac101 either ADc, DAC, both ADC and DAC or LINE
 * @param media_hal_state select start stop state for specific mode
 *
 * @return     int, 0--success, others--fail
 */
esp_err_t ac101_set_state(media_hal_codec_mode_t mode, media_hal_sel_state_t media_hal_state);

/**
 * @brief Set voice volume for audio output
 *        @note if volume is 0, mute is enabled
 *
 * @param volume value of volume in percent(%)
 *
 * @return     int, 0--success, others--fail
 */
esp_err_t ac101_control_volume(uint8_t volume);

/**
 * @brief get voice volume
 *        @note if volume is 0, mute is enabled
 *
 * @param volume value of volume in percent returned(%)
 *
 * @return     int, 0--success, others--fail
 */
esp_err_t ac101_get_volume(uint8_t *volume);

/**
 * @brief Set mute
 *
 * @param mute enable or disable
 *
 * @return     int, 0--success, others--fail
 */
esp_err_t ac101_set_mute(bool bmute);

/**
 * @brief Set microphone gain
 *
 * @param gain select gain value
 *
 * @return     int, 0--success, others--fail
 */
esp_err_t ac101_set_mic_gain(ac101_mic_gain_t gain);


//int ac101_config_adc_input(ac101_adc_input_t input);
/**
 * @exemple ac101_config_tac101_dac_output_t(AC101_DAC_OUTPUT_LOUT1 | AC101_DAC_OUTPUT_LOUT2 | AC101_DAC_OUTPUT_ROUT1 | AC101_DAC_OUTPUT_ROUT2);
 */
//int ac101_config_dac_output(ac101_dac_output_t output);
esp_err_t ac101_write_register(uint8_t regAdd, uint8_t data);

/**
 * @brief Power up ac101 audio codec
 *
 * @param none
 *
 * @return     int, 0--success, others--fail
 */
esp_err_t ac101_powerup();

/**
 * @brief Power down ac101 audio codec
 *
 * @param none
 *
 * @return     int, 0--success, others--fail
 */
esp_err_t ac101_powerdown();

void ac101_read_all_reg();


#endif //__AC101_INTERFACE_H__