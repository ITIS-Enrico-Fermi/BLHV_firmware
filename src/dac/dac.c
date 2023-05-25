#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "sdkconfig.h"

#include "esp_log.h"
#include "driver/i2s_std.h"

#include "dac.h"

static i2s_chan_handle_t tx_chan;
static bool tx_chan_enabled = false;

bool dac_setup() {
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));

    i2s_std_config_t tx_std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = DAC_I2S_MCLK_PIN,
            .bclk = DAC_I2S_BCLK_PIN,
            .ws   = DAC_I2S_WS_PIN,
            .dout = DAC_I2S_DATA_PIN,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));

    return true;
}

int dac_write(uint8_t datum) {
    uint32_t i2sdata[2] = {datum << 24, datum << 24};
    size_t data_sz = sizeof(i2sdata);

    if(!tx_chan_enabled) {
        ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
        tx_chan_enabled = true;
    }
    
    if (i2s_channel_write(tx_chan, i2sdata, sizeof(uint32_t) * 2, &data_sz, 1000) != ESP_OK)
        ESP_LOGE("DAC", "I2S write failed");

    
    return 1;
}
