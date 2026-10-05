# Copyright (c) Meta Platforms, Inc. and affiliates.
# SPDX-License-Identifier: Apache-2.0

"""Check the production audio initializer's codec wiring, without hardware.

Two logical devices must share one physical codec and duplex I2S interface.
This catches the separate BSP initializers that fight over the same amp pin;
it does not simulate the ES8311 or establish audible output.
"""

import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
BOARD = ROOT / "components/muse/boards/board_waveshare_s3_18.c"

FAKES = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM -2
#define ESP_RETURN_ON_ERROR(call, ...) do { int rc = (call); if (rc) return rc; } while (0)
#define ESP_RETURN_ON_FALSE(ok, rc, ...) do { if (!(ok)) return (rc); } while (0)
#define CONFIG_BSP_I2S_NUM 0
#define BSP_I2C_NUM 0
#define BSP_I2S_MCLK 16
#define BSP_I2S_SCLK 9
#define BSP_I2S_LCLK 45
#define BSP_I2S_DOUT 8
#define BSP_I2S_DSIN 10
#define BSP_POWER_AMP_IO 46
#define MUSE_AUDIO_RATE 16000
#define ES8311_CODEC_DEFAULT_ADDR 0x30
#define I2S_ROLE_MASTER 1
#define I2S_DATA_BIT_WIDTH_16BIT 16
#define I2S_SLOT_MODE_STEREO 2
#define I2S_CHANNEL_DEFAULT_CONFIG(port, role) {false}
#define I2S_STD_CLK_DEFAULT_CONFIG(rate) (rate)
#define I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bits, slots) {(bits), (slots)}
#define ESP_CODEC_DEV_TYPE_IN 1
#define ESP_CODEC_DEV_TYPE_OUT 2
#define ESP_CODEC_DEV_WORK_MODE_BOTH 3
typedef int esp_err_t;
typedef void *i2s_chan_handle_t;
typedef struct { bool auto_clear; } i2s_chan_config_t;
typedef struct {
    int clk_cfg;
    struct { int bits, slots; } slot_cfg;
    struct { int mclk, bclk, ws, dout, din; } gpio_cfg;
} i2s_std_config_t;
typedef struct { int port; void *rx_handle, *tx_handle; } audio_codec_i2s_cfg_t;
typedef struct { int port, addr; void *bus_handle; } audio_codec_i2c_cfg_t;
typedef struct { int unused; } audio_codec_data_if_t;
typedef struct { int unused; } audio_codec_ctrl_if_t;
typedef struct { int unused; } audio_codec_gpio_if_t;
typedef struct { int unused; } audio_codec_if_t;
typedef struct {
    const audio_codec_ctrl_if_t *ctrl_if;
    const audio_codec_gpio_if_t *gpio_if;
    int codec_mode, pa_pin;
    bool use_mclk;
    struct { float pa_voltage, codec_dac_voltage; } hw_gain;
} es8311_codec_cfg_t;
typedef struct {
    int dev_type;
    const audio_codec_if_t *codec_if;
    const audio_codec_data_if_t *data_if;
} esp_codec_dev_cfg_t;
typedef esp_codec_dev_cfg_t *esp_codec_dev_handle_t;

static int tx, rx, bus, initialized, enabled, codec_count, device_count, failure;
static audio_codec_data_if_t data;
static audio_codec_ctrl_if_t control;
static audio_codec_gpio_if_t gpio;
static audio_codec_if_t codecs[2];
static esp_codec_dev_cfg_t devices[2];
int i2s_new_channel(const i2s_chan_config_t *cfg, void **out, void **in) {
    assert(cfg->auto_clear);
    if (failure == 1) return ESP_FAIL;
    *out = &tx; *in = &rx; return ESP_OK;
}
int i2s_channel_init_std_mode(void *chan, const i2s_std_config_t *cfg) {
    assert(chan == &tx || chan == &rx);
    assert(cfg->clk_cfg == MUSE_AUDIO_RATE && cfg->slot_cfg.bits == 16 && cfg->slot_cfg.slots == 2);
    assert(cfg->gpio_cfg.mclk == 16 && cfg->gpio_cfg.bclk == 9 && cfg->gpio_cfg.ws == 45);
    assert(cfg->gpio_cfg.dout == 8 && cfg->gpio_cfg.din == 10);
    initialized++; return ESP_OK;
}
int i2s_channel_enable(void *chan) { assert(chan == &tx || chan == &rx); enabled++; return ESP_OK; }
void *bsp_i2c_get_handle(void) { return &bus; }
const audio_codec_data_if_t *audio_codec_new_i2s_data(const audio_codec_i2s_cfg_t *cfg) {
    assert(cfg->rx_handle == &rx && cfg->tx_handle == &tx); return &data;
}
const audio_codec_ctrl_if_t *audio_codec_new_i2c_ctrl(const audio_codec_i2c_cfg_t *cfg) {
    assert(cfg->addr == ES8311_CODEC_DEFAULT_ADDR && cfg->bus_handle == &bus); return &control;
}
const audio_codec_gpio_if_t *audio_codec_new_gpio(void) { return &gpio; }
const audio_codec_if_t *es8311_codec_new(const es8311_codec_cfg_t *cfg) {
    assert(cfg->ctrl_if == &control && cfg->gpio_if == &gpio);
    assert(cfg->codec_mode == ESP_CODEC_DEV_WORK_MODE_BOTH);
    assert(cfg->pa_pin == BSP_POWER_AMP_IO && cfg->use_mclk);
    if (failure == 2) return NULL;
    assert(codec_count < 2); return &codecs[codec_count++];
}
esp_codec_dev_handle_t esp_codec_dev_new(const esp_codec_dev_cfg_t *cfg) {
    if (failure == 3) return NULL;
    assert(device_count < 2); devices[device_count] = *cfg; return &devices[device_count++];
}
/* The old BSP path shares I2S but gives the two logical devices separate
 * ES8311 instances. Keep that behavior here so the unfixed initializer
 * compiles and fails the identity assertion, rather than a missing symbol. */
int bsp_audio_init(const i2s_std_config_t *cfg) {
    (void)cfg; initialized = enabled = 2; return ESP_OK;
}
esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void) {
    codec_count++;
    esp_codec_dev_cfg_t cfg = {ESP_CODEC_DEV_TYPE_OUT, &codecs[0], &data};
    return esp_codec_dev_new(&cfg);
}
esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void) {
    codec_count++;
    esp_codec_dev_cfg_t cfg = {ESP_CODEC_DEV_TYPE_IN, &codecs[1], &data};
    return esp_codec_dev_new(&cfg);
}
'''

HARNESS = r'''
int main(int argc, char **argv) {
    assert(argc == 2); failure = atoi(argv[1]);
    esp_codec_dev_handle_t spk = NULL, mic = NULL;
    int rc = audio_init(&spk, &mic);
    if (failure) {
        assert(rc != ESP_OK);
        assert(device_count == 0);
        return 0;
    }
    assert(rc == ESP_OK && spk && mic && spk != mic);
    assert(spk->codec_if == mic->codec_if);
    assert(spk->data_if == mic->data_if);
    assert(spk->dev_type == ESP_CODEC_DEV_TYPE_OUT && mic->dev_type == ESP_CODEC_DEV_TYPE_IN);
    assert(codec_count == 1 && device_count == 2 && initialized == 2 && enabled == 2);
    return 0;
}
'''


class WaveshareS318Audio(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = BOARD.read_text()
        start = source.index("static esp_err_t audio_init(")
        end = source.index("\n}\n", start) + 3
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        c = Path(cls.tmp.name) / "audio.c"
        c.write_text(FAKES + source[start:end] + HARNESS)
        cls.exe = c.with_suffix("")
        result = subprocess.run([
            *shlex.split(os.environ.get("CC", "cc")), "-std=c11", "-Wall", "-Wextra", "-Werror",
            str(c), "-o", str(cls.exe),
        ], capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)

    def test_shared_physical_codec(self):
        subprocess.run([str(self.exe), "0"], check=True, capture_output=True, timeout=10)

    def test_initialization_failures_are_reported(self):
        for failure in (1, 2, 3):
            with self.subTest(failure=failure):
                subprocess.run([str(self.exe), str(failure)], check=True, capture_output=True, timeout=10)
