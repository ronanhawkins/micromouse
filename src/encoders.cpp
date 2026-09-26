#include "encoders.h"

#include <driver/gpio.h>
#include <driver/pulse_cnt.h>

#include "config.h"
#include "polarity.h"

namespace {

pcnt_unit_handle_t units[2];

bool setupUnit(int index, int pinA, int pinB) {
  pcnt_unit_config_t unitCfg = {};
  unitCfg.low_limit = -32768;
  unitCfg.high_limit = 32767;
  unitCfg.flags.accum_count = 1;  // extend the 16-bit counter in software
  if (pcnt_new_unit(&unitCfg, &units[index]) != ESP_OK) return false;

  pcnt_glitch_filter_config_t filter = {};
  filter.max_glitch_ns = 1000;
  pcnt_unit_set_glitch_filter(units[index], &filter);

  pcnt_chan_config_t aCfg = {};
  aCfg.edge_gpio_num = pinA;
  aCfg.level_gpio_num = pinB;
  pcnt_channel_handle_t chanA;
  if (pcnt_new_channel(units[index], &aCfg, &chanA) != ESP_OK) return false;

  pcnt_chan_config_t bCfg = {};
  bCfg.edge_gpio_num = pinB;
  bCfg.level_gpio_num = pinA;
  pcnt_channel_handle_t chanB;
  if (pcnt_new_channel(units[index], &bCfg, &chanB) != ESP_OK) return false;

  // x4 quadrature decode. Counts up when B leads A, the same sign convention
  // as tools/motor_test, so the direction flags mean the same in both.
  pcnt_channel_set_edge_action(chanA, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(chanA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_channel_set_edge_action(chanB, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(chanB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  // Needed for accum_count: the driver folds overflows in at these points.
  pcnt_unit_add_watch_point(units[index], unitCfg.low_limit);
  pcnt_unit_add_watch_point(units[index], unitCfg.high_limit);

  gpio_pullup_en(gpio_num_t(pinA));
  gpio_pullup_en(gpio_num_t(pinB));

  return pcnt_unit_enable(units[index]) == ESP_OK && pcnt_unit_clear_count(units[index]) == ESP_OK &&
         pcnt_unit_start(units[index]) == ESP_OK;
}

int32_t read(int index, bool invert) {
  int count = 0;
  pcnt_unit_get_count(units[index], &count);
  return invert ? -count : count;
}

}  // namespace

bool encodersBegin() {
  return setupUnit(0, PIN_ENC_L_A, PIN_ENC_L_B) && setupUnit(1, PIN_ENC_R_A, PIN_ENC_R_B);
}

int32_t encoderLeft() { return read(0, polarity.encL); }
int32_t encoderRight() { return read(1, polarity.encR); }
