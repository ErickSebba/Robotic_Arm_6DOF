// Emulador do encoder magnetico MT6701 (saida PWM) montado no eixo traseiro do motor.
//
// O Wokwi nao deixa um chip ler o angulo de outro componente, entao o chip
// "enxerga" o eixo contando os pulsos STEP/DIR que vao para o driver
// (eixo ideal, sem folga). O controle "slip" descarta N micropassos a cada
// 1000 para simular perda de passo e testar a malha fechada.
//
// Quadro PWM do MT6701: 4119 clocks por periodo (~994,4 Hz);
// nivel alto = 16 + codigo (0..4095), depois baixo ate completar 4119.

#include "wokwi-api.h"
#include <stdint.h>
#include <stdlib.h>

#define FRAME_CLOCKS 4119u
#define START_CLOCKS 16u
#define CLOCK_NS 244u  // 4119 * 244 ns = 1,005 ms (~995 Hz)

typedef struct {
  pin_t step, dir, out;
  uint32_t microsteps_attr, slip_attr;
  int64_t usteps;          // posicao do eixo em micropassos
  uint32_t slip_acc;       // acumulador para descartar passos
  timer_t frame_timer, fall_timer;
} chip_state_t;

static void on_step(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *c = (chip_state_t *)user_data;
  uint32_t slip = (uint32_t)attr_read_float(c->slip_attr);
  c->slip_acc += slip;
  if (c->slip_acc >= 1000) {  // descarta este passo (motor "pulou")
    c->slip_acc -= 1000;
    return;
  }
  c->usteps += pin_read(c->dir) ? 1 : -1;
}

static uint32_t angle_code(chip_state_t *c) {
  int64_t per_rev = 200 * (int64_t)attr_read(c->microsteps_attr);
  int64_t p = c->usteps % per_rev;
  if (p < 0) p += per_rev;
  return (uint32_t)((p * 4096) / per_rev);  // 0..4095
}

static void on_fall(void *user_data) {
  chip_state_t *c = (chip_state_t *)user_data;
  pin_write(c->out, LOW);
}

static void on_frame(void *user_data) {
  chip_state_t *c = (chip_state_t *)user_data;
  uint32_t high = START_CLOCKS + angle_code(c);
  pin_write(c->out, HIGH);
  timer_start_ns(c->fall_timer, (uint64_t)high * CLOCK_NS, false);
}

void chip_init(void) {
  chip_state_t *c = calloc(1, sizeof(chip_state_t));
  c->step = pin_init("STEP", INPUT);
  c->dir = pin_init("DIR", INPUT);
  c->out = pin_init("OUT", OUTPUT_LOW);
  c->microsteps_attr = attr_init("microsteps", 16);
  c->slip_attr = attr_init_float("slip", 0.0f);

  const pin_watch_config_t w = {.edge = RISING, .pin_change = on_step, .user_data = c};
  pin_watch(c->step, &w);

  const timer_config_t tf = {.callback = on_frame, .user_data = c};
  const timer_config_t tl = {.callback = on_fall, .user_data = c};
  c->frame_timer = timer_init(&tf);
  c->fall_timer = timer_init(&tl);
  timer_start_ns(c->frame_timer, (uint64_t)FRAME_CLOCKS * CLOCK_NS, true);
}
