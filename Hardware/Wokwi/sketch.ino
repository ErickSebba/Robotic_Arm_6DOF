// BIMO - firmware de acionamento (ESP32 DevKit 30 pinos)
// Roda igual no Wokwi e no hardware real. Cinematica fica no computador;
// aqui: perfil sincrono, malha fechada por junta (encoder no eixo do motor),
// limites, falha por erro de seguimento, watchdog de comunicacao.
//
// Comandos (Serial 115200 ou UDP porta 4210, uma linha por comando/pacote;
// no UDP pode prefixar "#<seq> "):
//   M q1 q2 q3 q4 q5 q6 [T]   move para angulos de junta (graus), T minimo em s
//   MODE SYNC | MODE SEQ      padrao: sincrono; SEQ = uma junta por vez (J1..J6)
//   STOP                      para e segura posicao
//   EN 1 | EN 0               habilita/desabilita drivers
//   CLR                       limpa falha
//   S                         status
//   H                         heartbeat (UDP: mandar a cada ~100 ms)
//   K j kp ki kd              ganhos da junta j (1..6)
//   ZERO                      imprime codigos atuais dos encoders (calibrar ZERO_CODE)

// 1 = simulacao no Wokwi (captura do encoder por interrupcao; o Wokwi nao simula MCPWM)
// 0 = hardware real (captura por MCPWM: flanco e carimbo de tempo feitos pelo periferico)
#define BIMO_WOKWI 1

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_arduino_version.h>
#include "soc/gpio_reg.h"
#if !BIMO_WOKWI
#include "driver/mcpwm_cap.h"
#endif

// ============================================================= pinagem (de netlist.py)
static const uint8_t PIN_EN = 23;  // ativo em baixo, comum
static const uint8_t PIN_STEP[6] = {25, 26, 27, 16, 17, 18};
static const uint8_t PIN_DIR[6]  = {13, 14, 4, 19, 21, 22};
static const uint8_t PIN_ENC[6]  = {36, 39, 34, 35, 32, 33};

// ============================================================= parametros e tipos
// ----------------------------------------------------------------- parametros
static const int N = 6;
static const float FULL_STEPS = 200.0f;
static const float MICROSTEP = 16.0f;  // DRV8825: jumper so em M2 (1/16)
// Relacao de reducao por junta. J4-J6 (EBA-17-S): nominal 38,4, mas a contagem
// de dentes deu 35,444 -> MEDIR e corrigir antes do primeiro teste.
static const float GEAR[N] = {38.4f, 38.4f, 38.4f, 38.4f, 38.4f, 38.4f};
static const float JOINT_MIN_DEG[N] = {-170, -90, -135, -170, -110, -180};
static const float JOINT_MAX_DEG[N] = {170, 90, 135, 170, 110, 180};
static const float VMAX_DEG_S[N] = {15, 10, 15, 20, 20, 25};   // conservador (qualidade primeiro)
static const float AMAX_DEG_S2[N] = {20, 15, 20, 30, 30, 40};


// Decodificador do PWM do MT6701 a partir de flancos com carimbo de tempo.
// Unidades de tempo arbitrarias (us na captura por interrupcao, ticks no MCPWM):
// o codigo sai da RAZAO nivel_alto / periodo.
//
// Pulso curto: com o motor perto do codigo 0 (ou 4095) o nivel alto (ou baixo) dura
// so 16 (ou 8) clocks = 2-4 us. Na captura por interrupcao os dois flancos podem
// chegar antes da ISR ler o pino; ela ve o MESMO nivel da ultima vez. Com
// infer_short = true isso e interpretado como um pulso curto (codigo ~0 ou ~4095).
// No MCPWM (flanco informado pelo hardware) infer_short = false: so ressincroniza.
struct PwmDecoder {
  bool infer_short = true;
  uint32_t period_min = 800, period_max = 1250;  // em unidades de tempo (us por padrao)
  uint32_t dup_window = 50;                      // eventos repetidos dentro disto sao ignorados
  int last_level = -1;
  int64_t t_rise = 0, t_last = 0;
  uint32_t high = 0, period = 0;  // ultima medida (period = estimativa filtrada)
  float period_f = 0;             // periodo filtrado: o do MT6701 e fixo (cristal)
  int64_t t_valid = 0;            // instante da ultima medida completa (0 = nunca)
  uint32_t frames = 0;

  void feed_period(uint32_t p) {  // media exponencial; ignora amostras > 3 % fora
    if (!in_range(p)) return;
    if (period_f == 0) { period_f = (float)p; return; }
    if (fabsf((float)p - period_f) > 0.03f * period_f) return;
    period_f += ((float)p - period_f) * (1.0f / 16.0f);
  }
  void accept(int64_t t, uint32_t h) {
    if (period_f == 0) return;
    uint32_t p = (uint32_t)lroundf(period_f);
    if (h > p) h = p;
    high = h; period = p; t_valid = t; frames++;
  }
  bool in_range(uint32_t p) const { return p >= period_min && p <= period_max; }
  void edge(int64_t t, int level) {
    if (level == last_level) {
      if (!infer_short || t - t_last < (int64_t)dup_window) { t_last = t; return; }
      // pulso curto: os dois flancos chegaram juntos; o quadro comecou em ~t
      uint32_t p = t_rise ? (uint32_t)(t - t_rise) : 0;
      if (in_range(p)) {
        feed_period(p);
        accept(t, level == 0 ? 0 : p);     // alto curto -> codigo ~0 ; baixo curto -> ~4095
      }
      t_rise = t;
    } else if (level) {                    // subida = inicio do quadro
      if (t_rise) feed_period((uint32_t)(t - t_rise));
      t_rise = t;
    } else if (t_rise) {                   // descida: fecha o nivel alto
      accept(t, (uint32_t)(t - t_rise));
    }
    last_level = level;
    t_last = t;
  }
};

// Desenrola voltas do motor: acumula posicao em contagens (4096/volta).
// Chamar UMA vez por quadro novo do sensor. Rejeita saltos implausiveis (leitura
// corrompida) em vez de somar uma volta falsa, e entrega a mediana dos 3 ultimos
// quadros (remove um quadro isolado com erro de latencia; atraso de ~1 quadro).
struct Unwrapper {
  int last = -1;
  int64_t pos = 0;         // posicao crua desenrolada
  int64_t hist[3] = {0, 0, 0};
  int max_jump = 400;      // contagens por quadro (~1 ms): ~10 voltas/s no motor
  uint32_t rejected = 0;
  void reset(int code, int64_t start_pos) {
    last = code; pos = start_pos;
    hist[0] = hist[1] = hist[2] = start_pos;
  }
  // retorna false se a leitura foi rejeitada (posicao mantida)
  bool update(int code) {
    int d = code - last;
    if (d > 2048) d -= 4096;
    if (d < -2048) d += 4096;
    if (d > max_jump || d < -max_jump) { rejected++; return false; }
    pos += d;
    last = code;
    hist[0] = hist[1]; hist[1] = hist[2]; hist[2] = pos;
    return true;
  }
  int64_t filtered() const {
    int64_t a = hist[0], b = hist[1], c = hist[2];
    if (a > b) { int64_t t = a; a = b; b = t; }
    if (b > c) { int64_t t = b; b = c; c = t; }
    if (a > b) { int64_t t = a; a = b; b = t; }
    return b;
  }
};


// ----------------------------------------------- controlador por junta (SLOT)
// Saida: correcao de velocidade (graus/s) somada ao feedforward.
// Aqui entra PID / fuzzy / rede neural na fase de comparacao.
struct PID {
  float kp = 8.0f, ki = 0.0f, kd = 0.0f;  // 1/s
  float integ = 0, prev_e = 0, i_lim = 5.0f;
  float deadband_deg = 0.002f;
  void reset() { integ = 0; prev_e = 0; }
  float update(float e, float dt) {
    if (fabsf(e) < deadband_deg) e = 0;
    integ += e * dt;
    if (integ > i_lim) integ = i_lim;
    if (integ < -i_lim) integ = -i_lim;
    float d = (e - prev_e) / dt;
    prev_e = e;
    return kp * e + ki * integ + kd * d;
  }
};

struct EncCapture {
  uint8_t pin;
  PwmDecoder dec;              // decodificacao (em us no Wokwi, em ticks no MCPWM)
  volatile int64_t seen_us = 0;  // esp_timer do ultimo quadro valido
  uint32_t last_raw = 0;       // MCPWM: extensao do contador de 32 bits
  int64_t t64 = 0;
};

enum Fault : uint8_t { F_NONE = 0, F_FOLLOW = 1, F_SENSOR = 2, F_LIMIT = 4, F_WATCHDOG = 8 };

// ============================================================= funcoes auxiliares
inline float usteps_per_deg(int i) { return FULL_STEPS * MICROSTEP * GEAR[i] / 360.0f; }

// -------------------------------------------------- perfil trapezoidal sincrono
// Todas as juntas compartilham s(t) em [0,1] (ta = T/3): comecam e terminam juntas.
// s'max = 1.5/T, s''max = 4.5/T^2.
struct SyncMove {
  float q0[N], dq[N];
  float T = 0, t = 0;
  bool active = false;

  static float min_time(float d, float vmax, float amax) {
    d = fabsf(d);
    if (d < 1e-6f) return 0;
    float tv = 1.5f * d / vmax, ta = sqrtf(4.5f * d / amax);
    return tv > ta ? tv : ta;
  }
  // retorna T escolhido
  float plan(const float *from, const float *to, float t_min) {
    T = t_min;
    for (int i = 0; i < N; i++) {
      q0[i] = from[i];
      dq[i] = to[i] - from[i];
      float ti = min_time(dq[i], VMAX_DEG_S[i], AMAX_DEG_S2[i]);
      if (ti > T) T = ti;
    }
    t = 0;
    active = T > 0;
    return T;
  }
  // s e ds/dt no instante t
  void s_at(float tt, float &s, float &sd) const {
    if (tt <= 0) { s = 0; sd = 0; return; }
    if (tt >= T) { s = 1; sd = 0; return; }
    float ta = T / 3, a = 4.5f / (T * T), vs = 1.5f / T;
    if (tt < ta) { s = 0.5f * a * tt * tt; sd = a * tt; }
    else if (tt < T - ta) { s = 0.5f * a * ta * ta + vs * (tt - ta); sd = vs; }
    else { float r = T - tt; s = 1 - 0.5f * a * r * r; sd = a * r; }
  }
  // posicao (graus) e velocidade (graus/s) de referencia da junta i
  void ref(int i, float &q, float &qd) const {
    float s, sd;
    s_at(t, s, sd);
    q = q0[i] + s * dq[i];
    qd = sd * dq[i];
  }
};

// ------------------------------------------------ decodificador PWM do MT6701
// high_us / period_us -> codigo 0..4095 (razao cancela tolerancia do clock)
inline int mt6701_code(uint32_t high_us, uint32_t period_us) {
  if (period_us == 0) return -1;
  float clocks = (float)high_us * 4119.0f / (float)period_us;
  int code = (int)lroundf(clocks - 16.0f);
  if (code < 0) code = 0;
  if (code > 4095) code = 4095;
  return code;
}

inline float counts_to_joint_deg(int i, int64_t counts) {
  return (float)counts * 360.0f / (4096.0f * GEAR[i]);
}

// ------------------------------------------------------------------ config
const char *WIFI_SSID = "Wokwi-GUEST";  // hardware real: trocar
const char *WIFI_PASS = "";
const uint16_t UDP_PORT = 4210;
const uint32_t WATCHDOG_MS = 250;       // sem pacote UDP -> para e segura
const uint32_t IDLE_DISABLE_MS = 3000;  // parado por 3 s -> desabilita drivers (0 = nunca)
const uint32_t TICK_US = 20;            // ISR de passo: 50 kHz, pulso STEP = 20 us
const float FOLLOW_ERR_LIMIT_USTEPS = 2 * MICROSTEP;  // 2 passos inteiros...
const uint16_t FOLLOW_ERR_MS = 20;                    // ...por 20 ms seguidos -> falha
const uint16_t SENSOR_LOSS_MS = 20;                   // sem quadro novo por 20 ms -> falha
const uint32_t BOOT_WAIT_MS = 500;                    // espera encoders no boot
const int16_t ZERO_CODE[N] = {0, 0, 0, 0, 0, 0};       // codigo do encoder na marca de referencia
const int8_t DIR_SIGN[N] = {1, 1, 1, 1, 1, 1};         // inverter sentido de uma junta: -1
const int8_t ENC_POL[N] = {1, 1, 1, 1, 1, 1};          // encoder conta + quando pino DIR em nivel alto? senao -1

// ------------------------------------------------------------ estado da ISR
static hw_timer_t *step_timer = nullptr;
static volatile uint32_t phase_inc[N];  // incremento do acumulador por tick (Q32)
static volatile int8_t want_dir[N];     // +1 / -1
static uint32_t phase_acc[N];
static int8_t pin_dir[N];
static volatile int32_t step_count[N];  // passos emitidos (malha aberta, diagnostico)
static uint32_t step_mask_all = 0;

void IRAM_ATTR on_step_tick() {
  REG_WRITE(GPIO_OUT_W1TC_REG, step_mask_all);  // termina o pulso anterior
  uint32_t set_mask = 0;
  for (int i = 0; i < N; i++) {
    uint32_t inc = phase_inc[i];
    if (!inc) continue;
    int8_t d = want_dir[i];
    if (d != pin_dir[i]) {  // troca DIR e espera um tick (setup >= 650 ns)
      pin_dir[i] = d;
      REG_WRITE(((d * DIR_SIGN[i]) > 0) ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, 1u << PIN_DIR[i]);
      continue;
    }
    uint32_t prev = phase_acc[i];
    phase_acc[i] = prev + inc;
    if (phase_acc[i] < prev) {  // estouro -> um micropasso
      set_mask |= 1u << PIN_STEP[i];
      step_count[i] += d;
    }
  }
  if (set_mask) REG_WRITE(GPIO_OUT_W1TS_REG, set_mask);
}

// ------------------------------------------------------- captura do encoder
static EncCapture enc[N];

#if BIMO_WOKWI
void IRAM_ATTR on_enc_edge(void *arg) {
  EncCapture *e = (EncCapture *)arg;
  int64_t t = esp_timer_get_time();
  // nivel lido direto do registrador (mais rapido que digitalRead); encoders estao em GPIO 32-39
  uint32_t in = e->pin >= 32 ? REG_READ(GPIO_IN1_REG) >> (e->pin - 32) : REG_READ(GPIO_IN_REG) >> e->pin;
  uint32_t f = e->dec.frames;
  e->dec.edge(t, in & 1);
  if (e->dec.frames != f) e->seen_us = t;
}

static void enc_begin() {
  for (int i = 0; i < N; i++) {
    enc[i].pin = PIN_ENC[i];
    pinMode(PIN_ENC[i], INPUT);
    attachInterruptArg(PIN_ENC[i], on_enc_edge, &enc[i], CHANGE);
  }
}
#else
static bool IRAM_ATTR on_cap(mcpwm_cap_channel_handle_t, const mcpwm_capture_event_data_t *ev, void *arg) {
  EncCapture *e = (EncCapture *)arg;
  uint32_t raw = ev->cap_value;
  e->t64 += (uint32_t)(raw - e->last_raw);
  e->last_raw = raw;
  uint32_t f = e->dec.frames;
  e->dec.edge(e->t64, ev->cap_edge == MCPWM_CAP_EDGE_POS ? 1 : 0);
  if (e->dec.frames != f) e->seen_us = esp_timer_get_time();
  return false;
}

static void enc_begin() {
  // 2 grupos MCPWM x 3 canais de captura = 6 encoders (J1-J3 grupo 0, J4-J6 grupo 1)
  mcpwm_cap_timer_handle_t tmr[2];
  for (int g = 0; g < 2; g++) {
    mcpwm_capture_timer_config_t tc = {};
    tc.group_id = g;
    tc.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&tc, &tmr[g]));
  }
  uint32_t res_hz = 80000000;
  mcpwm_capture_timer_get_resolution(tmr[0], &res_hz);
  float tpu = res_hz / 1e6f;  // ticks por us
  for (int i = 0; i < N; i++) {
    enc[i].pin = PIN_ENC[i];
    PwmDecoder &d = enc[i].dec;
    d.infer_short = false;  // o hardware informa o flanco: nada a inferir
    d.period_min = (uint32_t)(800 * tpu);
    d.period_max = (uint32_t)(1250 * tpu);
    d.dup_window = 0;
    mcpwm_capture_channel_config_t cc = {};
    cc.gpio_num = PIN_ENC[i];
    cc.prescale = 1;
    cc.flags.pos_edge = true;
    cc.flags.neg_edge = true;
    mcpwm_cap_channel_handle_t ch;
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(tmr[i / 3], &cc, &ch));
    mcpwm_capture_event_callbacks_t cbs = {};
    cbs.on_cap = on_cap;
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(ch, &cbs, &enc[i]));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(ch));
  }
  for (int g = 0; g < 2; g++) {
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(tmr[g]));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(tmr[g]));
  }
}
#endif

// ------------------------------------------------------------ estado geral
static uint8_t fault = F_NONE;
static int fault_joint = -1;
static bool enabled = false;
static bool seq_mode = false;
static SyncMove mv;
static float target[N], q_ref[N], qd_ref[N], q_meas[N], err_deg[N];
static float seq_goal[N];
static int seq_next = -1;
static float seq_tmin = 0;
static Unwrapper unwrap[N];
static PID pid[N];
static uint32_t last_motion_ms = 0, last_udp_ms = 0;
static bool udp_armed = false;
static WiFiUDP udp;
static bool udp_up = false;
static IPAddress peer_ip;
static uint16_t peer_port = 0;
static int32_t last_seq = -1;
static uint32_t lost_pkts = 0;

static uint32_t last_frames[N];
static uint16_t follow_ms[N], loss_ms[N];

// codigo 0..4095 do ultimo quadro; -1 se nao ha quadro recente. fresh = quadro novo desde a ultima chamada
static int enc_code(int i, bool *fresh) {
  uint32_t h, p, fr;
  int64_t seen;
  noInterrupts();
  h = enc[i].dec.high; p = enc[i].dec.period; fr = enc[i].dec.frames; seen = enc[i].seen_us;
  interrupts();
  if (fresh) { *fresh = fr != last_frames[i]; last_frames[i] = fr; }
  if (!seen || esp_timer_get_time() - seen > 5000) return -1;  // sem quadro ha 5 ms
  return mt6701_code(h, p);
}

// posicao inicial: junta assumida a +-1/2 volta do motor da marca de referencia (ZERO_CODE)
static bool init_joint(int i) {
  int code = enc_code(i, nullptr);
  if (code < 0) return false;
  int rel = code - ZERO_CODE[i];
  if (rel > 2048) rel -= 4096;
  if (rel < -2048) rel += 4096;
  unwrap[i].reset(code, rel);
  q_meas[i] = counts_to_joint_deg(i, rel) * DIR_SIGN[i] * ENC_POL[i];
  target[i] = q_ref[i] = q_meas[i];
  err_deg[i] = 0;
  follow_ms[i] = loss_ms[i] = 0;
  pid[i].reset();
  return true;
}

static void set_enable(bool on) {
  if (on && fault) return;
  enabled = on;
  digitalWrite(PIN_EN, on ? LOW : HIGH);
  for (int i = 0; i < N; i++) {
    pid[i].reset();
    if (!on) phase_inc[i] = 0;
  }
  if (on) { delay(20); last_motion_ms = millis(); }
}

static void trip(Fault f, int j) {
  fault |= f;
  fault_joint = j;
  mv.active = false;
  seq_next = -1;
  set_enable(false);
  Serial.printf("FAULT %u J%d\n", fault, j + 1);
}

static void start_move(const float *goal, float tmin) {
  for (int i = 0; i < N; i++) {
    if (goal[i] < JOINT_MIN_DEG[i] || goal[i] > JOINT_MAX_DEG[i]) {
      Serial.printf("ERR limite J%d (%.2f fora de [%.1f, %.1f])\n", i + 1, goal[i], JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
      return;
    }
  }
  if (fault) { Serial.println("ERR em falha (CLR)"); return; }
  if (!enabled) set_enable(true);
  if (seq_mode) {
    memcpy(seq_goal, goal, sizeof(seq_goal));
    seq_tmin = tmin;
    seq_next = 0;
    return;
  }
  float from[N];
  memcpy(from, q_ref, sizeof(from));
  float T = mv.plan(from, goal, tmin);
  memcpy(target, goal, sizeof(target));
  Serial.printf("OK T=%.2fs\n", T);
}

static void seq_step() {  // modo SEQ: dispara a proxima junta quando a anterior terminou
  if (seq_next < 0 || mv.active) return;
  while (seq_next < N && fabsf(seq_goal[seq_next] - q_ref[seq_next]) < 1e-4f) seq_next++;
  if (seq_next >= N) { seq_next = -1; return; }
  float g[N];
  memcpy(g, q_ref, sizeof(g));
  g[seq_next] = seq_goal[seq_next];
  mv.plan(q_ref, g, seq_tmin);
  memcpy(target, g, sizeof(target));
  seq_next++;
}

// ---------------------------------------------------------- malha de 1 kHz
static void control(float dt) {
  if (mv.active) {
    mv.t += dt;
    for (int i = 0; i < N; i++) mv.ref(i, q_ref[i], qd_ref[i]);
    if (mv.t >= mv.T) mv.active = false;
    last_motion_ms = millis();
  } else {
    for (int i = 0; i < N; i++) { q_ref[i] = target[i]; qd_ref[i] = 0; }
    seq_step();
  }
  for (int i = 0; i < N; i++) {
    bool fresh;
    int code = enc_code(i, &fresh);
    if (code >= 0 && fresh) {
      unwrap[i].update(code);  // rejeita salto implausivel sozinho
      loss_ms[i] = 0;
    } else if (++loss_ms[i] >= SENSOR_LOSS_MS && !(fault & F_SENSOR)) {
      trip(F_SENSOR, i);
    }
    q_meas[i] = counts_to_joint_deg(i, unwrap[i].filtered()) * DIR_SIGN[i] * ENC_POL[i];
    err_deg[i] = q_ref[i] - q_meas[i];
  }
  if (!enabled) return;
  for (int i = 0; i < N; i++) {
    if (fabsf(err_deg[i]) * usteps_per_deg(i) > FOLLOW_ERR_LIMIT_USTEPS) {
      if (++follow_ms[i] >= FOLLOW_ERR_MS) { trip(F_FOLLOW, i); return; }
    } else {
      follow_ms[i] = 0;
    }
    float v = qd_ref[i] + pid[i].update(err_deg[i], dt);  // graus/s
    float us = v * usteps_per_deg(i);                       // micropassos/s
    float mag = fabsf(us);
    float max_rate = 0.5f * 1e6f / TICK_US;                 // 1 passo a cada 2 ticks
    if (mag > max_rate) mag = max_rate;
    want_dir[i] = us >= 0 ? 1 : -1;
    phase_inc[i] = (uint32_t)(mag * TICK_US * 1e-6f * 4294967296.0f);
  }
  if (IDLE_DISABLE_MS && !mv.active && seq_next < 0 && millis() - last_motion_ms > IDLE_DISABLE_MS) {
    set_enable(false);
    Serial.println("IDLE -> drivers desabilitados");
  }
}

// ------------------------------------------------------------- comandos
static void print_status(Print &out) {
  out.printf("S en=%d fault=%u j=%d mode=%s moving=%d wifi=%d lost=%lu\n", enabled, fault, fault_joint + 1,
             seq_mode ? "SEQ" : "SYNC", mv.active || seq_next >= 0, udp_up, (unsigned long)lost_pkts);
  for (int i = 0; i < N; i++)
    out.printf("  J%d ref=%9.4f meas=%9.4f err=%8.4f steps=%ld code=%d quadros=%lu rejeit=%lu\n", i + 1, q_ref[i],
               q_meas[i], err_deg[i], (long)step_count[i], enc_code(i, nullptr), (unsigned long)enc[i].dec.frames,
               (unsigned long)unwrap[i].rejected);
}

static void handle_line(char *line, bool from_udp) {
  char *tok = strtok(line, " \t\r\n");
  if (!tok) return;
  if (tok[0] == '#') {
    int32_t s = atol(tok + 1);
    if (s <= last_seq) return;  // atrasado/duplicado
    if (last_seq >= 0 && s > last_seq + 1) lost_pkts += s - last_seq - 1;
    last_seq = s;
    tok = strtok(nullptr, " \t\r\n");
    if (!tok) return;
  }
  if (from_udp) { udp_armed = true; last_udp_ms = millis(); }
  if (!strcmp(tok, "M")) {
    float g[N], tmin = 0;
    for (int i = 0; i < N; i++) {
      char *a = strtok(nullptr, " \t\r\n");
      if (!a) { Serial.println("ERR M precisa de 6 angulos"); return; }
      g[i] = atof(a);
    }
    char *a = strtok(nullptr, " \t\r\n");
    if (a) tmin = atof(a);
    start_move(g, tmin);
  } else if (!strcmp(tok, "MODE")) {
    char *a = strtok(nullptr, " \t\r\n");
    if (a) seq_mode = !strcmp(a, "SEQ");
    Serial.printf("OK MODE %s\n", seq_mode ? "SEQ" : "SYNC");
  } else if (!strcmp(tok, "STOP")) {
    mv.active = false; seq_next = -1;
    memcpy(target, q_ref, sizeof(target));
    Serial.println("OK STOP");
  } else if (!strcmp(tok, "EN")) {
    char *a = strtok(nullptr, " \t\r\n");
    set_enable(a && atoi(a));
  } else if (!strcmp(tok, "CLR")) {
    fault = F_NONE; fault_joint = -1;
    mv.active = false; seq_next = -1;
    for (int i = 0; i < N; i++) {
      if (!init_joint(i)) { trip(F_SENSOR, i); return; }  // continua sem encoder -> falha de novo
    }
    Serial.println("OK CLR");
  } else if (!strcmp(tok, "S")) {
    print_status(Serial);
  } else if (!strcmp(tok, "K")) {
    char *a[4];
    for (int k = 0; k < 4; k++) a[k] = strtok(nullptr, " \t\r\n");
    int j = a[0] ? atoi(a[0]) - 1 : -1;
    if (j < 0 || j >= N || !a[1] || !a[2] || !a[3]) { Serial.println("ERR K j kp ki kd"); return; }
    pid[j].kp = atof(a[1]); pid[j].ki = atof(a[2]); pid[j].kd = atof(a[3]);
    Serial.printf("OK K J%d %.3f %.3f %.3f\n", j + 1, pid[j].kp, pid[j].ki, pid[j].kd);
  } else if (!strcmp(tok, "ZERO")) {
    for (int i = 0; i < N; i++) Serial.printf("J%d code=%d\n", i + 1, enc_code(i, nullptr));
  } else if (!strcmp(tok, "H")) {
    // so atualiza o watchdog
  } else {
    Serial.printf("ERR comando '%s'\n", tok);
  }
}

static void poll_serial() {
  static char buf[160];
  static size_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (n) { buf[n] = 0; handle_line(buf, false); n = 0; }
    } else if (n < sizeof(buf) - 1) buf[n++] = ch;
  }
}

static void poll_udp() {
  if (!udp_up) {
    if (WiFi.status() == WL_CONNECTED) {
      udp.begin(UDP_PORT);
      udp_up = true;
      Serial.printf("WiFi ok, IP %s, UDP %u\n", WiFi.localIP().toString().c_str(), UDP_PORT);
    }
    return;
  }
  int len = udp.parsePacket();
  if (len > 0) {
    static char buf[256];
    int n = udp.read(buf, sizeof(buf) - 1);
    buf[n > 0 ? n : 0] = 0;
    peer_ip = udp.remoteIP();
    peer_port = udp.remotePort();
    handle_line(buf, true);
  }
  static uint32_t last_tel = 0;
  if (peer_port && millis() - last_tel >= 20) {  // telemetria 50 Hz
    last_tel = millis();
    char t[256];
    int k = snprintf(t, sizeof(t), "T %lu %ld %u", (unsigned long)millis(), (long)last_seq, fault);
    for (int i = 0; i < N; i++) k += snprintf(t + k, sizeof(t) - k, " %.4f %.4f", q_meas[i], err_deg[i]);
    udp.beginPacket(peer_ip, peer_port);
    udp.write((const uint8_t *)t, k);
    udp.endPacket();
  }
}

// ------------------------------------------------------------------ setup
void setup() {
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);  // desabilitado ate o firmware assumir
  Serial.begin(115200);
  for (int i = 0; i < N; i++) {
    pinMode(PIN_STEP[i], OUTPUT); digitalWrite(PIN_STEP[i], LOW);
    pinMode(PIN_DIR[i], OUTPUT); digitalWrite(PIN_DIR[i], LOW);
    pin_dir[i] = -DIR_SIGN[i];
    want_dir[i] = pin_dir[i];
    step_mask_all |= 1u << PIN_STEP[i];
  }
  enc_begin();
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  step_timer = timerBegin(1000000);
  timerAttachInterrupt(step_timer, &on_step_tick);
  timerAlarm(step_timer, TICK_US, true, 0);
#else
  step_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(step_timer, &on_step_tick, true);
  timerAlarmWrite(step_timer, TICK_US, true);
  timerAlarmEnable(step_timer);
#endif
  // espera alguns quadros de cada encoder (em vez de um atraso fixo)
  uint32_t t0 = millis();
  while (millis() - t0 < BOOT_WAIT_MS) {
    bool all = true;
    for (int i = 0; i < N; i++) all &= enc[i].dec.frames >= 3 && enc_code(i, nullptr) >= 0;
    if (all) break;
    delay(5);
  }
  for (int i = 0; i < N; i++) {
    if (!init_joint(i)) {
      Serial.printf("ERRO: sem sinal do encoder J%d (GPIO %d) - conferir ligacao do MT6701\n", i + 1, PIN_ENC[i]);
      trip(F_SENSOR, i);
    }
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("BIMO pronto. Comandos: M, MODE, STOP, EN, CLR, S, K, ZERO. Ex.: M 10 0 0 0 0 0");
}

void loop() {
  static uint32_t last_us = micros();
  uint32_t now = micros();
  if (now - last_us >= 1000) {
    float dt = (now - last_us) * 1e-6f;
    last_us = now;
    control(dt);
  }
  poll_serial();
  poll_udp();
  if (udp_armed && millis() - last_udp_ms > WATCHDOG_MS && (mv.active || seq_next >= 0)) {
    mv.active = false; seq_next = -1;
    memcpy(target, q_ref, sizeof(target));
    Serial.println("WATCHDOG: sem pacote UDP, parado");
  }
}
