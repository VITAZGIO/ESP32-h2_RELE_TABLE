// ESP32-H2 "Реле под столом" — Zigbee Router
// Встроенная Arduino Zigbee (Zigbee.h) + OneWireNg (поддержка H2)

#ifndef ZIGBEE_MODE_ZCZR
#error "Zigbee router mode is not selected. Add -DZIGBEE_MODE_ZCZR to build_flags"
#endif

#include "Zigbee.h"
#include <esp_adc/adc_oneshot.h>
#include "OneWireNg_CurrentPlatform.h"
#include "drivers/DSTherm.h"
#include "utils/Placeholder.h"

// =====================================================
// =============== ПИНЫ =================================
// =====================================================

#define RELAY_1_PIN     4    // GPIO4  — реле 1
#define RELAY_2_PIN     5    // GPIO5  — реле 2
#define MOSFET_USB_PIN  10   // GPIO10 — MOSFET USB питание
#define PSU_TEMP_PIN    12   // GPIO12 — DS18B20
#define FAN_PWM_PIN     14   // GPIO14 — PWM вентилятор
#define BUTTONS_ADC_PIN 1    // GPIO1  — кнопки (ADC1_CH0)
#define BLUE_LED_PIN    13   // GPIO13 — синий LED

// =====================================================
// =============== ZIGBEE ENDPOINTS ====================
// =====================================================

#define EP_RELAY_1  10
#define EP_RELAY_2  11
#define EP_USB      12
#define EP_FAN      13   // Analog output (скорость вентилятора)
#define EP_TEMP     14   // Датчик температуры
#define EP_BTN_1    20   // Кнопки как ContactSwitch
#define EP_BTN_2    21
#define EP_BTN_3    22
#define EP_BTN_4    23
#define EP_BTN_5    24

// =====================================================
// =============== ZIGBEE ОБЪЕКТЫ =====================
// =====================================================

ZigbeeLight       zbRelay1(EP_RELAY_1);
ZigbeeLight       zbRelay2(EP_RELAY_2);
ZigbeeLight       zbUSB(EP_USB);
ZigbeeAnalog      zbFan(EP_FAN);
ZigbeeTempSensor  zbTemp(EP_TEMP);

ZigbeeLight zbBtn1(EP_BTN_1);
ZigbeeLight zbBtn2(EP_BTN_2);
ZigbeeLight zbBtn3(EP_BTN_3);
ZigbeeLight zbBtn4(EP_BTN_4);
ZigbeeLight zbBtn5(EP_BTN_5);

ZigbeeLight* zbButtons[5] = {&zbBtn1, &zbBtn2, &zbBtn3, &zbBtn4, &zbBtn5};

// Локальное состояние кнопок-лайтов (для toggle)
bool btn_light_state[5] = {false, false, false, false, false};

// =====================================================
// =============== РЕЗИСТИВНЫЙ МОСТ ====================
// =====================================================

// Номиналы: 680Ω, 2kΩ, 4.7kΩ, 10kΩ, 20kΩ, 68kΩ
#define ADC_THRESHOLD_1 471    // 680Ω  — кнопка 1
#define ADC_THRESHOLD_2 995    // 2kΩ   — кнопка 2
#define ADC_THRESHOLD_3 1678   // 4.7kΩ — кнопка 3
#define ADC_THRESHOLD_4 2389   // 10kΩ  — кнопка 4
#define ADC_THRESHOLD_5 3153   // 20kΩ  — кнопка 5
#define ADC_THRESHOLD_6 3829   // 68kΩ  — кнопка 6 (системная)

// =====================================================
// =============== FAN PWM =============================
// =====================================================

static const int PWM_FREQ = 25000;
static const int PWM_RESOLUTION = 8;

static const uint8_t FAN_SPEED_PWM[11] = {
  255, 230, 205, 180, 155, 130, 105, 80, 55, 30, 0
};
static int current_fan_speed = 4;

// =====================================================
// =============== DS18B20 =============================
// =====================================================

static Placeholder<OneWireNg_CurrentPlatform> ow;
uint32_t last_temp_read_ms = 0;
const uint32_t TEMP_READ_INTERVAL_MS = 10000;
float last_temp = 0.0f;
bool temp_sensor_ok = false;

// =====================================================
// =============== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ==============
// =====================================================

adc_oneshot_unit_handle_t adc_handle = nullptr;

volatile bool relay_state[2] = {false, false};
volatile bool usb_power_state = false;
bool zb_ready = false;   // true только после Zigbee.connected()

// Антидребезг кнопок
uint32_t last_button_press_ms = 0;
int last_button = -1;
const uint32_t DEBOUNCE_MS = 50;

// Системная кнопка (6-я)
uint32_t system_button_press_start_ms = 0;
bool system_button_was_pressed = false;
const uint32_t FACTORY_RESET_HOLD_MS = 3000;

// =====================================================
// =============== ZIGBEE CALLBACKS ====================
// =====================================================

void onRelay1Change(bool state) {
  relay_state[0] = state;
  digitalWrite(RELAY_1_PIN, state ? LOW : HIGH);   // active-LOW реле
  Serial.printf("[ZB] Relay 1 -> %s\n", state ? "ON" : "OFF");
}

void onRelay2Change(bool state) {
  relay_state[1] = state;
  digitalWrite(RELAY_2_PIN, state ? LOW : HIGH);   // active-LOW реле
  Serial.printf("[ZB] Relay 2 -> %s\n", state ? "ON" : "OFF");
}

void onUSBChange(bool state) {
  usb_power_state = state;
  digitalWrite(MOSFET_USB_PIN, state ? HIGH : LOW);
  Serial.printf("[ZB] USB -> %s\n", state ? "ON" : "OFF");
}

// Пустой колбэк для кнопок-лайтов (убирает warning, тумблер локальный)
void onButtonLightChange(bool state) {
  (void)state;
}

// Вентилятор (analog 0..100 -> скорость 0..10)
void onFanChange(float value) {
  int level = (int)round(value / 10.0f);
  if (level < 0) level = 0;
  if (level > 10) level = 10;
  current_fan_speed = level;
  ledcWrite(FAN_PWM_PIN, FAN_SPEED_PWM[level]);
  Serial.printf("[ZB] Fan -> %.0f%% (level %d, PWM %d)\n", value, level, FAN_SPEED_PWM[level]);
}

// =====================================================
// =============== ADC И КНОПКИ ======================
// =====================================================

static void adc_init() {
  adc_oneshot_unit_init_cfg_t init_config = {
    .unit_id = ADC_UNIT_1,
  };
  adc_oneshot_new_unit(&init_config, &adc_handle);

  adc_oneshot_chan_cfg_t config = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_12,
  };
  adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &config);
  Serial.println("[ADC] Initialized on GPIO1");
}

static int read_button_adc() {
  if (!adc_handle) return 4095;
  // 5 чтений, берём среднее — гасит шум на неподключённом/длинном входе
  int sum = 0, v = 0;
  for (int k = 0; k < 5; k++) {
    adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &v);
    sum += v;
    delayMicroseconds(50);
  }
  return sum / 5;
}

static int detect_button_press(int adc_value) {
  if (adc_value < ADC_THRESHOLD_1) return 0;
  if (adc_value < ADC_THRESHOLD_2) return 1;
  if (adc_value < ADC_THRESHOLD_3) return 2;
  if (adc_value < ADC_THRESHOLD_4) return 3;
  if (adc_value < ADC_THRESHOLD_5) return 4;
  if (adc_value < ADC_THRESHOLD_6) return 5;
  return -1;
}

// Кнопки 1-5: нажатие -> Contact "closed" (сработал), потом отпускание -> "open"
static void on_button_pressed(int button_idx) {
  if (button_idx < 0 || button_idx > 4) return;
  Serial.printf("[BTN] Button %d pressed\n", button_idx + 1);
  if (!zb_ready) return;
  // toggle — в HA видно как переключение этого "света"
  btn_light_state[button_idx] = !btn_light_state[button_idx];
  zbButtons[button_idx]->setLight(btn_light_state[button_idx]);
}

static void on_button_released(int button_idx) {
  // ничего: toggle уже отправлен при нажатии
  (void)button_idx;
}

// =====================================================
// =============== СИСТЕМНАЯ КНОПКА ====================
// =====================================================

static void factory_reset() {
  Serial.println("[SYS] Factory Reset...");
  for (int i = 0; i < 6; i++) {
    digitalWrite(BLUE_LED_PIN, LOW);
    delay(150);
    digitalWrite(BLUE_LED_PIN, HIGH);
    delay(150);
  }
  delay(500);
  Zigbee.factoryReset();
}

static void permit_join() {
  Serial.println("[SYS] Permit Join 180s...");
  for (int i = 0; i < 3; i++) {
    digitalWrite(BLUE_LED_PIN, LOW);
    delay(300);
    digitalWrite(BLUE_LED_PIN, HIGH);
    delay(300);
  }
  Zigbee.openNetwork(180);
}

static void handle_system_button(int button_state) {
  uint32_t now = millis();

  if (button_state == 5) {
    if (!system_button_was_pressed) {
      system_button_press_start_ms = now;
      system_button_was_pressed = true;
      Serial.println("[SYS] System button pressed...");
    } else {
      if ((now - system_button_press_start_ms) >= FACTORY_RESET_HOLD_MS) {
        factory_reset();
      }
    }
  } else {
    if (system_button_was_pressed) {
      uint32_t held = now - system_button_press_start_ms;
      if (held < FACTORY_RESET_HOLD_MS) {
        permit_join();
      }
      system_button_was_pressed = false;
    }
  }
}

// =====================================================
// =============== ДАТЧИК ТЕМПЕРАТУРЫ =================
// =====================================================

static void temperature_read() {
  uint32_t now = millis();
  if (now - last_temp_read_ms < TEMP_READ_INTERVAL_MS) return;
  last_temp_read_ms = now;

  DSTherm drv(ow);

  // Запустить конверсию на всех датчиках шины
  drv.convertTempAll(DSTherm::MAX_CONV_TIME, false);

  // Один датчик на шине -> читаем напрямую
  static PlaceholderInit<DSTherm::Scratchpad> scrpd;
  OneWireNg::ErrorCode ec = drv.readScratchpadSingle(scrpd);

  if (ec == OneWireNg::EC_SUCCESS) {
    // getTemp2() возвращает температуру в 1/16 градуса
    float temp = (float)scrpd->getTemp2() / 16.0f;
    last_temp = temp;
    temp_sensor_ok = true;
    Serial.printf("[TEMP] PSU: %.2f C\n", temp);
    if (zb_ready) {
      zbTemp.setTemperature(temp);
      zbTemp.reportTemperature();
    }
  } else if (ec == OneWireNg::EC_CRC_ERROR) {
    Serial.println("[TEMP] CRC error");
  } else {
    if (temp_sensor_ok) Serial.println("[TEMP] Sensor read failed!");
    temp_sensor_ok = false;
  }
}

// =====================================================
// =============== SETUP ===============================
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[SETUP] ESP32-H2 Rele pod stolom (Zigbee Router)");

  // === GPIO ===
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  pinMode(MOSFET_USB_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(RELAY_1_PIN, HIGH);   // active-LOW: HIGH = выкл
  digitalWrite(RELAY_2_PIN, HIGH);   // active-LOW: HIGH = выкл
  digitalWrite(MOSFET_USB_PIN, LOW); // MOSFET active-HIGH: LOW = выкл
  digitalWrite(BLUE_LED_PIN, HIGH);

  // === ADC ===
  // Внутренняя подтяжка вверх: без лестницы вход не болтается (читает ~max = "не нажато")
  pinMode(BUTTONS_ADC_PIN, INPUT_PULLUP);
  adc_init();

  // === FAN PWM ===
  ledcAttach(FAN_PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(FAN_PWM_PIN, FAN_SPEED_PWM[current_fan_speed]);
  Serial.println("[FAN] PWM ready");

  // === DS18B20 (OneWireNg / DSTherm) ===
  new (&ow) OneWireNg_CurrentPlatform(PSU_TEMP_PIN, false);
  {
    DSTherm drv(ow);
    drv.writeScratchpadAll(0, 0, DSTherm::RES_12_BIT);  // 12 бит разрешение
    drv.copyScratchpadAll(false);
  }
  Serial.println("[TEMP] DS18B20 ready (OneWireNg)");

  // === ZIGBEE ENDPOINTS ===
  zbRelay1.setManufacturerAndModel("VITAZGIO", "ReleTable");
  zbRelay1.onLightChange(onRelay1Change);
  Zigbee.addEndpoint(&zbRelay1);

  zbRelay2.setManufacturerAndModel("VITAZGIO", "ReleTable");
  zbRelay2.onLightChange(onRelay2Change);
  Zigbee.addEndpoint(&zbRelay2);

  zbUSB.setManufacturerAndModel("VITAZGIO", "ReleTable");
  zbUSB.onLightChange(onUSBChange);
  Zigbee.addEndpoint(&zbUSB);

  // Вентилятор (analog output)
  zbFan.setManufacturerAndModel("VITAZGIO", "ReleTable");
  zbFan.addAnalogOutput();
  zbFan.onAnalogOutputChange(onFanChange);
  Zigbee.addEndpoint(&zbFan);

  // Датчик температуры
  zbTemp.setManufacturerAndModel("VITAZGIO", "ReleTable");
  zbTemp.setMinMaxValue(-10, 125);
  zbTemp.setTolerance(0.5);
  Zigbee.addEndpoint(&zbTemp);

  // Кнопки 1-5 (ContactSwitch)
  for (int i = 0; i < 5; i++) {
    zbButtons[i]->setManufacturerAndModel("VITAZGIO", "ReleTable");
    zbButtons[i]->onLightChange(onButtonLightChange);
    Zigbee.addEndpoint(zbButtons[i]);
  }

  // === СТАРТ ZIGBEE (Router) ===
  Serial.println("[ZB] Starting Zigbee Router...");
  if (!Zigbee.begin(ZIGBEE_ROUTER)) {
    Serial.println("[ZB] Zigbee begin FAILED! Rebooting...");
    delay(1000);
    ESP.restart();
  }

  Serial.println("[ZB] Waiting for network...");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\n[ZB] Connected!");

  // Кнопки теперь ZigbeeLight — начальных значений слать не нужно.
  zb_ready = true;
  Serial.println("[SETUP] Ready!");
}

// =====================================================
// =============== LOOP ================================
// =====================================================

void loop() {
  uint32_t now = millis();

  int adc_value = read_button_adc();
  int pressed_raw = detect_button_press(adc_value);

  // Подтверждение: кнопка засчитывается только если детект стабилен 3 раза подряд
  static int stable_btn = -1;
  static int stable_cnt = 0;
  if (pressed_raw == stable_btn) {
    if (stable_cnt < 3) stable_cnt++;
  } else {
    stable_btn = pressed_raw;
    stable_cnt = 1;
  }
  int pressed = (stable_cnt >= 3) ? stable_btn : last_button;

  if (pressed >= 0 && pressed <= 4) {
    if (pressed != last_button && (now - last_button_press_ms) > DEBOUNCE_MS) {
      on_button_pressed(pressed);
      last_button_press_ms = now;
    }
    last_button = pressed;
  } else if (pressed == 5) {
    handle_system_button(5);
    last_button = pressed;
  } else {
    if (last_button == 5) {
      handle_system_button(-1);
    } else if (last_button >= 0 && last_button <= 4) {
      on_button_released(last_button);
    }
    last_button = -1;
  }

  temperature_read();

  static uint32_t last_print = 0;
  if (now - last_print > 3000) {
    Serial.printf("[STAT] ADC=%4d btn=%d rel:%d %d usb:%d fan:%d temp:%.1f zb:%d\n",
                  adc_value, pressed, relay_state[0], relay_state[1],
                  usb_power_state, current_fan_speed, last_temp, Zigbee.connected());
    last_print = now;
  }

  delay(10);
}
