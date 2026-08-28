// ESP32-H2 "Реле под столом" — Zigbee Router
// Встроенная Arduino Zigbee (Zigbee.h) + OneWireNg (поддержка H2)

#ifndef ZIGBEE_MODE_ZCZR
#error "Zigbee router mode is not selected. Add -DZIGBEE_MODE_ZCZR to build_flags"
#endif

#include "Zigbee.h"
#include <esp_adc/adc_oneshot.h>
#include <driver/gpio.h>
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
int btn_cur_state = -1;   // подтверждённое состояние кнопок: -1 отпущено, 0..5 кнопка
const uint32_t DEBOUNCE_MS = 50;

// Системная кнопка (6-я)
uint32_t system_button_press_start_ms = 0;
bool system_button_was_pressed = false;
const uint32_t FACTORY_RESET_HOLD_MS = 3000;

// =====================================================
// =============== ZIGBEE CALLBACKS ====================
// =====================================================

// Защита от петли: setLight() сам дёргает onLightChange
static volatile bool selfReport = false;
static void reportLight(ZigbeeLight *ep, bool value) {
  selfReport = true;
  ep->setLight(value);
  selfReport = false;
}

// Флаг состояния сети
static bool zbOnline = false;

// ---- Очередь запросов из HA (колбэк только кладёт, работу делает loop) ----
enum ReqType : uint8_t { REQ_RELAY1, REQ_RELAY2, REQ_USB, REQ_FAN };
struct Req { ReqType type; bool value; float fval; };
static const uint8_t Q_SIZE = 16;
static volatile Req  q_buf[Q_SIZE];
static volatile uint8_t q_head = 0, q_tail = 0;

static void queuePush(ReqType t, bool v, float f = 0.0f) {
  uint8_t next = (uint8_t)((q_head + 1) % Q_SIZE);
  if (next == q_tail) return;          // очередь полна — молча теряем
  q_buf[q_head].type = t;
  q_buf[q_head].value = v;
  q_buf[q_head].fval = f;
  q_head = next;
}

static bool queuePop(Req &r) {
  if (q_tail == q_head) return false;
  r.type  = q_buf[q_tail].type;
  r.value = q_buf[q_tail].value;
  r.fval  = q_buf[q_tail].fval;
  q_tail = (uint8_t)((q_tail + 1) % Q_SIZE);
  return true;
}

void onRelay1Change(bool state) {
  if (selfReport) return;
  queuePush(REQ_RELAY1, state);
}

void onRelay2Change(bool state) {
  if (selfReport) return;
  queuePush(REQ_RELAY2, state);
}

void onUSBChange(bool state) {
  if (selfReport) return;
  queuePush(REQ_USB, state);
}

// Пустой колбэк для кнопок-лайтов (убирает warning, тумблер локальный)
void onButtonLightChange(bool state) {
  (void)state;
}

// Вентилятор (analog 0..100 -> скорость 0..10)
void onFanChange(float value) {
  if (selfReport) return;
  queuePush(REQ_FAN, false, value);
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

  // ADC-драйвер сбрасывает подтяжку пина — возвращаем её ПОСЛЕ настройки канала.
  // Без внешней лестницы вход иначе болтается и ловит наводки.
  gpio_set_pull_mode((gpio_num_t)BUTTONS_ADC_PIN, GPIO_PULLUP_ONLY);
  gpio_pullup_en((gpio_num_t)BUTTONS_ADC_PIN);

  Serial.println("[ADC] Initialized on GPIO1 (pull-up on)");
}

static int read_button_adc() {
  if (!adc_handle) return 4095;
  // 5 чтений, берём среднее — гасит шум на неподключённом/длинном входе
  int sum = 0, v = 0;
  for (int k = 0; k < 3; k++) {
    adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &v);
    sum += v;
  }
  return sum / 3;
}

// Окна ADC — откалибровано по реальным замерам на плате.
// Замеры: к1=257..264  к2=739..747  к3=1327..1359  к4=1980..2005
//         к5=2523..2702 (плавает)   к6=3039..3175   отпущено=3393..3477
static const int BTN_LO[6] = {  160,  640, 1220, 1870, 2440, 2960 };
static const int BTN_HI[6] = {  380,  850, 1470, 2120, 2790, 3260 };

// Кнопка засчитывается ТОЛЬКО если ADC попал в её окно.
// Всё остальное (в т.ч. отпущенное ~3400) -> "не нажато".
static int detect_button_press(int adc_value) {
  for (int i = 0; i < 6; i++) {
    if (adc_value >= BTN_LO[i] && adc_value <= BTN_HI[i]) return i;
  }
  return -1;
}

// Кнопки 1-5: нажатие -> Contact "closed" (сработал), потом отпускание -> "open"
static void on_button_pressed(int button_idx) {
  if (button_idx < 0 || button_idx > 4) return;
  Serial.printf("[BTN] Button %d pressed\n", button_idx + 1);
  if (!zb_ready) return;
  // импульс: нажал -> ON (в HA видно срабатывание)
  btn_light_state[button_idx] = true;
  reportLight(zbButtons[button_idx], true);
}

static void on_button_released(int button_idx) {
  if (button_idx < 0 || button_idx > 4) return;
  if (!zb_ready) return;
  // отпустили -> OFF, кнопка не залипает в HA
  btn_light_state[button_idx] = false;
  reportLight(zbButtons[button_idx], false);
  Serial.printf("[BTN] Button %d released\n", button_idx + 1);
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
    Serial.println("[TEMP] Ошибка CRC — плохой контакт или помеха");
    temp_sensor_ok = false;
  } else if (ec == OneWireNg::EC_NO_DEVS) {
    Serial.println("[TEMP] Датчик не найден на шине (проверь GPIO12 и подтяжку 4.7k к 3.3V)");
    temp_sensor_ok = false;
  } else {
    Serial.printf("[TEMP] Ошибка чтения, код %d\n", (int)ec);
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
  new (&ow) OneWireNg_CurrentPlatform(PSU_TEMP_PIN, true);  // true = внутренняя подтяжка
  {
    DSTherm drv(ow);
    drv.writeScratchpadAll(0, 0, DSTherm::RES_12_BIT);  // 12 бит разрешение
    drv.copyScratchpadAll(false);
  }
  // пробное чтение — сразу видно, есть датчик или нет
  {
    DSTherm drv(ow);
    drv.convertTempAll(DSTherm::MAX_CONV_TIME, false);
    static PlaceholderInit<DSTherm::Scratchpad> sp0;
    OneWireNg::ErrorCode ec0 = drv.readScratchpadSingle(sp0);
    if (ec0 == OneWireNg::EC_SUCCESS) {
      Serial.printf("[TEMP] DS18B20 найден, сейчас %.2f C\n", (float)sp0->getTemp2() / 16.0f);
    } else {
      Serial.printf("[TEMP] DS18B20 НЕ найден (код %d). Проверь: GPIO12, подтяжка 4.7k к 3.3V, питание датчика\n", (int)ec0);
    }
  }

  // === ZIGBEE ENDPOINTS ===
  zbRelay1.setManufacturerAndModel("VITAZGIO", "ReleTable");
  zbRelay1.setPowerSource(ZB_POWER_SOURCE_MAINS);
  zbRelay1.onLightChange(onRelay1Change);
  Zigbee.addEndpoint(&zbRelay1);

  zbRelay2.setManufacturerAndModel("VITAZGIO", "ReleTable");
  zbRelay2.setPowerSource(ZB_POWER_SOURCE_MAINS);
  zbRelay2.onLightChange(onRelay2Change);
  Zigbee.addEndpoint(&zbRelay2);

  zbUSB.setManufacturerAndModel("VITAZGIO", "ReleTable");
  zbUSB.setPowerSource(ZB_POWER_SOURCE_MAINS);
  zbUSB.onLightChange(onUSBChange);
  Zigbee.addEndpoint(&zbUSB);

  // Вентилятор (analog output)
  zbFan.setManufacturerAndModel("VITAZGIO", "ReleTable");
  zbFan.setPowerSource(ZB_POWER_SOURCE_MAINS);
  zbFan.addAnalogOutput();
  zbFan.onAnalogOutputChange(onFanChange);
  Zigbee.addEndpoint(&zbFan);

  // Датчик температуры
  zbTemp.setManufacturerAndModel("VITAZGIO", "ReleTable");
  zbTemp.setPowerSource(ZB_POWER_SOURCE_MAINS);
  zbTemp.setMinMaxValue(-10, 125);
  zbTemp.setTolerance(0.5);
  Zigbee.addEndpoint(&zbTemp);

  // Кнопки 1-5 (ContactSwitch)
  for (int i = 0; i < 5; i++) {
    zbButtons[i]->setManufacturerAndModel("VITAZGIO", "ReleTable");
    zbButtons[i]->setPowerSource(ZB_POWER_SOURCE_MAINS);
    zbButtons[i]->onLightChange(onButtonLightChange);
    Zigbee.addEndpoint(zbButtons[i]);
  }

  // === СТАРТ ZIGBEE (Router) ===
  // НЕ блокируем setup ожиданием сети — стек стартует, loop работает дальше.
  Serial.println("[ZB] Starting Zigbee Router...");
  if (!Zigbee.begin(ZIGBEE_ROUTER)) {
    Serial.println("[ZB] Stack failed to start. Check zigbee.csv / -DZIGBEE_MODE_ZCZR");
  } else {
    Serial.println("[ZB] Stack started, searching network...");
  }

  Serial.println("[SETUP] Ready!");
}

// =====================================================
// =============== LOOP ================================
// =====================================================

void loop() {
  uint32_t now = millis();

  // ---- следим за подключением к сети (как в рабочем проекте магнитолы) ----
  {
    bool online = Zigbee.connected();
    if (online != zbOnline) {
      zbOnline = online;
      if (online) {
        Serial.println("[ZB] Подключились к сети");
        // синхронизируем всё, что видит HA
        reportLight(&zbRelay1, relay_state[0]);
        reportLight(&zbRelay2, relay_state[1]);
        reportLight(&zbUSB, usb_power_state);
        for (int i = 0; i < 5; i++) reportLight(zbButtons[i], btn_light_state[i]);
        zb_ready = true;
      } else {
        Serial.println("[ZB] Связь с сетью потеряна");
        zb_ready = false;
      }
    }
  }

  // ---- разбираем очередь запросов из HA ----
  Req r;
  while (queuePop(r)) {
    switch (r.type) {
      case REQ_RELAY1:
        relay_state[0] = r.value;
        digitalWrite(RELAY_1_PIN, r.value ? LOW : HIGH);   // active-LOW
        Serial.printf("[ZB] Relay 1 -> %s\n", r.value ? "ON" : "OFF");
        break;
      case REQ_RELAY2:
        relay_state[1] = r.value;
        digitalWrite(RELAY_2_PIN, r.value ? LOW : HIGH);   // active-LOW
        Serial.printf("[ZB] Relay 2 -> %s\n", r.value ? "ON" : "OFF");
        break;
      case REQ_USB:
        usb_power_state = r.value;
        digitalWrite(MOSFET_USB_PIN, r.value ? HIGH : LOW);
        Serial.printf("[ZB] USB -> %s\n", r.value ? "ON" : "OFF");
        break;
      case REQ_FAN: {
        int level = (int)round(r.fval / 10.0f);
        if (level < 0) level = 0;
        if (level > 10) level = 10;
        current_fan_speed = level;
        ledcWrite(FAN_PWM_PIN, FAN_SPEED_PWM[level]);
        Serial.printf("[ZB] Fan -> %.0f%% (level %d)\n", r.fval, level);
        break;
      }
    }
  }

  // подсказка если сети нет
  static uint32_t lastHint = 0;
  if (!zbOnline && now - lastHint > 30000) {
    lastHint = now;
    Serial.println("[ZB] Нет сети. Открой Permit Join в Zigbee2MQTT.");
  }

  // кнопки опрашиваем раз в 50 мс — не грузим стек
  static uint32_t last_adc_ms = 0;
  static int adc_value = 4095;
  // btn_cur_state объявлен глобально (виден в диагностике)
  static int cand = -1;           // кандидат
  static int cand_cnt = 0;

  if (now - last_adc_ms >= 50) {
    last_adc_ms = now;
    adc_value = read_button_adc();

    int raw = detect_button_press(adc_value);

    // подтверждение: 4 одинаковых опроса подряд (200 мс)
    if (raw == cand) {
      if (cand_cnt < 4) cand_cnt++;
    } else {
      cand = raw;
      cand_cnt = 1;
    }

    // КАЛИБРОВКА: печатаем ADC когда значение заметно отличается от "отпущено"
    static int last_shown = -9999;
    if (adc_value < 3200 && abs(adc_value - last_shown) > 60) {
      last_shown = adc_value;
      Serial.printf("[CAL] ADC=%d  (окно кнопки: %d..%d)\n",
                    adc_value, adc_value - 120, adc_value + 120);
    }

    // меняем состояние только по подтверждённому кандидату
    if (cand_cnt >= 4 && cand != btn_cur_state) {
      int prev = btn_cur_state;
      btn_cur_state = cand;

      // Событие ТОЛЬКО при переходе "отпущено (-1)" -> "кнопка"
      if (prev == -1 && btn_cur_state >= 0 && btn_cur_state <= 4) {
        on_button_pressed(btn_cur_state);
      }
      // Отпускание: кнопка -> отпущено (или другая кнопка) => гасим прошлую
      if (prev >= 0 && prev <= 4 && btn_cur_state != prev) {
        on_button_released(prev);
      }
      // системная кнопка (6-я)
      if (prev == -1 && btn_cur_state == 5) {
        handle_system_button(5);
      }
      if (prev == 5 && btn_cur_state == -1) {
        handle_system_button(-1);
      }
      last_button = btn_cur_state;
    }

    // удержание системной кнопки (для Factory Reset по таймеру)
    if (btn_cur_state == 5) {
      handle_system_button(5);
    }
  }

  temperature_read();

  static uint32_t last_print = 0;
  if (now - last_print > 3000) {
    Serial.printf("[STAT] ADC=%4d btn=%d rel:%d %d usb:%d fan:%d temp:%.1f zb:%d\n",
                  adc_value, btn_cur_state, relay_state[0], relay_state[1],
                  usb_power_state, current_fan_speed, last_temp, Zigbee.connected());
    last_print = now;
  }

  delay(5);
}