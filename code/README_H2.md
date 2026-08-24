# ESP32-H2 «Реле под столом» — Zigbee Router

Рабочая прошивка. Устройство в Zigbee2MQTT как **VITAZGIO / ReleTable**.

---

## Распиновка

| GPIO | Что | Сущность в z2m |
|------|-----|----------------|
| GPIO4 | Реле 1 | relay1 |
| GPIO5 | Реле 2 | relay2 |
| GPIO10 | MOSFET (USB-питание) | usb |
| GPIO1 | Кнопки (резист. лестница) | btn1–btn5 |
| GPIO12 | DS18B20 | temp |
| GPIO14 | PWM вентилятор | fan (0–100%) |
| GPIO13 | Синий LED | — |
| GPIO8 | RGB-LED / **strapping** | только для прошивки |

Свободно: GPIO11, GPIO22.

---

## Подключение

### Реле (модуль SONGLE, active-LOW)
```
GPIO4 → IN1
GPIO5 → IN2
GND   → GND модуля
5V    → VCC модуля
```

### MOSFET (USB)
```
GPIO10 → TRIG
GND    → GND мосфета
VIN+/− → 5V питание
OUT+/− → USB-разъём (+5V / GND)
```

### Кнопки — резистивная лестница (pull-up 10k к 3.3V)
```
        +3.3V
          │
        [10k] pull-up
          │
GPIO1 ────┼──[680]──[Кн1]── GND   btn1
          ├──[2k]───[Кн2]── GND   btn2
          ├──[4.7k]─[Кн3]── GND   btn3
          ├──[10k]──[Кн4]── GND   btn4
          ├──[20k]──[Кн5]── GND   btn5
          └──[68k]──[Кн6]── GND   системная (Permit Join / Reset)
```
Пороги ADC: 471 / 995 / 1678 / 2389 / 3153 / 3829.

### DS18B20
```
DATA → GPIO12 + резистор 4.7k к 3.3V (обязательно)
VCC  → 3.3V
GND  → GND
```

### Вентилятор
```
GPIO14 → PWM-вход
GND    → GND
+12V   → отдельное питание
```

---

## ⚡ ПИТАНИЕ (важно!)

- Плату питать от **отдельного стабильного 5V БП**, НЕ от USB-программатора.
- **Реле питать своим 5V** (катушки дают просадку → плата отваливается от Zigbee).
- Конденсаторы у платы между 5V и GND: **2×470мкФ 16V параллельно** (или 1000мкФ) + керамика 0.1мкФ.
- Без конденсаторов плата ребутится/мигает в сети при щелчке реле.

---

## Прошивка (ESP32-H2 SuperMini)

Встроенный USB платы НЕ работает. Прошивать через **YP-01 (PL2303)**:
```
YP-01 TXD → GPIO23 (U0RXD)
YP-01 RXD → GPIO24 (U0TXD)
YP-01 GND → GND
YP-01 3V3 → 3V3      (3V3 и 5V одновременно НЕ подключать)
```

### Вход в bootloader — ГЛАВНАЯ ЗАСАДА
Одного BOOT недостаточно. **GPIO8 обязан быть HIGH.**

1. **GPIO8 → 3V3 через резистор 10k** (припаять насовсем; напрямую нельзя — на GPIO8 висит RGB-LED, будет КЗ).
2. Зажать **BOOT** → нажать/отпустить **RESET** → отпустить BOOT.
3. В мониторе: `waiting for download`.
4. **Закрыть монитор порта** (иначе Upload падает — COM занят).
5. Upload.

### Сборка
```
pio run -e esp32-h2
pio run -e esp32-h2 -t upload
```

### Стереть флеш (при смене сети / если врёт "Connected" к старой сети)
```
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" `
  "$env:USERPROFILE\.platformio\packages\tool-esptoolpy\esptool.py" `
  --chip esp32h2 --port COM13 erase_flash
```
После стирания в логе должно быть `factory-reset mode` → делает настоящий join.

---

## Подключение к Zigbee2MQTT

1. Файл `reletable.js` → в папку z2m (рядом с `configuration.yaml`, туда же где `korona.js`).
2. В `configuration.yaml`:
   ```yaml
   external_converters:
     - reletable.js
   ```
3. Перезапустить z2m.
4. В z2m нажать **«Разрешить присоединение» (Permit Join)**.
5. Прошитая плата (после erase_flash) сама сделает join → появится как **VITAZGIO / ReleTable**.

Кнопка 6 (68k) в z2m не видна — она системная: короткое нажатие = Permit Join, удержание 3с = Factory Reset.

---

## Диагностика

| Симптом | Причина / лечение |
|---|---|
| `Delivery failed` / плата мигает в сети | просадка питания при щелчке реле → конденсаторы + отдельное 5V |
| `non factory-reset mode`, в z2m не появляется | не стёрт флеш → erase_flash |
| `Network steering not successful` в цикле | Permit Join закрыт в z2m, либо далеко от координатора |
| btn1 сам ON на пустой плате | лестница не подключена, вход шумит — норма, уйдёт после подключения |
| `waiting for download` | плата в bootloader — закрыть монитор, прошить |

---

**Проект:** ESP32-H2 Реле под столом · Zigbee Router · VITAZGIO
