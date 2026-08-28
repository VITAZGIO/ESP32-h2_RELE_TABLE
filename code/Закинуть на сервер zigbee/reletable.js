const {onOff, temperature, numeric, linkQuality} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['ReleTable'],
    model: 'ReleTable',
    vendor: 'VITAZGIO',
    description: 'ESP32-H2 реле под столом (2 реле, USB, вентилятор, температура, 5 кнопок)',
    extend: [
        // Качество связи (LQI). БЕЗ reporting — иначе configure падает
        // с UNREPORTABLE_ATTRIBUTE на genBasic.zclVersion.
        linkQuality(),
        // Реле 1 (endpoint 10)
        onOff({endpointNames: ['relay1'], powerOnBehavior: false}),
        // Реле 2 (endpoint 11)
        onOff({endpointNames: ['relay2'], powerOnBehavior: false}),
        // USB (endpoint 12)
        onOff({endpointNames: ['usb'], powerOnBehavior: false}),
        // Кнопки 1-5 (endpoints 20-24) как on/off
        onOff({endpointNames: ['btn1'], powerOnBehavior: false}),
        onOff({endpointNames: ['btn2'], powerOnBehavior: false}),
        onOff({endpointNames: ['btn3'], powerOnBehavior: false}),
        onOff({endpointNames: ['btn4'], powerOnBehavior: false}),
        onOff({endpointNames: ['btn5'], powerOnBehavior: false}),
        // Датчик температуры (endpoint 14)
        // min 10с, max 300с, изменение 0.5°C
        temperature({
            endpointNames: ['temp'],
            reporting: {min: 10, max: 300, change: 50},
        }),
        // Вентилятор (endpoint 13) — analog output, 0..100%
        numeric({
            name: 'fan_speed',
            cluster: 'genAnalogOutput',
            attribute: 'presentValue',
            endpointName: 'fan',
            valueMin: 0,
            valueMax: 100,
            valueStep: 10,
            unit: '%',
            description: 'Скорость вентилятора',
            access: 'ALL',
        }),
    ],
    endpoint: (device) => {
        return {
            relay1: 10,
            relay2: 11,
            usb: 12,
            fan: 13,
            temp: 14,
            btn1: 20,
            btn2: 21,
            btn3: 22,
            btn4: 23,
            btn5: 24,
        };
    },
    meta: {multiEndpoint: true},
};

module.exports = definition;
