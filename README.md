# README
Устройство работает в двух режимах
- При подключении к ПК по USB, работает как 2 виртуальных COM-порта и 1 I2C-порт
- При подаче только питания определяет подключение датчиков к портам и выводит информацию на дисплей

# [probe-run](https://github.com/knurling-rs/probe-run)
1. rb - build and flash
2. rrb - build release and flash

# Connections

## 485-UART (UART2)
| Pin | Usage |
| --- | --- |
| `PA2` | TX |
| `PA3` | RX |
| `PA1` | RE/DE |

## UART (UART1)
| Pin | Usage |
| --- | --- |
| `PB6` | TX |
| `PB7` | RX |

## Sensor I2C (I2C1)
| Pin | Usage |
| --- | --- |
| `PB8` | SCL |
| `PB9` | SDA |

## Current measure I2C (I2C2)
| Pin | Usage |
| --- | --- |
| `PB10` | SCL |
| `PB11` | SDA |

## Dispaly SSD1309 (SPI1)
| Pin | Usage |
| --- | --- |
| `PA5` | SCK |
| `PA7` | MOSI |
| `PA4` | NSS |
| `PB1` | DC |
| `PB0` | RES |

## USB
| Pin | Usage |
| --- | --- |
| `PA11` | D- |
| `PA12` | D+ |
| `PULL-UP` | `PA10` |

## SWD
| Pin | Usage |
| --- | --- |
| `PA13` | SWDIO |
| `PA14` | SWCLK |

## Мост I2C
Передача команд и прием данных происходит через HID-репорты. Поскольку библиотека `usbd-hid` не поддерживает чтение `Feature`-репортов, пришлось сделать весь обмен через единственную точку обмена и упростить дескриптор HID до предела.
Передача происходит в виде транзакций Запрос-ответ. При этом в ответе возвращается код ошибки.

| Назначение              | Запрос                   | Ответ               | Описание                                                                                      |
| ----------------------- | ------------------------ | ------------------- | --------------------------------------------------------------------------------------------- |
| Запись данных в I2C     | `0xA0 NN dd [xx xx ...]` | `RR`                | `NN` - количество байт (1-61), `dd` - Адрес устройства I2C (7 бит), `xx` - Данные (если есть) |
| Чтение данных из I2C    | `0xB0 NN dd`             | `RR NN xx [xx ...]` | `NN` - количество байт (1-60), `dd` - Адрес устройства I2C (7 бит), `xx` - Данные (1+ байт)   |
| Сброс шины I2C          | `0x69`                   | `RR`                | Перезапускает шину I2C                                                                        |
| Установить скорость I2C | `0x10 LL MM`             | `RR`                | `[LL, MM] -> u16` - новое значение скорости шины в диапазоне 1-400 kHz                        |

> Возможные коды ошибок

| Название           | Код               | Описание                                                        |
| ------------------ | ----------------- | --------------------------------------------------------------- |
| Ok                 | `0x00`            | Успешное выполнение команды                                     |
| LengthError        | `0x80`            | Запрошено неверное количество байт на чтение/запись (<1 \| >60) |
| InvalidCommand(u8) | `0x80 + command`  | Неизвестная команда                                             |
| NotSupported       | `0x82`            | Команда не поддерживается                                       |
| HwError(HwError)   | `0xC0 + hw_error` | Ошибка аппаратной части                                         |

> Возможные ошибки аппаратной части (зависит от аппаратной части микроконтроллера)

| Название           | Код      | Описание                                               |
| ------------------ | -------- | ------------------------------------------------------ |
| I2cBusError        | `1 << 0` | Ошибка шины I2C                                        |
| I2cArbitrationLost | `1 << 1` | Ошибка Арбитража шины I2C                              |
| Acknowledge        | `1 << 2` | Ошибка подтверждения шины I2C (нет ответа)             |
| Overrun            | `1 << 3` | Ошибка переполнения буфера                             |
| Timeout            | `1 << 4` | Операция не выполнена за отведенный промежуток времени |

## Макет
![Схема](./maket_board.pdf)

| Назначение | Пин |
| --- | --- |
| `SDA` | `PB7` |
| `SCL` | `PB6` |
| `Sensor_VCC` | `Direct +5v` |
| `Sensor_GND` | `P1-4` |
| `DISPLAY_VCC` | `P4-1 (+3.3v)` |
| `DISPLAY_GND` | `P4-5` |
| `DISPLAY_CS` | `P4-2 (PA1)` |
| `DISPLAY_DC` | `P4-3 (PA3)` |
| `DISPLAY_RES` | `P4-4 (PA2)` |
| `DISPLAY_SCK` | `P2-2 (SPI1_SCK)` |
| `DISPLAY_SDA` | `P2-4 (SPI1_MOSI)` |