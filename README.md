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
