# RF GPS Time Source

Firmware for an ESP32/ESP8266 controller that speaks Trimble TSIP and NMEA to a
Mini-T GPS disciplined clock, renders status pages on a 128×64 SSD1306 OLED, and
offers a transparent USB-to-Mini-T serial bridge for host tools.

<img alt="Mini-T controller assembly" src="Build%20Images/device_open.png" width="420">

## Features
- Rotating on-device status pages for timing, GNSS, packet counters, and alarm detail
- Transparent bridge mode for piping TSIP/NMEA between a PC and the Mini-T
- Periodic serial log that summarizes discipline state, PPS health, and alarms
- Accepts USB commands (`0`–`3`, `B`, or `BRG`) to lock pages or toggle bridge mode
- Works on ESP32 (default) or ESP8266 by switching board definition at compile time

## Hardware List
- Trimble Mini-T GPS disciplined clock module (TSIP AB/AC capable)
- ESP32 DevKit (recommended) or NodeMCU-style ESP8266 board with native USB
- 0.96" or 1.3" 128×64 I²C OLED display based on the SSD1306 controller
- GNSS antenna for the Mini-T with suitable coax/feed
- 3.3 V wiring harness for Mini-T serial (TXD/RXD/GND) and power distribution
- USB-C/Micro-B cable for programming and runtime control
- 5v power supply with barrel jack 
- Female barrel jack connector for 5v supply

## Wiring Overview
Both the ESP board and the Mini-T speak 3.3 V logic, so connect them directly
without additional level shifting. The table lists the default ESP32 pinout; the
ESP8266 variant automatically reassigns the pins (SDA=`D2`, SCL=`D1`, RX=`D5`,
TX=`D6`).

| Mini-T / Display | ESP32 Pin | Notes |
| ---------------- | --------- | ----- |
| Mini-T TXD       | GPIO16 (RX2) | To ESP; driven by Mini-T |
| Mini-T RXD       | GPIO17 (TX2) | From ESP to Mini-T |
| Mini-T GND       | GND        | Common ground with ESP |
| Mini-T VCC       | 5V        | Power from ESP board (ensure current budget) |
| OLED SDA         | GPIO21     | I²C data (with 4.7 kΩ pull-up if needed) |
| OLED SCL         | GPIO22     | I²C clock |
| OLED VCC         | 3V3        | From ESP board |
| OLED GND         | GND        | Shared ground |

## Firmware Setup
1. Install the Arduino IDE (or PlatformIO) with the ESP32 or ESP8266 core.
2. Install the required libraries via Library Manager:
   - Adafruit SSD1306
   - Adafruit GFX
3. Open `ESP_Controller.ino`, choose the matching board (ESP32 Dev Module or
   NodeMCU 1.0), and confirm the serial port.
4. Compile and upload as usual. On first boot the display shows a splash screen,
   then cycles pages every 5 seconds.

## Serial Controls
- `0`–`3`: Lock the display on Timing, GNSS, Packets, or Alarm page.
- `B` or `BRG`: Toggle bridge mode. When enabled the ESP transparently forwards
  USB serial bytes to the Mini-T and echoes responses back.
- While in normal mode the firmware emits a status line every 5 seconds with TSIP
  decode results, alarm flags, and UTC time.

## Tips
- Ensure the Mini-T has a clear sky view; the alarm page will highlight conditions
  such as antenna faults or loss of discipline.
- The OLED header lamps indicate GNSS fix (right, large circle) and PPS quality
  (left, small circle) for quick at-a-glance health checks.
- If you prefer a static page, send `0`–`3`; the firmware continues to advance
  automatically after power cycles.

## License
This project is released under the MIT License. See `LICENSE` for details.
