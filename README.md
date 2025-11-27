# StandardFirmataPishiAi

Enhanced StandardFirmata by Pishi.ai enabling Arduino to connect to Scratch via either USB (WebSerial) or external BLE-UART/Serial modules (WebBluetooth/WebSerial). Ideal for Uno/Nano/Mega/Leonardo/Due/Zero and projects needing flexible wired/wireless serial.

## Overview
- **Serial/BLE:** Works with BLE-UART modules (HM-10/11, JDY-08/10/23/24, etc.) and standard UART adapters.
- **Pin protection:** Protect RX/TX pins used for external modules from being altered by Scratch blocks.
- **Multiple modes:** Built-in USB, SoftwareSerial, or hardware UARTs.
- **Defaults:** RX `4`, TX `7`; USB `57600`, SoftwareSerial `9600`.

## Communication Modes
- `SERIAL_MODE 0` — Built-in USB only (Hardware Serial)
- `SERIAL_MODE 1` — SoftwareSerial only (external BLE/serial module) on `RX 4`, `TX 7`
- `SERIAL_MODE 2` — USB + SoftwareSerial (dual-channel, recommended)
- `SERIAL_MODE 3` — USB + Hardware Serial (e.g., `Serial1` on multi-UART boards)

## Installation
1. Open `StandardFirmataPishiAi.ino` in Arduino IDE (`File → Open`).
2. (Optional) Set mode in code:
	- `#define SERIAL_MODE 2` (default)
3. (Optional) Adjust baud rates:
	- `#define HARDWARE_SERIAL_BAUD_RATE 57600`
	- `#define SOFTWARE_SERIAL_BAUD_RATE 9600`
4. (Optional) Change SoftwareSerial pins if needed:
	- `#define SW_SERIAL_RX_PIN 4`
	- `#define SW_SERIAL_TX_PIN 7`
5. Upload the sketch via USB.

## Wiring (External BLE/UART Module)
Mode 1 & 2 (SoftwareSerial):
- Module `VCC` → Arduino `5V/3.3V` (check module voltage)
- Module `GND` → Arduino `GND`
- Module `TX` → Arduino `RX` pin `4`
- Module `RX` → Arduino `TX` pin `7` (level shift if module is 3.3V)

Mode 3 (Hardware Serial):
- Mega: `Serial1 (19/18)`, `Serial2 (17/16)`, `Serial3 (15/14)`
- Leonardo/Due/Zero: `Serial1`

## Connect to Scratch (Pishi.ai)
1. Open `https://pishi.ai/play` and add the Arduino extension.
2. Enable protection in Scratch: `enable serial/ble on rx: [4] tx: [7]`.
3. Connect:
	- Bluetooth → for BLE-UART modules
	- USB → for built-in USB or USB-to-Serial adapters
4. Optional blocks:
	- Disable protection: `disable serial/ble and release pins`
	- Check status: `serial/ble enabled?`

## Customization
- **Change pins:** Update both firmware and Scratch block (e.g., `RX 10`, `TX 11`).
- **Increase speed:** Configure your module, then raise `SOFTWARE_SERIAL_BAUD_RATE` (e.g., `57600`).

## Troubleshooting
- No data: verify baud rate, wiring, and pin protection.
- Upload fails (Mode 1): temporarily disconnect the external module.
- Weak BLE range: typical 3–10 meters.
- Wrong pins: confirm RX/TX mapping.

## Performance Tips
- Use Mode 2 during development for flexibility.
- Use Mode 3 on multi-UART boards for best throughput.
- Power motors/servos from external supplies.

## Examples
- See `examples/MinimalDualMode/MinimalDualMode.ino` for a minimal dual-mode setup.

## License
LGPL-2.1. See `LICENSE`.
