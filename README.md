# ESP32 MPU6050 Robotic Arm

An ESP32 robotic-arm controller that reads roll, pitch, and gyro data from an
MPU6050 and maps the movement to four servos:

- Base rotation
- Shoulder
- Elbow
- Claw

The firmware also calculates a simple forward-kinematics position and prints
the end-effector position, movement vector, direction, speed, and state to the
Serial Monitor.

## Hardware

### Required parts

- ESP32 Dev Module
- MPU6050 module
- Four hobby servos
- Robotic-arm mechanism
- Separate regulated 5 V servo power supply
- Jumper wires

### Wiring

| Device | Pin | ESP32 |
|---|---:|---:|
| MPU6050 VCC | VCC | 3V3 |
| MPU6050 GND | GND | GND |
| MPU6050 SDA | SDA | GPIO 21 |
| MPU6050 SCL | SCL | GPIO 22 |
| Base servo signal | Signal | GPIO 13 |
| Shoulder servo signal | Signal | GPIO 12 |
| Elbow servo signal | Signal | GPIO 14 |
| Claw servo signal | Signal | GPIO 27 |

Connect every servo ground to ESP32 ground. Power the servos from the
external 5 V supply, not from the ESP32 3V3 pin or USB 5 V rail. The external
servo-supply ground and ESP32 ground must be common.

> Servo wire colors vary by manufacturer. Confirm signal, 5 V, and GND from
> the servo datasheet before connecting power.

## Software setup

1. Install [Visual Studio Code](https://code.visualstudio.com/).
2. Install the PlatformIO IDE extension.
3. Open this repository folder in VS Code.
4. Connect the ESP32 by USB.
5. Open `platformio.ini` and verify the environment:

   ```ini
   [env:esp32dev]
   platform = espressif32
   board = esp32dev
   framework = arduino
   monitor_speed = 115200
   lib_deps =
       madhephaestus/ESP32Servo
   ```

6. Let PlatformIO download the ESP32 platform, Arduino framework, and
   `ESP32Servo` library automatically.
7. Select the correct serial port in PlatformIO if more than one is present.
8. Click **Build**.
9. Click **Upload**.
10. Open the Serial Monitor at **115200 baud**.

The expected startup message is:

```text
Robotic arm ready. Sending coordinates and state...
```

## How the firmware works

1. `Wire.begin(21, 22)` starts I2C using GPIO 21 for SDA and GPIO 22 for SCL.
2. The MPU6050 is woken through register `0x6B`.
3. Four servo objects are attached using `ESP32Servo`.
4. The accelerometer values are converted into roll and pitch angles.
5. A deadzone prevents small sensor noise from moving the arm.
6. Roll controls base rotation.
7. Pitch controls the shoulder and elbow targets.
8. Gyro Z rotation controls claw opening and closing.
9. Servo targets are approached gradually using smoothing factors.
10. Forward kinematics calculates the estimated X, Y, and Z position.
11. Position and movement data are printed every 200 ms.

## Calibration

Before attaching the mechanical links:

1. Upload the firmware with the servos disconnected or with the arm unloaded.
2. Confirm that the initial positions are safe:
   - Base: 90 degrees
   - Shoulder: 120 degrees
   - Elbow: 90 degrees
   - Claw: 90 degrees
3. Change the servo limits near the top of `src/main.cpp` to match the
   mechanism.
4. Adjust `L1` and `L2` to the actual arm link lengths in centimetres.
5. Mount the MPU6050 firmly and keep its orientation consistent.
6. Adjust `ROLL_SENS`, `PITCH_SENS`, and `ACC_DEADZONE` to reduce unwanted
   movement.
7. If a servo moves in the wrong direction, invert its mapping or swap the
   corresponding mechanical linkage direction.

Never test a servo against a hard mechanical stop. Excess current can reset
the ESP32 or damage the servo.

## Problems encountered while replicating

### `Wire.h` or `ESP32Servo.h` cannot be found

The project originally used the ESP-IDF framework while the source used
Arduino APIs. `Wire.h`, `Servo`, `String`, `Serial`, and `constrain()` require
the Arduino framework. This repository uses:

```ini
framework = arduino
```

PlatformIO installs the library listed in `lib_deps`; do not manually copy
library files into `src`.

### The source must be C++

The Arduino servo library uses C++ classes. The source file must be named
`main.cpp`, not `main.c`. A `.c` file causes errors for `Servo`, `String`, and
other C++ constructs.

### The ESP32 resets when servos move

This is usually a power problem. Four servos can draw more current than USB
or the ESP32 board regulator can provide. Use a separate regulated 5 V supply,
connect grounds together, and add a suitable bulk capacitor near the servo
power distribution if necessary.

### MPU6050 readings are missing

- Check VCC and GND.
- Verify SDA/SCL are not swapped.
- Confirm that the module is using address `0x68`.
- Modules with AD0 pulled high use address `0x69`; change `MPU_addr` if needed.
- Check that the I2C pull-up resistors are suitable for 3.3 V operation.

### Servos move erratically

- Use a stable external supply.
- Make sure all grounds are common.
- Keep servo power wires separate from I2C signal wires where practical.
- Increase `ACC_DEADZONE`.
- Reduce `ROLL_SENS` or `PITCH_SENS`.
- Check that the MPU6050 is mechanically fixed and not vibrating.

### The arm direction is reversed

Sensor axes depend on how the MPU6050 is mounted. Change the sign of the
roll or pitch mapping in `loop()` after confirming the physical axis
orientation.

### Build errors mention missing helper functions

This project declares `readMPU()`, `smoothMove()`, and
`detectStateFromTargets()` before `setup()` because their implementations are
below `loop()`. Keep those declarations if the functions are moved.

## Project structure

```text
.
├── platformio.ini
├── src/
│   └── main.cpp
├── include/
├── lib/
└── test/
```

The `.pio` build directory is intentionally ignored and should not be
committed.

## Build verification

From a terminal with PlatformIO available:

```powershell
pio run
```

The current project builds successfully for the `esp32dev` board.

## Safety

- Remove power before changing wiring.
- Keep fingers clear of moving joints.
- Use servo limits appropriate for the physical arm.
- Do not power servos from the ESP32 3V3 output.
- Test one servo and one movement at a time during initial setup.
