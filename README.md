# ESP32 Competition Line Follower

Firmware and notes for a fast line follower using:

- ESP32-WROOM based board
- TB6612FNG motor driver
- N20 gear motors
- 16-channel analog IR sensor array with onboard multiplexer
- SSD1306 I2C OLED
- 6 buttons: Next, Prev, Enter, Back, Calibrate, Start

The current firmware lives at:

`firmware/CompetitionLineFollower/CompetitionLineFollower.ino`

Motor-only test sketch:

`firmware/MotorTest/MotorTest.ino`

## Arduino Setup

Install these Arduino libraries:

- `Adafruit SSD1306`
- `Adafruit GFX Library`

Board settings:

- Board: `ESP32 Dev Module` for most ESP32-WROOM boards
- CPU Frequency: `240MHz`
- Flash Frequency: `80MHz`
- Upload Speed: `921600` if stable, otherwise `115200`

## Important ESP32 ADC Note

The 16-channel sensor array uses one analog output, `MUX_OUT`, so the firmware only needs one ESP32 ADC pin:

`GPIO36`

GPIO36 is an ADC1 input-only pin, which is a good fit for sensor output. The mux select lines `S0-S3` are regular digital outputs.

## Complete Pin Layout

Change pin assignments near the top of the `.ino` file if needed.

| Part | Signal | ESP32 GPIO | Notes |
| --- | --- | --- | --- |
| 16ch IR array | MUX_OUT | 36 | ADC1 input-only pin |
| 16ch IR array | S0 | 32 | Mux select bit 0 |
| 16ch IR array | S1 | 33 | Mux select bit 1 |
| 16ch IR array | S2 | 25 | Mux select bit 2 |
| 16ch IR array | S3 | 26 | Mux select bit 3 |
| 16ch IR array | VIN 1 | 3.3V or sensor-rated VIN | Connect both VIN pins |
| 16ch IR array | VIN 2 | 3.3V or sensor-rated VIN | Connect both VIN pins |
| 16ch IR array | GND 1 | GND | Common ground |
| 16ch IR array | GND 2 | GND | Common ground |
| 16ch IR array | GND 3 | GND | Common ground |
| TB6612FNG | PWMA | 23 | Left motor PWM |
| TB6612FNG | AIN1 | 18 | Left motor direction |
| TB6612FNG | AIN2 | 19 | Left motor direction |
| TB6612FNG | PWMB | 5 | Right motor PWM |
| TB6612FNG | BIN1 | 16 | Right motor direction |
| TB6612FNG | BIN2 | 17 | Right motor direction |
| TB6612FNG | STBY | 4 | Driver standby enable |
| OLED | SDA | 21 | I2C data |
| OLED | SCL | 22 | I2C clock |
| Button | Next | 13 | Button to GND |
| Button | Prev | 14 | Button to GND |
| Button | Enter | 27 | Button to GND |
| Button | Back | 12 | Button to GND |
| Button | Calibrate | 35 | Button to GND, external pull-up required |
| Button | Start | 34 | Button to GND, external pull-up required |

### 16-Channel Sensor Array

| Sensor array pin | ESP32 GPIO |
| --- | --- |
| MUX_OUT | 36 |
| S0 | 32 |
| S1 | 33 |
| S2 | 25 |
| S3 | 26 |
| VIN 1 | 3.3V or sensor-rated VIN |
| VIN 2 | 3.3V or sensor-rated VIN |
| GND 1 | GND |
| GND 2 | GND |
| GND 3 | GND |

Connect both VIN pins and all three GND pins. If your sensor board is designed for 5V, confirm that `MUX_OUT` is still ESP32-safe before connecting it; ESP32 GPIO pins are not 5V tolerant.

### TB6612FNG

| TB6612FNG | ESP32 GPIO |
| --- | --- |
| PWMA | 23 |
| AIN1 | 18 |
| AIN2 | 19 |
| PWMB | 5 |
| BIN1 | 16 |
| BIN2 | 17 |
| STBY | 4 |

Connect motor supply to a separate battery rail. Connect ESP32 GND, TB6612 GND, and battery GND together.

### OLED

| OLED | ESP32 GPIO |
| --- | --- |
| SDA | 21 |
| SCL | 22 |

Default I2C address is `0x3C`.

### Buttons

Buttons are wired from GPIO to GND. The firmware uses `INPUT_PULLUP`.

GPIO34 and GPIO35 are input-only pins and do not have internal pull-ups. Use an external pull-up resistor, typically 10k from each pin to 3.3V, for the `Start` and `Calibrate` buttons.

| Button | ESP32 GPIO |
| --- | --- |
| Next | 13 |
| Prev | 14 |
| Enter | 27 |
| Back | 12 |
| Calibrate | 35 |
| Start | 34 |

GPIO 4, 5, and 12 are ESP32 boot strapping pins in this layout. They usually work, but avoid external pull-downs/pull-ups that fight the ESP32 during boot. If booting becomes unreliable, move those signals first.

## Robot Setup

1. Upload the sketch.
2. Lift the robot and press `Start` briefly to check motor direction.
3. If a motor spins backward, change `invertLeftMotor` or `invertRightMotor` in the sketch.
4. Put the robot on the track.
5. Press `Calibrate`.
6. During the 5 second calibration window, slide/rotate the sensor array across both line and background.
7. Press `Start` to run.

## Motor Test

If the motors do not move, upload:

`firmware/MotorTest/MotorTest.ino`

Open Serial Monitor at `115200` baud. The test runs:

- left motor forward/backward
- right motor forward/backward
- both motors forward/backward

If nothing moves, check:

- TB6612 `VM` has motor battery voltage.
- TB6612 `VCC` has ESP32 logic voltage, usually `3.3V`.
- ESP32 GND, TB6612 GND, and motor battery GND are connected together.
- TB6612 `STBY` is connected to GPIO4 and goes HIGH.
- Motor wires are connected to `A01/A02` and `B01/B02`.
- Battery can supply enough current for the N20 motors.

## Tuning Order

Use the OLED menu:

- `Next` / `Prev`: select menu item
- `Enter`: edit selected value
- `Next` / `Prev`: change value
- `Enter` or `Back`: save
- `Sensor view` + `Enter`: show live 16-channel line detection bars
- `Back`: exit sensor view
- `Calibrate`: recalibrate sensors
- `Start`: run/stop

In `Sensor view`, each vertical bar represents one IR channel from left to right. Taller bars mean stronger line detection after calibration. The small marker at the top shows the calculated line position used by PID.

Tune in this order:

1. Start with low `Base speed`, around `300-450`.
2. Increase `Kp` until it follows the line but starts to wiggle.
3. Increase `Kd` until the wiggle damps out.
4. Keep `Ki` at `0` for most line follower tracks.
5. Raise `Base speed` gradually.
6. Raise `Max speed` only after turns are stable.

If it turns away from the line, either:

- swap left/right motor outputs,
- flip motor inversion flags, or
- reverse the mux channel order in `readSensors()` if your board maps channel 0 to the opposite physical side.

## Mechanical Notes For Speed

- Put the sensor array slightly ahead of the wheel axle.
- Keep the center of gravity low and forward enough that the sensor height stays constant.
- Use foam/silicone tires with good grip.
- Use a battery that can supply motor stall current without browning out the ESP32.
- Power the ESP32 through a stable regulator, not directly from a noisy motor rail.
- Keep sensor height close to the track, usually 3-8 mm depending on the module.

## Competition Upgrades

The competition firmware now handles the track features that the 16-channel
downward-facing array can observe:

- Smooth, ramped launch after `Start`, rather than a full-power jerk.
- Automatic black/white line polarity change after a confirmed colour inversion.
- Short line-gap traversal by continuing the last valid steering command.
- Marked road-split preference: an edge marker before a fork selects that side
  for the following intersection.
- Wide-intersection continuation instead of treating every broad reading as a
  lost line.

The `Line color` menu item is the starting line colour. The active colour is
shown on the running OLED screen and may switch automatically in a colour
inversion section.

### Required Hardware For Full Rulebook Coverage

The rulebook also requires the robot to avoid obstacles and detect a dead-end
or end block. A downward-facing IR line array cannot detect a physical block
in front of the robot, so this cannot be solved in software with the current
parts list. Add a front-facing time-of-flight distance sensor (for example a
VL53L0X/VL53L1X) before the event. It can share the OLED's I2C bus on GPIO21
and GPIO22. Do not enable an autonomous run with obstacle/dead-end sections
until that sensor has been integrated and tested.

The firmware cannot enforce the mechanical/event rules. Before scrutineering,
verify the chassis is within 25 cm x 25 cm x 15 cm (including the 5% stated
tolerance), weighs no more than 1.5 kg (with tolerance), uses an onboard DC
power source no higher than 24 V between any two points, and has no part that
can mark or damage the track.

Once basic PID is fast and reliable:

- Add per-section speed profiling for known tracks.
- Add sharper lost-line recovery using last detected edge.
- Add intersection detection if the event includes maze solving.
- Add feed-forward turn boost based on error magnitude.
- Log sensor readings over Serial during test runs for better calibration thresholds.
