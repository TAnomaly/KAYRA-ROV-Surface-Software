# KAYRA ROV — Onboard Controller (Raspberry Pi 5)

C-based real-time onboard controller that runs on the Raspberry Pi 5 inside
the ROV.  Receives MAVLink commands from the Surface Control PC, drives
6 thrusters via PCA9685 PWM, and streams camera video back.

---

## Data Flow

```
PC (192.168.2.1)                       RPi 5 (192.168.2.2)
                                       
 surface-control ──UDP:14550──►  main.c (this software)
   MAVLink MANUAL_CONTROL               │
                                        ├─► MAVLink parse
                                        ├─► 6-thruster mixer
                                        ├─► PCA9685 I2C → ESC → motors
                                        ├─► Heartbeat → back to PC
                                        └─► Camera H264 → UDP:5600 → PC
```

## Thruster Layout

```
       FL ╲     ╱ FR        Horizontal (45° vectored)
            ╲ ╱
           [ROV]
            ╱ ╲
       BL ╱     ╲ BR

        VL         VR        Vertical (heave)
```

| Motor | PCA9685 Channel | Function |
|-------|----------------|----------|
| FR    | 0              | Front-Right horizontal |
| FL    | 1              | Front-Left horizontal  |
| BR    | 2              | Back-Right horizontal  |
| BL    | 3              | Back-Left horizontal   |
| VL    | 4              | Vertical Left          |
| VR    | 5              | Vertical Right         |

---

## Build

```bash
# On the Raspberry Pi:
make
```

## Run

```bash
# Normal operation
./kayra-rov

# Without camera (for testing without ArduCam)
./kayra-rov --no-camera

# Dry run — no motor output (safe for desk testing)
./kayra-rov --no-pwm

# Both
./kayra-rov --no-camera --no-pwm
```

## Install as Service (auto-start on boot)

```bash
# Copy files to Pi
scp -r rpi/* pi@192.168.2.2:~/kayra-rov/

# On the Pi, build and install
ssh pi@192.168.2.2
cd ~/kayra-rov
make
sudo make install
sudo systemctl start kayra-rov

# Check status
sudo systemctl status kayra-rov
sudo journalctl -u kayra-rov -f
```

---

## Hardware Wiring

### PCA9685 → RPi 5 (I2C)

| PCA9685 | RPi GPIO |
|---------|----------|
| VCC     | 3.3V (pin 1) |
| GND     | GND (pin 6)  |
| SDA     | GPIO 2 / SDA (pin 3) |
| SCL     | GPIO 3 / SCL (pin 5) |
| V+      | ESC power supply (external, NOT from Pi) |

### PCA9685 → ESCs

| PCA9685 Ch | ESC Signal Wire |
|-----------|----------------|
| CH0       | FR ESC signal  |
| CH1       | FL ESC signal  |
| CH2       | BR ESC signal  |
| CH3       | BL ESC signal  |
| CH4       | VL ESC signal  |
| CH5       | VR ESC signal  |

**Important:** ESC ground must be common with PCA9685 ground.

### Enable I2C on RPi

```bash
sudo raspi-config
# Interface Options → I2C → Enable

# Verify
i2cdetect -y 1
# Should show 0x40 for PCA9685
```

---

## Configuration

All parameters in `include/config.h`:

- Network: listen port, GCS IP, camera port
- PCA9685: I2C bus/address, PWM frequency, pulse widths
- Motor channels: PCA9685 channel assignment
- Timing: loop rate, failsafe timeout
- Camera: resolution, framerate

---

## Project Structure

```
rpi/
├── Makefile
├── README.md
├── kayra-rov.service          systemd auto-start
├── .gitignore
├── include/
│   ├── config.h               All tuneable parameters
│   ├── mavlink_parser.h       MAVLink v2 parser
│   ├── pca9685.h              PCA9685 I2C PWM driver
│   ├── mixer.h                6-thruster mixing matrix
│   └── camera.h               GStreamer camera stream
└── src/
    ├── main.c                 Main loop
    ├── mavlink_parser.c       MAVLink parse + heartbeat pack
    ├── pca9685.c              PCA9685 register-level driver
    ├── mixer.c                4 DOF → 6 motor mixing
    └── camera.c               Fork + exec GStreamer pipeline
```
