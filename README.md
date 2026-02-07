# KAYRA ROV — Surface Control Software

Production-minded PC-side Ground Control Software for the KAYRA ROV
underwater vehicle.  Reads a USB joystick via SDL2, converts inputs into
MAVLink v2 `MANUAL_CONTROL` messages, and sends them to the vehicle
over UDP or Serial (UART-over-USB).

---

## Architecture

```
┌──────────────────────────  PC (Linux)  ──────────────────────────┐
│                                                                  │
│  USB Joystick ──► SDL2 ──► Normalize ──► MAVLink Pack ──► TX     │
│                   50 Hz     Deadzone      MANUAL_CONTROL   │     │
│                             [-1,+1]       [-1000,+1000]    │     │
│                                                            │     │
│                                           HEARTBEAT 1 Hz ──┤     │
│                                                            │     │
│                      Safety Module ──────────────────────── │     │
│                      (failsafe / neutral on disconnect)    │     │
└────────────────────────────────────────────────────────────┼─────┘
                                                             │
                                           UDP :14550        │
                                        or Serial 115200     │
                                             ▼               │
                                     ┌──────────────┐
                                     │ Raspberry Pi  │
                                     │  (ROV side)   │
                                     └──────────────┘
```

## Data Flow

```
joystick  →  SDL2 poll  →  deadzone + normalize [-1.0, +1.0]
          →  axis mapping (config.h)
          →  scale to [-1000, +1000]  →  safety clamp
          →  mavlink_msg_manual_control_pack()
          →  mavlink_msg_to_send_buffer()
          →  UDP sendto()  or  Serial write()
          →  Raspberry Pi receives MAVLink frame
```

Each iteration of the main loop (50 Hz) sends one `MANUAL_CONTROL`
message.  A `HEARTBEAT` is sent at 1 Hz for MAVLink protocol compliance.

---

## Why MANUAL_CONTROL (#69)?

| Criterion | MANUAL_CONTROL | RC_CHANNELS_OVERRIDE |
|---|---|---|
| **Purpose** | GCS joystick input | Override RC transmitter channels |
| **Fields** | 4 semantic axes + buttons | 18 raw PWM channels |
| **Coupling** | Vehicle-agnostic | Tied to channel assignment |
| **ArduSub** | Native support (QGC uses it) | Supported but less idiomatic |
| **Values** | Normalised [-1000, 1000] | PWM us (typically 1000-2000) |

`MANUAL_CONTROL` is the standard joystick-to-vehicle message in the
MAVLink ecosystem.  It carries four normalised axes (`x`, `y`, `z`, `r`)
and a button bitmask — exactly what a joystick provides.  The vehicle
firmware (ArduSub) maps these to thrusters internally.

---

## Field Mapping

| Joystick | SDL axis | MAVLink field | DOF | Range |
|---|---|---|---|---|
| Left stick Y | axis 1 | `x` | Surge (fwd/back) | [-1000, +1000] |
| Left stick X | axis 0 | `y` | Sway (left/right) | [-1000, +1000] |
| Right stick Y | axis 3 | `z` | Heave (up/down) | [-1000, +1000] |
| Right stick X | axis 2 | `r` | Yaw (CW/CCW) | [-1000, +1000] |
| Buttons 0-15 | — | `buttons` | — | bitmask |

Axis indices and inversion are configurable in `include/config.h`.

---

## Project Structure

```
Surface-Control/
├── Makefile                          Build system
├── README.md                         This file
├── .gitignore                        Git ignore rules
│
├── assets/
│   └── Kayra_ROV.stl                3-D ROV model (binary STL)
│
├── include/
│   ├── config.h                      All tuneable parameters
│   ├── gui.h                         GUI interface (C-compatible)
│   ├── joystick.h                    Joystick interface
│   ├── mavlink_minimal.h             Self-contained MAVLink v2
│   ├── mavlink_packer.h              Joystick → MAVLink conversion
│   ├── safety.h                      Failsafe logic
│   └── transport.h                   UDP / Serial abstraction
│
├── src/
│   ├── main.c                        Main loop, CLI arg parsing
│   ├── gui.cpp                       Dear ImGui visual dashboard
│   ├── joystick.c                    SDL2 joystick with deadzone
│   ├── mavlink_packer.c              MAVLink message packing
│   ├── safety.c                      Watchdog and failsafe
│   └── transport.c                   UDP and Serial transport
│
├── third_party/
│   └── imgui/                        Dear ImGui (core + SDL2/GL3 backends)
│       ├── imgui.cpp / .h            Core library
│       ├── imgui_draw.cpp            Rendering primitives
│       ├── imgui_tables.cpp          Table layout
│       ├── imgui_widgets.cpp         Built-in widgets
│       └── backends/                 SDL2 + OpenGL3 backends only
│
└── build/                            Object files (generated, gitignored)
```

---

## Build Instructions

### Prerequisites

```bash
sudo apt update
sudo apt install build-essential libsdl2-dev libgl-dev
```

### Compile

```bash
make
```

### Clean

```bash
make clean
```

---

## Usage

### CLI mode (default — text status line)

```bash
./surface-control
```

### GUI mode (visual dashboard with Dear ImGui)

```bash
./surface-control --gui
```

### Custom IP and port

```bash
./surface-control --ip 10.0.0.2 --port 14555
```

### Serial (UART over USB)

```bash
./surface-control --transport serial --device /dev/ttyUSB0 --baud 115200
```

### All options

```
Usage: ./surface-control [OPTIONS]

  -g, --gui                     Launch visual dashboard
  -t, --transport <udp|serial>  Transport type     (default: udp)
  -i, --ip <address>            Target IP address  (default: 192.168.2.1)
  -p, --port <port>             UDP port           (default: 14550)
  -d, --device <path>           Serial device      (default: /dev/ttyUSB0)
  -b, --baud <rate>             Serial baud rate   (default: 115200)
  -h, --help                    Show this help
```

---

## Safety Features

| Condition | Behaviour |
|---|---|
| Joystick disconnected | All axes forced to 0 (neutral), warning printed |
| No joystick read for 500 ms | Failsafe triggered, neutral commands sent |
| Axes out of range | Clamped to [-1000, +1000] before packing |
| Ctrl+C / SIGTERM | Final neutral command sent before exit |
| Joystick reconnected | Automatic recovery, failsafe cleared |

---

## Visual Dashboard (GUI Mode)

Launch with `--gui` to open a real-time instrument dashboard built with
[Dear ImGui](https://github.com/ocornut/imgui) + SDL2 + OpenGL3.

```
┌── Title Bar ───────────────────────────────────────────────────┐
├── Left 18% ──┬───── Camera Viewport 52% ──────┬─ Right 30% ──┤
│  Stick L (○)  │  ┌─ Heading Tape ────────────┐ │ [Surge ◠]   │
│  Stick R (○)  │  │                            │ │ [Sway  ◠]   │
│  Buttons      │  │  Attitude    Depth  3D ROV │ │ [Heave ◠]   │
│  Status       │  │  Indicator   Gauge  Model  │ │ [Yaw   ◠]   │
│  Battery      │  │  ──── info strip ────────  │ │             │
│  Telemetry    │  └────────────────────────────┘ │             │
└──────────────┴────────────────────────────────┴─────────────┘
```

### Dashboard features

- **Central camera viewport** — live feed or animated ocean placeholder with HUD overlays
- **Attitude indicator** — artificial horizon with pitch ladder, roll arc, aircraft symbol
- **Heading tape** — horizontal scrolling compass with cardinal labels
- **Depth gauge** — vertical bar for heave / depth
- **3-D ROV model** — STL mesh rendered in real-time, rotates with IMU orientation
- **Arc gauges** — 270° sweep gauges for Surge, Sway, Heave, Yaw with severity colouring
- **Joystick sticks** — 2-D circular representations of left/right stick positions
- **Button grid** — 16-button panel, active buttons highlighted
- **Battery bar** — voltage, current, percentage with green/yellow/red colour coding
- **Telemetry** — depth, water temperature, internal temperature with threshold warnings
- **Status panel** — joystick connection, transport info, failsafe state, packet rate
- **Title bar** — NOMINAL/FAILSAFE indicator with coloured status light

Bright beach / sea colour theme with varied accent colours (turquoise, coral,
seafoam green, sunset orange, sandy gold, lavender).

---

## Configuration

All tuneable parameters live in `include/config.h`:

- Network: target IP, UDP port, serial device, baud rate
- Joystick: index, deadzone, axis mapping, inversion
- MAVLink: system/component IDs
- Timing: loop rate, heartbeat interval, failsafe timeout
- Safety: axis clamp bounds

---

## MAVLink Implementation

The file `include/mavlink_minimal.h` is a self-contained MAVLink v2
implementation that provides:

- Correct v2 wire format (STX `0xFD`, 3-byte message ID, CRC)
- X.25 CRC with per-message CRC_EXTRA seed
- HEARTBEAT and MANUAL_CONTROL pack + serialize functions

It is **drop-in compatible** with the official MAVLink C headers.  If you
later generate the full library from the MAVLink XML definitions, simply
replace this file and adjust the includes.

### Generating full MAVLink headers (optional)

```bash
pip install pymavlink
python -m pymavlink.tools.mavgen \
    --lang=C --wire-protocol=2.0 \
    --output=third_party/mavlink \
    mavlink/message_definitions/v1.0/common.xml
```

Then add `-Ithird_party` to CFLAGS and replace the include in
`mavlink_packer.c` with `#include <mavlink/common/mavlink.h>`.
