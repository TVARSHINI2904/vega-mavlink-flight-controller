# Vega-FC v2 — RISC-V Flight Controller Firmware

Bare-metal flight controller firmware for the **Vega Thejas (32-bit RISC-V)** processor, communicating via **MAVLink v2** protocol. Supports telemetry transmission, ARM/DISARM, flight mode switching (LOITER, AUTO, GUIDED, RTL), mission upload, waypoint navigation, and autonomous mission execution.

---

## Table of Contents
1. [Features](#features)
2. [Prerequisites](#prerequisites)
3. [Toolchain Installation](#toolchain-installation)
4. [Build Instructions](#build-instructions)
5. [Project Structure](#project-structure)
6. [Code Overview](#code-overview)
7. [Testing with QEMU + Mission Planner](#testing-with-qemu--mission-planner)
8. [Common Issues](#common-issues)
9. [Toolchain Details](#toolchain-details)

---

## Features

### MAVLink Communication
- Full MAVLink v2 protocol stack over UART (16550-compatible at 0x10000000)
- Bidirectional communication with Mission Planner / QGroundControl
- MAVLink v1/v2 frame auto-detection via `mavlink_parse_char()`

### Telemetry Transmission (7 periodic messages)
| Message | Period | Description |
|---------|--------|-------------|
| HEARTBEAT | 500 ms | Vehicle type (quadrotor), armed state, flight mode |
| ATTITUDE | 100 ms | Roll/pitch/yaw (simulated via incrementing counters) |
| VFR_HUD | 200 ms | Airspeed, groundspeed, heading, throttle, altitude |
| SYS_STATUS | 1000 ms | Battery voltage, load, sensor health |
| GPS_RAW_INT | 1000 ms | GPS position, fix type, satellites visible |
| GLOBAL_POSITION_INT | 1000 ms | Global position with heading |
| MISSION_CURRENT | On waypoint change | Current active waypoint sequence number |

### Command Reception & Handling
- **ARM / DISARM** — `MAV_CMD_COMPONENT_ARM_DISARM` with STATUSTEXT confirmation
- **Flight Mode Change** — STABILIZE (0), AUTO (3), GUIDED (4), LOITER (5), RTL (6)
- **Set Home Position** — Via `MAV_CMD_DO_SET_HOME` or `SET_HOME_POSITION` message
- **Parameter System** — Read/write/list 5 parameters (SYSID_THISMAV, SYSID_MYGCS, ARMING_CHECK, FRAME_CLASS, SERIAL0_BAUD)
- **Mission Upload** — Full mission protocol: MISSION_COUNT → MISSION_ITEM_INT → MISSION_ACK (up to 20 waypoints)
- **Mission Download** — Responds to MISSION_REQUEST_LIST, MISSION_REQUEST_INT, MISSION_REQUEST

### Autonomous Mission Execution
- **AUTO mode**: Vehicle navigates through uploaded waypoints sequentially, updating simulated GPS position toward each target
- **GUIDED mode**: Same waypoint-following behavior as AUTO
- **RTL mode**: Vehicle navigates back to the home position autonomously
- **LOITER mode**: Placeholder for future implementation
- **Waypoint arrival detection**: Position threshold (300 units) and altitude threshold (500 mm)
- **Progress reporting**: Sends MISSION_ITEM_REACHED, MISSION_CURRENT, and MISSION_COMPLETE STATUSTEXT
- **Heading computation**: Direction calculated from delta lat/lon between current position and target

### Simulated Vehicle Movement
- Position updated in 500-unit steps toward each waypoint
- Altitude adjusted in 100 mm steps toward target altitude
- Attitude (roll/pitch) simulated via incrementing counters
- GPS coordinates start at Chennai (13.0827°N, 80.2707°E) by default

### Bare-Metal Architecture
- **No RTOS** — cooperative scheduler with 1ms timer tick
- **No libc** — custom `memcpy`, `memset`, `memcmp`
- **No FPU** — all float operations emulated via IEEE754 integer bit manipulation
- **No heap** — all memory statically allocated
- **Minimal footprint** — ~16KB total (text + data + bss)

---

## Prerequisites

- **Windows PC** (the firmware toolchain — `riscv64-vega-elf-gcc` — is Windows-native and is built and run from PowerShell)
- **Git** (to clone the SDK and this project)
- **Git Bash or WSL** (only for the one-time SDK setup script in [Toolchain Installation](#toolchain-installation) — everything after that uses PowerShell)
- **PowerShell** (for all building and running — do not use WSL for `make` or QEMU steps)

---

## Toolchain Installation

### Step 1: Clone the Vega SDK
```powershell
git clone https://gitlab.com/cdac-vega/vega-sdk.git
cd vega-sdk
```

### Step 2: Checkout the Aries branch
```powershell
git checkout aries
```

### Step 3: Run the setup script

`setup.sh` is a bash script — run it from **Git Bash or WSL**, not plain PowerShell:

```bash
./setup.sh
```

This installs the **riscv64-vega-elf-gcc** toolchain to `C:\Users\<username>\vega-tools-windows\bin\`.

> After this step, switch back to PowerShell for everything else.

### Step 4: Install `make` for Windows

`make` is included in the toolchain folder after Step 3. If setting up on a fresh machine and it's missing, install it manually:

```powershell
# Download ezwinports make
python -c "import urllib.request; urllib.request.urlretrieve('https://sourceforge.net/projects/ezwinports/files/make-4.4.1-without-guile-w32-bin.zip/download', 'make.zip')"

# Extract
Expand-Archive -Path make.zip -DestinationPath make_install -Force
Copy-Item make_install/bin/make.exe "$env:USERPROFILE\vega-tools-windows\bin\" -Force
```

### Step 5: Clone this project
```powershell
cd $env:USERPROFILE
git clone https://github.com/<your-org>/vega-fc-v2.git
cd vega-fc-v2
```

---

## Build Instructions

### From PowerShell (recommended):
```powershell
# Add toolchain to PATH (one-time per session)
$env:Path += ";$env:USERPROFILE\vega-tools-windows\bin"

# Clean and build
cd $env:USERPROFILE\vega-fc-v2
make clean
make all
```

### Makefile commands:
| Command | Action |
|---------|--------|
| `make` or `make all` | Compile all source files and link → `vega-fc-v2.elf` |
| `make clean` | Delete all object files and build artifacts |
| `make bin` | Generate raw binary `vega-fc-v2.bin` (for flashing) |

### Expected output:
```
C:/Users/<username>/vega-tools-windows/bin/riscv64-vega-elf-size vega-fc-v2.elf
   text    data     bss     dec     hex filename
  13733     220    2078   16031    3e9f vega-fc-v2.elf
```

**Note:** Only warnings from MAVLink headers will appear (harmless `-Waddress-of-packed-member`). Zero errors expected.

---

## Project Structure

| File | Purpose |
|------|---------|
| `start.s` | RISC-V assembly startup: stack init, trap vector, timer enable, calls `main()` |
| `link.ld` | Linker script: `.text` at 0x80000000, `.data`/`.bss`, stack at BSS+0x1000 |
| `main.c` | Entry point: `memcpy/memset/memcmp`, timer ISR, main loop (WFI → scheduler → MAVLink RX) |
| `uart.c` / `uart.h` | 16550-compatible UART driver at MMIO address 0x10000000 |
| `scheduler.c` / `scheduler.h` | Cooperative scheduler: 7 periodic tasks driven by 1ms `sys_tick` |
| `mavlink_tx.c` / `mavlink_tx.h` | MAVLink telemetry transmitter + parameter system + mission upload responses |
| `mavlink_rx.c` / `mavlink_rx.h` | MAVLink command receiver: ARM/DISARM, mode change, param ops, mission upload |
| `mission.c` / `mission.h` | Waypoint storage (up to 20), autonomous navigation, RTL, position simulation |
| `c_library_v2/` | Auto-generated MAVLink v2 C library headers |

---

## Code Overview

### Main Loop (`main.c`)
```
Wait For Interrupt (WFI)
    ↓
scheduler_run() — runs 7 periodic tasks
    ↓
mavlink_rx_poll() — processes incoming commands
    ↓
(repeat)
```

### Scheduler (`scheduler.c`) — Tasks:
| Task | Period | Description |
|------|--------|-------------|
| `send_heartbeat` | 500 ms | Vehicle type, armed state, flight mode |
| `send_attitude` | 100 ms | Simulated roll/pitch/yaw |
| `mission_update` | 100 ms | Waypoint navigation, position simulation, RTL |
| `send_vfr_hud` | 200 ms | Airspeed, altitude, heading, throttle |
| `send_sys_status` | 1000 ms | Battery, load, sensor status |
| `send_gps_raw_int` | 1000 ms | GPS position, fix type, satellites |
| `send_global_position_int` | 1000 ms | Global position with heading |

### Supported MAVLink Commands (RX):
- ARM / DISARM
- Flight mode change (STABILIZE, AUTO, GUIDED, LOITER, RTL)
- Set home position (via COMMAND_LONG or SET_HOME_POSITION message)
- Parameter read / write / list
- Mission upload (MISSION_COUNT → MISSION_ITEM_INT → MISSION_ACK)
- Mission download (MISSION_REQUEST_LIST, MISSION_REQUEST_INT, MISSION_REQUEST)
- REQUEST_MESSAGE (HOME_POSITION)

### Autonomous Mission Execution Flow:
1. GCS sends `MISSION_COUNT` → firmware stores count, resets mission state
2. Firmware requests each waypoint via `MISSION_REQUEST_INT`
3. GCS sends `MISSION_ITEM_INT` for each waypoint → stored in `mission[]` array
4. On last waypoint, firmware sends `MISSION_ACK` (accepted)
5. When flight mode is set to AUTO (3) or GUIDED (4), `mission_update()` begins:
   - Computes heading toward next waypoint
   - Steps position by 500 units/tick toward target
   - Steps altitude by 100 mm/tick toward target altitude
   - On arrival (within threshold), sends `MISSION_ITEM_REACHED`
   - Advances to next waypoint, sends `MISSION_CURRENT`
   - On final waypoint, sends "MISSION COMPLETE" STATUSTEXT
6. In RTL mode (6), vehicle navigates back to home position autonomously

### Key Architecture Notes:
- **No RTOS** — bare-metal cooperative scheduling
- **No libc** — custom `memcpy`, `memset`, `memcmp`
- **No FPU** — all float operations emulated via integer bit manipulation (IEEE754 union trick)
- **Simulated sensors** — attitude (incrementing counters), position (step-based toward waypoints), altitude (step-based toward target)
- **Soft-float param decode** — `float_bits_to_scaled_i32()` manually extracts sign/exponent/mantissa from IEEE754 bits for parsing MAVLink float params
- **No dynamic memory** — all buffers and waypoint storage are statically allocated

---

## Testing with QEMU + Mission Planner

### What it does
The firmware can run on a **RISC-V QEMU emulator** and communicate with **Mission Planner** (or any GCS) via MAVLink over TCP.

The Vega toolchain compiles the firmware into an `.elf` file. QEMU runs the ELF as a bare-metal RISC-V guest, and the UART output is tunneled over TCP. Mission Planner connects to that TCP port and communicates with the firmware just like a real flight controller.

### Prerequisites
- **QEMU for RISC-V** installed (e.g., `qemu-system-riscv32`)
- **Mission Planner** or any MAVLink GCS (e.g., QGroundControl)
- The built `vega-fc-v2.elf` file

### From Windows PowerShell:
```powershell
# Start QEMU (this will block the terminal)
& "C:\Program Files\qemu\qemu-system-riscv32.exe" `
  -machine virt `
  -nographic `
  -serial tcp::5760,server,nowait `
  -kernel vega-fc-v2.elf
```

### From WSL / Linux:
```bash
qemu-system-riscv32 \
  -machine virt \
  -nographic \
  -serial tcp::5760,server,nowait \
  -kernel /mnt/c/Users/<username>/vega-fc-v2/vega-fc-v2.elf
```

### Connect Mission Planner:
1. In Mission Planner, go to **Connect** (top right)
2. Select **TCP** connection type
3. Set **Port**: `5760`
4. Click **Connect**
5. You should see simulated telemetry data (heartbeat, attitude, GPS, etc.)

### What you can do:
| Action | How |
|--------|-----|
| See telemetry | HEARTBEAT, ATTITUDE, VFR_HUD, GPS, SYS_STATUS |
| ARM / DISARM | Click ARM/DISARM button or send via COMMAND_LONG |
| Change flight mode | Select mode from dropdown (Stabilize, Auto, Guided, Loiter, RTL) |
| Set home position | Use "Set Home Here" or specify coordinates |
| Upload waypoints | Plan a mission in Mission Planner and upload (up to 20 waypoints) |
| Watch autonomous flight | Switch to AUTO mode — vehicle will navigate waypoints step by step |
| Test RTL | Switch to RTL mode — vehicle will return to home position |
| Read/write parameters | Use Mission Planner's parameter tree |

### Supported MAVLink messages visible in Mission Planner:
| Message | What you see |
|---------|-------------|
| HEARTBEAT | Vehicle type (quadrotor), armed state, flight mode |
| ATTITUDE | Roll/pitch/yaw (simulated) |
| VFR_HUD | Airspeed, altitude, heading, throttle |
| GPS_RAW_INT | GPS position (default: Chennai) |
| GLOBAL_POSITION_INT | Same position as GPS_RAW_INT with heading |
| SYS_STATUS | Battery voltage, load, sensor status |
| MISSION_CURRENT | Active waypoint during mission execution |
| MISSION_ITEM_REACHED | Waypoint arrival notification |
| STATUSTEXT | "ARMED", "DISARMED", "MISSION START", "WAYPOINT REACHED", "MISSION COMPLETE", "RTL START", "RTL COMPLETE" |

---

## Common Issues

### 1. `make` not found
```powershell
# Add toolchain bin to PATH and try again:
$env:Path += ";$env:USERPROFILE\vega-tools-windows\bin"
```

### 2. "No such file or directory" for compiler
**Cause**: Running from WSL instead of PowerShell. The Vega toolchain is a Windows `.exe` — it only works in PowerShell.

**Fix**: Use PowerShell, not WSL/Ubuntu, for `make` and QEMU steps.

### 3. Build errors: "too few arguments"
**Cause**: The MAVLink library was updated and a message-pack function gained a new field, so older call sites are missing an argument.

**Fix**: Add the missing argument at the call site. For example:
```c
// Before (older c_library_v2):
mavlink_msg_sys_status_pack(system_id, component_id, &msg,
    sensors_present, sensors_enabled, sensors_health, load,
    voltage_battery, current_battery, battery_remaining,
    drop_rate_comm, errors_comm, errors_count1, errors_count2,
    errors_count3, errors_count4);

// After (newer c_library_v2 — added onboard_control_sensors_present_extended etc.):
mavlink_msg_sys_status_pack(system_id, component_id, &msg,
    sensors_present, sensors_enabled, sensors_health, load,
    voltage_battery, current_battery, battery_remaining,
    drop_rate_comm, errors_comm, errors_count1, errors_count2,
    errors_count3, errors_count4,
    0, 0, 0); // new trailing fields — pass 0 if unused
```
The 3 known call sites already fixed this way are `send_sys_status`, `send_gps_raw_int`, and `send_mission_ack` in `mavlink_tx.c`. If a future library update breaks a 4th call site the same way, diff the old vs. new function signature in `c_library_v2/common/mavlink_msg_*.h` and pad the new trailing parameters with `0`.

### 4. Linker errors: "undefined reference to `__ashldi3'"
**Cause**: RV32 doesn't have 64-bit shift instructions — needs libgcc.

**Fix**: Already fixed in Makefile — `-lgcc` is placed after object files.

### 5. No telemetry visible in Mission Planner
**Cause**: QEMU not running, wrong TCP port, or firewall blocking.

**Fix**: Ensure QEMU is running and listening on port 5760. Check with `netstat -an | findstr 5760`.

---

## Toolchain Details

- **SDK**: `https://gitlab.com/cdac-vega/vega-sdk.git` (branch: `aries`)
- **Toolchain**: `riscv64-vega-elf-gcc` (GCC 10.1.0, custom Vega target)
- **Architecture**: `rv32im` (32-bit RISC-V with Integer + Multiply extensions)
- **ABI**: `ilp32` (32-bit soft-float ABI)
- **Type**: Cross-compiler for bare-metal (no OS)

---

## License

_TODO: add license before sharing outside the team._