# Vega-FC v2 — RISC-V Flight Controller Firmware

Bare-metal flight controller firmware for the **Vega Thejas (32-bit RISC-V)** processor, communicating via **MAVLink v2** protocol. Supports telemetry transmission, ARM/DISARM, flight mode switching (STABILIZE, AUTO, GUIDED, LOITER, RTL), mission upload/download/clear, waypoint navigation with proportional control, autonomous mission execution, GUIDED click-to-fly, battery simulation, failsafe logic, mission persistence, and configurable parameters.

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

### Telemetry Transmission (10 periodic tasks)
| Task | Period | Description |
|------|--------|-------------|
| HEARTBEAT | 500 ms | Vehicle type (quadrotor), armed state, flight mode |
| ATTITUDE | 100 ms | Roll/pitch/yaw — heading now reported accurately from `current_heading` |
| mission_update | 100 ms | Waypoint navigation, position simulation, RTL, GUIDED click-to-fly |
| sim_battery_update | 100 ms | Battery drain simulation, GPS satellite simulation |
| failsafe_check | 100 ms | Low battery → RTL, GPS loss → LOITER |
| VFR_HUD | 200 ms | Airspeed, groundspeed, heading, throttle, altitude |
| NAV_CONTROLLER_OUTPUT | 200 ms | Distance-to-waypoint (meters), target bearing, heading for GCS display |
| SYS_STATUS | 1000 ms | Battery voltage, current, remaining % (dynamic) |
| GPS_RAW_INT | 1000 ms | GPS position, fix type, satellites visible |
| GLOBAL_POSITION_INT | 1000 ms | Global position with heading |

### Command Reception & Handling
- **ARM / DISARM** — `MAV_CMD_COMPONENT_ARM_DISARM` with STATUSTEXT confirmation
- **Flight Mode Change** — STABILIZE (0), AUTO (3), GUIDED (4), LOITER (5), RTL (6)
- **Set Home Position** — Via `MAV_CMD_DO_SET_HOME` or `SET_HOME_POSITION` message
- **Parameter System** — Read/write/list 6 parameters including the new `WP_RADIUS`
- **Mission Upload** — Full mission protocol: MISSION_COUNT → MISSION_ITEM_INT → MISSION_ACK (up to 20 waypoints)
- **Mission Download** — Responds to MISSION_REQUEST_LIST, MISSION_REQUEST_INT, MISSION_REQUEST
- **Mission Clear** — New `MISSION_CLEAR_ALL` handler to reset mission from GCS

### Autonomous Mission Execution
- **AUTO mode**: Vehicle navigates through uploaded waypoints sequentially using smooth **bearing-based proportional control**
- **GUIDED mode**: Supports **click-to-fly** via `SET_POSITION_TARGET_GLOBAL_INT` — right-click the map in GCS and drone flies there
- **RTL mode**: Vehicle navigates back to the home position autonomously
- **LOITER mode**: Placeholder for future implementation
- **Smooth diagonal flight**: Lat/lon movement is distributed proportionally based on bearing — no more stair-stepping
- **Proportional deceleration**: Step size reduces as drone approaches target (30% of remaining distance), preventing overshoot
- **Configurable acceptance radius**: `WP_RADIUS` parameter (default 33m, range 3-500m) tunable from GCS without recompiling
- **Waypoint arrival detection**: Position threshold calculated from `WP_RADIUS` parameter, altitude threshold (500 mm)
- **Progress reporting**: Sends MISSION_ITEM_REACHED, MISSION_CURRENT, NAV_CONTROLLER_OUTPUT, and MISSION_COMPLETE STATUSTEXT
- **Heading computation**: Accurate bearing calculation via `approx_heading_centideg()` — ratio-based quadrant mapping, no FPU needed
- **Waypoint action reporting**: Reports waypoint type on arrival (NAV, LAND, RTL, LOITER_TIME, DO_CHANGE_SPEED)

### Battery Simulation
- **Dynamic discharge**: Battery drains at 1% per 100ms when armed, 2% per 100ms when moving (AUTO/GUIDED/RTL)
- **Trickle charge**: Battery recharges at 1% per 100ms when disarmed
- **Voltage modeling**: Voltage scales linearly from 12.6V (100%) to 10.8V (0%)
- **Current reporting**: Current draw proportional to drain rate
- **Real-time reporting**: SYS_STATUS message uses live battery_voltage_mv, battery_current_ma, and battery_remaining_pct

### Failsafe Logic
- **Low Battery → RTL**: When battery drops to 15% or below while armed, automatically switches to RTL mode. Sends critical STATUSTEXT "FS:BAT LOW → RTL". Clears when battery recovers above 25% or RTL completes.
- **GPS Loss → LOITER**: When GPS fix drops below 3D for 5 continuous seconds while armed, automatically switches to LOITER mode. Sends critical STATUSTEXT "FS:GPS LOST→LOITER". Clears when 3D fix is restored.
- **Priority**: Battery failsafe takes priority over GPS failsafe (RTL > LOITER)

### Mission Persistence
- **Auto-save on upload**: Mission is automatically saved to simulated NVM when upload completes (both MISSION_ITEM_INT and MISSION_ITEM protocols)
- **Auto-save on completion**: Mission is saved to NVM when all waypoints are reached
- **Restore on boot**: On startup, firmware checks NVM for a valid mission signature (0xA5A5) and restores it automatically
- **Simulated NVM**: 1KB reserved in linker script (`.nvm` section) for non-volatile storage, accessed via `_nvm_start`/`_nvm_end` linker symbols
- **STATUSTEXT feedback**: "MISSION SAVED", "MISSION RESTORED", "NVM RESTORED", "MISSION CLEARED" confirm persistence operations

### Simulated Vehicle Movement
- Position updated with bearing-based proportional control toward each waypoint
- Altitude adjusted in 100 mm steps toward target altitude
- Attitude (roll/pitch) simulated via incrementing counters, yaw from actual heading
- GPS coordinates start at Chennai (13.0827°N, 80.2707°E) by default
- GPS satellite count fluctuates (simulated RF environment)

### Bare-Metal Architecture
- **No RTOS** — cooperative scheduler with 1ms timer tick
- **No libc** — custom `memcpy`, `memset`, `memcmp`
- **No FPU** — all float operations emulated via IEEE754 integer bit manipulation + lookup tables
- **No heap** — all memory statically allocated
- **Minimal footprint** — ~25KB total (text + data + bss)

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
  22193     288    3112   25593    63f9 vega-fc-v2.elf
```

**Note:** Only warnings from MAVLink headers will appear (harmless `-Waddress-of-packed-member`). Zero errors expected.

---

## Project Structure

| File | Purpose |
|------|---------|
| `start.s` | RISC-V assembly startup: stack init, trap vector, timer enable, calls `main()` |
| `link.ld` | Linker script: `.text` at 0x80000000, `.data`/`.bss`, `.nvm` (1KB simulated NVM), stack at BSS+0x1000 |
| `main.c` | Entry point: `memcpy/memset/memcmp`, timer ISR, main loop (WFI → scheduler → MAVLink RX), NVM restore on boot |
| `uart.c` / `uart.h` | 16550-compatible UART driver at MMIO address 0x10000000 |
| `scheduler.c` / `scheduler.h` | Cooperative scheduler: 10 periodic tasks driven by 1ms `sys_tick` |
| `mavlink_tx.c` / `mavlink_tx.h` | MAVLink telemetry transmitter + parameter system (including WP_RADIUS) + mission upload/download responses + NAV_CONTROLLER_OUTPUT |
| `mavlink_rx.c` / `mavlink_rx.h` | MAVLink command receiver: ARM/DISARM, mode change, param ops, mission upload/clear, GUIDED position target, NVM save on upload |
| `mission.c` / `mission.h` | Waypoint storage (up to 20), autonomous navigation with proportional control, RTL, GUIDED click-to-fly, position simulation, battery simulation, failsafe logic, NVM persistence, WP_RADIUS config |
| `c_library_v2/` | Auto-generated MAVLink v2 C library headers |

---

## Code Overview

### Main Loop (`main.c`)
```
Wait For Interrupt (WFI)
    ↓
scheduler_run() — runs 10 periodic tasks
    ↓
mavlink_rx_poll() — processes incoming commands
    ↓
(repeat)
```

### Scheduler (`scheduler.c`) — Tasks:
| Task | Period | Description |
|------|--------|-------------|
| `send_heartbeat` | 500 ms | Vehicle type, armed state, flight mode |
| `send_attitude` | 100 ms | Simulated roll/pitch, actual heading |
| `mission_update` | 100 ms | Waypoint navigation, GUIDED click-to-fly, RTL |
| `sim_battery_update` | 100 ms | Battery drain, GPS satellite simulation |
| `failsafe_check` | 100 ms | Low battery → RTL, GPS loss → LOITER |
| `send_vfr_hud` | 200 ms | Airspeed, altitude, heading, throttle |
| `send_nav_controller_output` | 200 ms | Distance-to-waypoint, target bearing, heading |
| `send_sys_status` | 1000 ms | Battery voltage, current, remaining % (dynamic) |
| `send_gps_raw_int` | 1000 ms | GPS position, fix type, satellites |
| `send_global_position_int` | 1000 ms | Global position with heading |

### Supported MAVLink Messages (RX):
| Message ID | Handler | Description |
|------------|---------|-------------|
| COMMAND_LONG (76) | `handle_command_long` | ARM/DISARM, mode change, set home, parameter ops |
| SET_MODE (11) | `handle_set_mode` | Flight mode change |
| SET_HOME_POSITION (242) | `handle_set_home_position` | Set home via message |
| PARAM_REQUEST_LIST (21) | `handle_param_request_list` | List all parameters (including WP_RADIUS) |
| PARAM_REQUEST_READ (20) | `handle_param_request_read` | Read specific parameter |
| PARAM_SET (23) | `handle_param_set` | Write parameter value |
| MISSION_COUNT (44) | `handle_mission_count` | Start mission upload |
| MISSION_ITEM_INT (73) | `handle_mission_item_int` | Receive waypoint data |
| MISSION_ITEM (39) | `handle_mission_item` | Receive float-latlon waypoint data |
| MISSION_CLEAR_ALL (45) | `handle_mission_clear_all` | **NEW** Clear entire mission |
| SET_POSITION_TARGET_GLOBAL_INT (86) | `handle_set_position_target_global_int` | **NEW** GUIDED click-to-fly |
| MISSION_REQUEST (40) | `handle_mission_request` | Upload request (float) |
| MISSION_REQUEST_INT (51) | `handle_mission_request_int` | Upload request (int) |
| MISSION_REQUEST_LIST (43) | `handle_mission_request_list` | Download mission list |

### Autonomous Mission Execution Flow:
1. GCS sends `MISSION_COUNT` → firmware stores count, resets mission state
2. Firmware requests each waypoint via `MISSION_REQUEST_INT`
3. GCS sends `MISSION_ITEM_INT` for each waypoint → stored in `mission[]` array
4. On last waypoint, firmware sends `MISSION_ACK` (accepted) and saves mission to NVM
5. When flight mode is set to AUTO (3), `mission_update()` begins:
   - Computes heading toward next waypoint using `approx_heading_centideg()`
   - Moves position using **bearing-based proportional control** — lat/lon distributed proportionally
   - Proportional step = 30% of remaining distance, clamped to max 500 units/tick
   - On arrival (within `WP_RADIUS` meters), sends `MISSION_ITEM_REACHED`
   - Advances to next waypoint, sends `MISSION_CURRENT`
   - On final waypoint, saves mission to NVM and sends "MISSION COMPLETE"
6. **GUIDED mode click-to-fly**: Right-click map → GCS sends `SET_POSITION_TARGET_GLOBAL_INT`
   - Firmware stores target, navigates toward it
   - Reports "GUIDED TARGET SET" and "GUIDED TARGET REACHED"
7. In RTL mode (6), vehicle navigates back to home position autonomously
8. **MISSION_CLEAR_ALL**: Clears waypoints, resets mission state, acknowledges GCS

### Battery Simulation Flow:
1. `sim_battery_update()` runs every 100ms
2. When armed: battery drains 1%/tick (idle) or 2%/tick (moving)
3. Voltage calculated linearly: 12.6V @ 100% → 10.8V @ 0%
4. When disarmed: battery trickle-charges at 1%/tick
5. GPS satellite count fluctuates randomly for realistic simulation
6. `send_sys_status()` reads live `battery_voltage_mv`, `battery_current_ma`, `battery_remaining_pct`

### Failsafe Logic Flow:
1. `failsafe_check()` runs every 100ms
2. **Low Battery**: If battery ≤ 15% and armed → force RTL mode, send critical STATUSTEXT
3. **GPS Loss**: If GPS fix < 3D for 5 seconds and armed → force LOITER mode, send critical STATUSTEXT
4. Battery failsafe takes priority over GPS failsafe
5. Failsafe clears on recovery (battery > 25% or GPS fix restored)

### Navigation Features Detail:
- **`move_toward_2d()`** — Bearing-based proportional 2D movement: distributes step between lat/lon proportionally based on distance to target. Enables diagonal flight paths instead of stair-stepping.
- **`move_toward()`** — 1D proportional movement for altitude: step = 30% of remaining distance.
- **`approx_heading_centideg()`** — Integer-only bearing calculation. Uses ratio of smaller/larger axis × 45° mapped to correct quadrant. No FPU needed.
- **`wp_threshold_from_radius()`** — Converts `WP_RADIUS` (meters) to degE7 threshold. Default 33m ≈ 297 degE7.

### Mission Persistence Flow:
1. On mission upload complete → `save_mission_to_nvm()` writes waypoints to `.nvm` section
2. On mission complete → `save_mission_to_nvm()` writes waypoints to `.nvm` section
3. On mission clear → `handle_mission_clear_all()` clears NVM state in memory
4. On boot → `load_mission_from_nvm()` checks for valid signature (0xA5A5) and restores mission

### Configurable Parameters:
| Parameter | Default | Description |
|-----------|---------|-------------|
| SYSID_THISMAV | 1.0 | Vehicle system ID |
| SYSID_MYGCS | 255.0 | GCS system ID |
| ARMING_CHECK | 0.0 | Arming checks (0 = disabled) |
| FRAME_CLASS | 1.0 | Frame class (1 = quadrotor) |
| SERIAL0_BAUD | 115.0 | Serial baud rate |
| **WP_RADIUS** | **33.0** | **NEW** Waypoint acceptance radius in meters (3-500) |

### Key Architecture Notes:
- **No RTOS** — bare-metal cooperative scheduling
- **No libc** — custom `memcpy`, `memset`, `memcmp`
- **No FPU** — all float operations emulated via integer bit manipulation (IEEE754 union trick) and lookup tables
- **Simulated sensors** — attitude (incrementing counters), position (bearing-based toward waypoints), altitude (step-based toward target), battery (dynamic drain model), GPS (fluctuating satellite count)
- **Soft-float param decode** — `float_bits_to_scaled_i32()` manually extracts sign/exponent/mantissa from IEEE754 bits for parsing MAVLink float params
- **Heading lookup table** — `heading_deg_to_rad_bits()` converts degrees to IEEE754 radian float bits for ATTITUDE message
- **Distance lookup table** — `int_to_float_bits()` converts integer meters to IEEE754 float bits for NAV_CONTROLLER_OUTPUT
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
| See telemetry | HEARTBEAT, ATTITUDE (with heading), VFR_HUD, GPS, SYS_STATUS (with live battery), NAV_CONTROLLER_OUTPUT (distance to WP) |
| ARM / DISARM | Click ARM/DISARM button or send via COMMAND_LONG |
| Change flight mode | Select mode from dropdown (Stabilize, Auto, Guided, Loiter, RTL) |
| Set home position | Use "Set Home Here" or specify coordinates |
| Upload waypoints | Plan a mission in Mission Planner and upload (up to 20 waypoints) |
| Watch autonomous flight | Switch to AUTO mode — vehicle navigates waypoints smoothly with diagonal flight |
| See distance to waypoint | NAV_CONTROLLER_OUTPUT shows meters remaining to current waypoint |
| See heading change | ATTITUDE yaw now updates accurately with drone heading |
| Test GUIDED click-to-fly | Switch to GUIDED → right-click map → "Fly to Here" → drone flies there |
| Clear mission | Right-click → "Clear All" → MISSION CLEARED confirmation |
| Tune WP_RADIUS | Go to Config → Parameter Tree → change WP_RADIUS value (3-500m) |
| Test RTL | Switch to RTL mode — vehicle returns to home position smoothly |
| Test battery failsafe | ARM and wait — battery drains to 15%, triggers automatic RTL |
| Test GPS failsafe | Watch satellite count fluctuate — if it drops below 4 for 5s, triggers LOITER |
| Test mission persistence | Upload mission, reboot QEMU, mission is restored from NVM |
| Read/write parameters | Use Mission Planner's parameter tree (6 parameters including WP_RADIUS) |

### Supported MAVLink messages visible in Mission Planner:
| Message | What you see |
|---------|-------------|
| HEARTBEAT | Vehicle type (quadrotor), armed state, flight mode |
| ATTITUDE | Roll/pitch/yaw — yaw now reflects actual heading to waypoint |
| VFR_HUD | Airspeed, altitude, heading, throttle |
| GPS_RAW_INT | GPS position (default: Chennai), fix type, satellites |
| GLOBAL_POSITION_INT | Same position as GPS_RAW_INT with heading |
| SYS_STATUS | Battery voltage (draining), current, remaining % (dynamic) |
| NAV_CONTROLLER_OUTPUT | **NEW** Distance to waypoint (meters), target bearing |
| MISSION_CURRENT | Active waypoint during mission execution |
| MISSION_ITEM_REACHED | Waypoint arrival notification |
| STATUSTEXT | "ARMED", "DISARMED", "MISSION START", "WP NAV/ LAND/ RTL", "MISSION COMPLETE", "RTL START", "RTL COMPLETE", "FS:BAT LOW → RTL", "FS:GPS LOST→LOITER", "MISSION SAVED", "MISSION RESTORED", "GUIDED TARGET SET", "GUIDED TARGET REACHED", "MISSION CLEARED" |

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

### 6. Drone not flying / Failsafe triggers immediately
**Cause**: Old firmware running on board. The code changes need to be **re-flashed** to take effect.

**Fix**: Run `make clean && make`, then flash the new `vega-fc-v2.elf` to the board.

### 7. Yaw still shows 0° in ATTITUDE
**Cause**: Old firmware still running. The `heading_deg_to_rad_bits()` fix requires re-flashing.

**Fix**: Rebuild and re-flash with `make clean && make`.

### 8. GUIDED mode doesn't fly
**Cause**: GUIDED mode requires clicking on the map to set a target. Just switching to GUIDED mode doesn't make the drone move. Also ensure you've re-flashed with the new firmware.

**Fix**: In GUIDED mode, right-click the map → "Fly to Here" or "Set GUIDED Target". Ensure `SET_POSITION_TARGET_GLOBAL_INT` is being sent (visible in GCS MAVLink Inspector).

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