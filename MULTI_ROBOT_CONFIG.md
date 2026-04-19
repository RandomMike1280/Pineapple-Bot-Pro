# Multi-Robot Configuration Guide

This document explains the layered configuration system that lets the same firmware run on two different robots with minimal overhead.

## Quick Start

To switch between robots, edit `src/main.h` and change which config file is included:

```cpp
// main.h — pick one, comment out the other:
#include "config_a.h"   // Robot A
// #include "config_b.h" // Robot B (uncomment to switch)
```

Everything else (WiFi, calibration, motor parameters) comes from the selected config.

---

## How It Works

The system uses a **three-layer config hierarchy**:

```
config_defaults.h   ← safe baseline values for every constant
config_a.h         ← includes defaults, then overrides Robot A values
config_b.h         ← includes defaults, then overrides Robot B values
main.h             ← includes the selected robot config, then declares API
main.cpp           ← firmware logic
```

### Layer 1: `config_defaults.h`

Contains every single constant in the codebase with a safe baseline value. It uses `#ifndef` / `#define` pattern so any constant can be overridden by a later-included file:

```cpp
#ifndef DISTANCE_FACTOR_V
#define DISTANCE_FACTOR_V  1.0f   // default
#endif
```

### Layer 2: `config_a.h` / `config_b.h`

Each robot config includes `config_defaults.h` first, then `#define`s only the values that differ for that robot:

```cpp
// config_a.h
#define BOT_ID           "A"
#define DISTANCE_FACTOR_H  1.25f   // Robot A needs different strafe calibration
// all other constants use the defaults automatically
```

### Layer 3: `main.h`

A thin header that includes the selected robot config, then declares all the shared API (function prototypes, structs). No constants live here anymore.

---

## What to Calibrate Per Robot

### 1. Distance Factors (`DISTANCE_FACTOR_V`, `DISTANCE_FACTOR_H`)

Dead-reckoning calibration — corrects the gap between commanded and actual distance.

**Calibration method:**
1. Command a **1000 mm forward move** from the phone app.
2. Measure the actual distance traveled with a ruler.
3. Calculate: `V factor = 1000.0f / measured_distance`
4. Repeat for **pure strafe** to calibrate the H factor.

Example: if Robot A travels 920 mm when commanded to go 1000 mm:
```
DISTANCE_FACTOR_V = 1000.0f / 920.0f ≈ 1.087f
```

> The existing value of `1.25f` for Robot A's H factor means its strafe distance is 80% of commanded — if it's consistently under- or over-shooting strafe, adjust this.

### 2. Drift Correction (`DRIFT_TRIM_*_DEG`)

Mecanum wheels on flat surfaces tend to pull to one side. This trim rotates the heading frame slightly to counteract that bias.

**Calibration method:**
1. Command **pure forward motion** at a fixed speed.
2. Observe how far the robot drifts laterally over a few meters.
3. Increase `DRIFT_TRIM_*_DEG` until the robot goes straight.

The trim is speed-dependent — calibrate at each speed level:
- `DRIFT_TRIM_SLOW_DEG` — for slow speed moves
- `DRIFT_TRIM_NORMAL_DEG` — for normal speed moves
- `DRIFT_TRIM_FAST_DEG` — for fast speed moves

### 3. WiFi Credentials (`ROBOT_WIFI_SSID`, `ROBOT_WIFI_PASS`)

Set these to the hotspot SSID and password for each robot. The robot will connect to its own hotspot automatically.

### 4. Speed Scaling (optional)

`SPEED_SCALING_FACTOR` and `ROTATION_SCALING_FACTOR` are global multipliers that shrink or grow all speed profiles proportionally. Use these instead of editing the raw speed constants directly.

---

## Adding a New Robot (Robot C)

1. Copy `config_b.h` to `config_c.h`.
2. Set `BOT_ID`, WiFi credentials, and calibrate the values.
3. In `main.h`, change `#include "config_a.h"` to `#include "config_c.h"`.

---

## Constants Reference

For full details on every constant (what it does, how to tune it), see the detailed comments in `src/config_defaults.h`. Each constant group has a calibration guide at the top of its section.
