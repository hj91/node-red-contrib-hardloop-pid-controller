# node-red-contrib-hardloop-pid-controller

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![npm version](https://img.shields.io/npm/v/node-red-contrib-hardloop-pid-controller)](https://www.npmjs.com/package/node-red-contrib-hardloop-pid-controller)
[![Node-RED](https://img.shields.io/badge/Node--RED-%3E%3D2.0.0-red)](https://nodered.org)
[![simple-pid-controller](https://img.shields.io/badge/simple--pid--controller-v2.0.0-green)](https://www.npmjs.com/package/simple-pid-controller)

A hard-loop PID controller node for Node-RED on **soft-PLC platforms** where Node-RED is the control runtime — not a supervisory overlay.

Built on [`simple-pid-controller` v2.0.1](https://www.npmjs.com/package/simple-pid-controller).

**Author:** Harshad Joshi @ Bufferstack.IO Analytics Technology LLP, Pune  
**License:** Apache-2.0

---

## Table of Contents

- [Design Philosophy](#design-philosophy)
- [Use Cases](#use-cases)
- [How It Differs from easy-pid-controller](#how-it-differs-from-easy-pid-controller)
- [The Signal Chain](#the-signal-chain)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Node Configuration](#node-configuration)
- [Input Topics](#input-topics)
- [Output Payload](#output-payload)
- [Watchdog Behaviour](#watchdog-behaviour)
- [ADC Count Ranges](#adc-count-ranges)
- [Derivative Filter](#derivative-filter)
- [Wiring to Hardware](#wiring-to-hardware)
- [Limitations](#limitations)
- [Changelog](#changelog)

---

## Design Philosophy

Most Node-RED PID nodes â€” including this author's own `easy-pid-controller` â€” are designed as **supervisory controllers**. They receive process values in engineering units, compute a PID output in engineering units, and publish it for dashboards, historians, and SCADA displays. The actual hardware driving (count-level I/O) is left to a PLC running beneath.

This is the correct architecture when a PLC is present.

However, a growing class of industrial hardware runs Node-RED **as the PLC runtime itself**:

- **Kunbus RevPi** â€” Raspberry Pi CM in DIN-rail housing with industrial I/O modules
- **Siemens SIMATIC IOT2050** â€” Linux industrial gateway with Node-RED built in
- **Wago PFC200** â€” IEC 61131-3 soft-PLC with Node-RED runtime
- **Advantech ADAM-3600** â€” Linux edge controller with Node-RED
- **Beckhoff CX series** â€” TwinCAT soft-PLC, Node-RED runs alongside
- Any **Linux-based soft-PLC** running Codesys runtime + Node-RED on the same hardware

On these platforms, **there is no PLC beneath**. Node-RED reads raw ADC counts directly from hardware registers and must write raw DAC counts back to hardware registers to drive physical actuators. Engineering unit conversion, PID computation, and hardware I/O are all owned by Node-RED.

`hardloop-pid-controller` is built for exactly this scenario.

### Core Principles

1. **Counts in, counts out** â€” PV arrives as a raw integer from the AI hardware register. AO_count leaves as a raw integer ready to write to the AO hardware register. No external conversion nodes required.

2. **PID runs in engineering units** â€” The scaling happens inside the node, not in the flow. The engineer configures AI count range and EU range once in the node editor. The PID always operates in physically meaningful values (Â°C, bar, RPM) regardless of the card resolution.

3. **AI and AO cards are configured independently** â€” In real installations, the input card and output card often have different resolutions (e.g. AI: 0â€“4000, AO: 0â€“4000, or AI: 0â€“16000, AO: 0â€“8000). Both are configured separately to match the actual card datasheet.

4. **Watchdog is mandatory, not optional** â€” In supervisory mode, a Node-RED crash means the dashboard goes dark. In hard-loop mode, a crash means the physical process runs uncontrolled. The built-in watchdog forces a safe AO count output if PV stops arriving, and stops the loop until the fault is cleared.

5. **Derivative filtering for ADC noise** â€” Integer counts produce a staircase waveform. Without filtering, the derivative term amplifies every LSB step into a spike. A configurable first-order low-pass filter on the D term is included as a first-class parameter, not an afterthought.

6. **Honest about timing limits** â€” Node-RED runs on the V8 event loop. It is not a real-time OS. This node is suitable for slow-to-medium processes (temperature, level, pressure, flow) with loop times â‰¥100ms. It is **not suitable** for fast loops (position, speed, torque). This is stated clearly in the node help and this README, not buried in small print.

---

## Use Cases

### Suitable

| Process | Typical loop time | Notes |
|---|---|---|
| Temperature control (oven, tank, reactor) | 500ms â€“ 2000ms | Classic PID application, ideal fit |
| Liquid level control | 500ms â€“ 2000ms | Slow dynamics, excellent fit |
| Pressure control (vessel, pipeline) | 200ms â€“ 1000ms | Depends on vessel volume |
| Flow control (liquid) | 100ms â€“ 500ms | Acceptable at lower end of range |
| Humidity control | 1000ms â€“ 5000ms | Very slow, ideal fit |
| pH / conductivity control | 500ms â€“ 2000ms | Chemical dosing processes |

### Not Suitable

| Process | Reason |
|---|---|
| Servo position control | Requires <1ms loop, hard real-time OS needed |
| VFD speed control (closed loop) | Requires <10ms, use drive's internal PID |
| Torque control | Sub-millisecond, requires dedicated drive firmware |
| CNC axis control | Hard real-time, LinuxCNC or dedicated controller |

---

## How It Differs from easy-pid-controller

| | `easy-pid-controller` | `hardloop-pid-controller` |
|---|---|---|
| **Role** | Supervisory / analytical | Soft-PLC hard control loop |
| **PV input** | Engineering units (float) | Raw ADC counts (integer) |
| **SV input** | Engineering units | Engineering units |
| **PID domain** | Engineering units | Engineering units (converted internally) |
| **Primary output** | `Signal` float (mA or V) for tracing | `AO_count` integer for register write |
| **Secondary output** | `Signal` for dashboard | `Signal` for tracing + `AO_count` for hardware |
| **Count scaling** | None (PLC handles it) | Built in, both AI and AO sides |
| **AI/AO independent config** | N/A | Yes â€” separate count ranges |
| **Watchdog** | Not needed | Mandatory, configurable timeout |
| **Safe state output** | N/A | `safe_ao_count` on watchdog trip |
| **Derivative filter** | Not needed | Built in (ADC noise mitigation) |
| **Scan time monitoring** | No | `scan_ms` field every cycle |
| **Fault field in output** | No | Yes â€” `fault` boolean |
| **Target platforms** | Any Node-RED, any hardware | Soft-PLC platforms with direct hardware I/O |
| **Failure impact** | Dashboard goes dark | Writes safe state, stops loop |

Both nodes use the same `simple-pid-controller` v2.0.0 core, so all v2.0.0 features (anti-windup, output clamping, derivative on measurement, bumpless transfer, runtime gain tuning, reset, EventEmitter) are present in both.

---

## The Signal Chain

```
Physical Sensor           Soft-PLC Hardware          Node-RED
â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€           â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€  â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
4-20mA / 0-10V    â†’       AI card ADC                msg.topic  = 'PV'
(pressure, temp,           samples signal             msg.payload = 2731  (raw count)
 flow, level)              stores integer count   â†’
                           in hardware register
                                                           â†“ countToEU()
                                                       2731 counts â†’ 68.27Â°C
                                                           â†“
                                                       PID controller (EU domain)
                                                       SV=75Â°C, PV=68.27Â°C
                                                       output = 4.2Â°C equivalent
                                                           â†“ euToAOCount()
                                                       4.2Â°C â†’ 1680 counts
                                                           â†“
                           AO card DAC            â†   msg.payload.AO_count = 1680
                           count â†’ mA/V               (write to AO register via
                           drives valve/VFD/heater     Modbus TCP / SLMP / OPC UA)
```

---

## Installation

```bash
npm install node-red-contrib-hardloop-pid-controller
```

Or via the Node-RED palette manager: search `hardloop-pid`.

---

## Quick Start

### Minimum working flow

```
[Inject PV count]  â†’  [hardloop-pid-controller]  â†’  [Modbus write AO register]
[Inject SV]        â†—
[Inject auto:true] â†—
```

### Step by step

1. Add a `hardloop-pid-controller` node to your flow.
2. Configure it:
   - Set Kp, Ki, Kd for your process.
   - Set AI count range to match your AI card (e.g. 0â€“4000 for Mitsubishi Q64AD).
   - Set AO count range to match your AO card (e.g. 0â€“4000 for Q62DA).
   - Set EU range (e.g. 0â€“100 for 0â€“100Â°C sensor).
   - Set watchdog timeout (recommend 3Ã— your loop dt in ms).
   - Set safe AO count (0 = valve closed, or a safe mid-position count).
3. Wire a **Modbus TCP read** / **SLMP read** / **OPC UA read** node to publish PV counts with `msg.topic = "PV"`.
4. Send the setpoint: `msg.topic = "SV"`, `msg.payload = 75` (engineering units).
5. Start the loop: `msg.topic = "auto"`, `msg.payload = true`.
6. Wire the output to a **Modbus TCP write** / **SLMP write** / **OPC UA write** node targeting your AO register, using `msg.payload.AO_count`.

---

## Node Configuration

### PID Gains

| Field | Default | Description |
|---|---|---|
| Kp | `1.2` | Proportional gain |
| Ki | `1.0` | Integral gain |
| Kd | `0.01` | Derivative gain |
| Loop dt (s) | `1.0` | Target scan interval. Minimum 0.05s (50ms) recommended. |

### AI Input â€” Count Range

| Field | Default | Description |
|---|---|---|
| AI Count Min | `0` | Minimum count from AI card (0 for 0-10V full range; check datasheet for 4-20mA live-zero offset) |
| AI Count Max | `4000` | Maximum count. Common values: 4000, 8000, 16000 â€” check your card datasheet |

### AO Output â€” Count Range

| Field | Default | Description |
|---|---|---|
| AO Count Min | `0` | Minimum count for AO card |
| AO Count Max | `4000` | Maximum count for AO card. Configure from AO card datasheet independently of AI |

### Engineering Unit Range

| Field | Default | Description |
|---|---|---|
| EU Min | `0` | Engineering unit value at count_min (e.g. 0Â°C, 0 bar) |
| EU Max | `100` | Engineering unit value at count_max (e.g. 100Â°C, 10 bar) |
| Signal Type | `4-20mA` | Physical signal standard â€” affects Signal output field and documentation only. Count scaling is purely linear. |

### PID Safety Limits

| Field | Default | Description |
|---|---|---|
| Output Min (EU) | `0` | Lower clamp on PID output in engineering units |
| Output Max (EU) | `100` | Upper clamp on PID output in engineering units |
| Integral Min | `-500` | Anti-windup lower clamp on integral accumulator |
| Integral Max | `500` | Anti-windup upper clamp on integral accumulator |
| Deadband (EU) | `0` | Suppress output when |error| â‰¤ deadband. Set 0 to disable |
| Settled Tolerance (EU) | `0` | Stop loop when |error| â‰¤ tolerance. Set 0 to disable |
| Derivative Filter Î± | `1.0` | Low-pass filter on D term. 1.0 = no filter. 0.3 = moderate. 0.1 = heavy. |

### Watchdog & Safe State

| Field | Default | Description |
|---|---|---|
| Watchdog (ms) | `3000` | Max time between PV updates before fault. Recommend 3Ã— loop dt |
| Safe AO Count | `0` | Count written to output on watchdog trip. Typically 0 (valve closed) |

---

## Input Topics

| `msg.topic` | `msg.payload` | Description |
|---|---|---|
| `PV` | integer (raw AI count) | Current process variable as raw count from hardware register |
| `SV` | number (EU) | Target setpoint in engineering units. Resets integral and derivative state. |
| `auto` | `true` / `false` | `true` â€” start loop. `false` â€” stop loop, switch to manual mode |
| `gains` | `{ k_p, k_i, k_d }` | Update PID gains at runtime without restarting |
| `mode` | `"auto"` or `"manual"` | Switch mode. `manualâ†’auto` performs bumpless transfer |
| `manual_output` | number (EU) | Fixed EU output for manual mode. Seeds bumpless transfer. |
| `reset` | any | Clear integral, derivative, fault state, timestamps |

---

## Output Payload

Every cycle in auto mode, `msg.payload` contains:

| Field | Type | Description |
|---|---|---|
| `AO_count` | integer | **Write this to your AO hardware register.** Clamped to [ao_count_min, ao_count_max]. |
| `PV_count` | integer | Raw AI count as received |
| `PV_eu` | number | PV converted to engineering units |
| `SV_eu` | number | Current setpoint in engineering units |
| `error_eu` | number | SV_eu âˆ’ PV_eu |
| `P` | number | Proportional component (EU) |
| `I` | number | Integral component (EU, anti-windup clamped) |
| `D` | number | Derivative component (EU, filtered) |
| `output_eu` | number | Raw clamped PID output in engineering units |
| `Signal` | number | Output in physical signal units (mA or V) for tracing |
| `fault` | boolean | `true` if watchdog has tripped |
| `watchdog_ok` | boolean | `true` on every healthy cycle |
| `scan_ms` | number | Actual scan execution time in ms (monitor for jitter) |
| `mode` | string | `"auto"`, `"manual"`, or `"fault"` |

On **watchdog trip or fault**, the output contains:

| Field | Value |
|---|---|
| `AO_count` | `safe_ao_count` |
| `fault` | `true` |
| `watchdog_ok` | `false` |
| `fault_reason` | human-readable string |
| `mode` | `"fault"` |

---

## Watchdog Behaviour

The watchdog is a safety mechanism for hard-loop deployments where Node-RED is the control system. It works as follows:

```
Every valid PV message  â†’  watchdog timer resets (countdown restarts)
PV stops arriving       â†’  countdown expires
Timeout fires           â†’  AO_count = safe_ao_count sent immediately
                           PID loop stopped
                           Node status: FAULT
                           fault=true in all subsequent outputs

Recovery:
  1. Fix the PV data source (SLMP/Modbus read / sensor)
  2. Send a valid PV message (this clears the fault flag)
  3. Send  auto: true  to restart the loop
```

**Recommended watchdog timeout:** 3Ã— your loop `dt`. For a 1-second loop, set watchdog to 3000ms.

**Safe AO count selection:**
- For fail-closed valves: `0` (minimum count = valve closed)
- For fail-open valves: `ao_count_max`
- For fail-in-place: use a mid-scale value that matches the last known good position (advanced â€” requires external logic to track last position)

---

## ADC Count Ranges

Common PLC analog input card resolutions and their count ranges:

| Count Range | Bits (effective) | Example Cards |
|---|---|---|
| 0 â€“ 4000 | ~12-bit | Mitsubishi Q64AD, FX3U-4AD, many Siemens S7-300 SM331 |
| 0 â€“ 8000 | ~13-bit | Mitsubishi iQ-R R60AD, some OMRON CJ1W |
| 0 â€“ 16000 | ~14-bit | Mitsubishi Q68AD, R60ADI, Yokogawa high-res cards |
| 0 â€“ 27648 | 15-bit | Siemens S7-1200 / S7-1500 standard AI modules |
| 0 â€“ 32767 | 15-bit | Generic 15-bit ADC, some Beckhoff EL3xxx |
| 0 â€“ 65535 | 16-bit | High-resolution process cards, Beckhoff EL3174 |

**Always verify the count range from your card's datasheet.** The same card family can have different ranges depending on firmware version or configuration mode.

### 4-20mA Live-Zero Offset

For 4-20mA signals, some cards map:
- `4mA â†’ count_min` (e.g. 0 counts â€” the offset is in the physical signal, not the count)
- `20mA â†’ count_max` (e.g. 4000 counts)

Other cards use an internal offset:
- `0mA â†’ 0 counts`
- `4mA â†’ 800 counts` (on a 0â€“4000 card)
- `20mA â†’ 4000 counts`

Set `ai_count_min` accordingly. **Check your card datasheet.** Mitsubishi Q64AD uses the first convention (0 counts = 4mA). Some generic ADC boards use the second.

---

## Derivative Filter

ADC output is an integer staircase. Even with a perfectly stable physical process, consecutive readings may be `2981, 2982, 2981, 2983`. The derivative term `âˆ’d(PV)/dt` amplifies every step into a spike.

The `Derivative Filter Î±` parameter applies a first-order low-pass (exponential moving average) filter:

```
D_filtered(t) = Î± Ã— D_raw(t) + (1 âˆ’ Î±) Ã— D_filtered(tâˆ’1)
```

| Î± value | Effect | Use when |
|---|---|---|
| `1.0` | No filter (raw derivative) | PV is already smooth (e.g. filtered at sensor level) |
| `0.5` | Light filtering | Low noise, 12-bit ADC, moderate process dynamics |
| `0.3` | Moderate filtering | Typical 12-bit ADC with moderate noise |
| `0.1` | Heavy filtering | Noisy signal, high-resolution ADC, slow process |

Start with `0.3` for most industrial ADC inputs and adjust based on observed D term behaviour in Grafana/dashboard.

---

## Wiring to Hardware

### Typical Node-RED flow for a soft-PLC temperature control loop

```
[SLMP Read D100]  â†’ [set topic=PV]  â†’ [hardloop-pid]  â†’ [extract AO_count] â†’ [SLMP Write D200]
[Inject SV=75]    â†’â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â†—                  â†’ [InfluxDB out]
[Inject auto=true]â†’â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â†—                  â†’ [Dashboard gauge]
```

### Extracting AO_count for register write

```js
// Function node between hardloop-pid output and Modbus/SLMP write node
msg.payload = msg.payload.AO_count;  // integer count only
return msg;
```

### Fault handling

```js
// Function node to route fault state to alarm
if (msg.payload.fault === true) {
    node.warn("PID fault: " + msg.payload.fault_reason);
    // Optionally route to email/SMS/alarm node
}
return msg;
```

### Monitoring scan jitter in InfluxDB

```js
// Send scan_ms to InfluxDB for jitter monitoring
msg.payload = { scan_ms: msg.payload.scan_ms };
return msg;
```

---

## Limitations

- **Not suitable for fast loops** (<100ms) â€” Node-RED V8 event loop jitter of 10â€“50ms makes it unsafe for position, speed, or torque control at these speeds.
- **Not a hard real-time system** â€” Node-RED is a general-purpose runtime. Garbage collection pauses and event queue congestion can cause occasional missed cycles. Design your process to tolerate a missed scan without unsafe conditions.
- **Watchdog is software only** â€” The watchdog runs inside Node-RED. If Node-RED itself crashes, the watchdog cannot fire. Rely on the platform's hardware watchdog (most soft-PLC platforms have one) as the final safety layer.
- **Single-loop only** â€” Each node instance controls one process variable. Use multiple instances for multi-loop control.
- **No feedforward** â€” This is a pure feedback PID. Feedforward terms must be implemented externally in function nodes if needed.

---

## Changelog

### v1.0.0 â€” 2026-03-14

- Initial release
- Raw AI count input with configurable count range (0â€“4000 / 0â€“8000 / 0â€“16000 and any custom range)
- Count â†’ EU conversion (AI side) built into the node
- PID computation via `simple-pid-controller` v2.0.0 in EU domain
- EU â†’ AO count conversion (AO side) with integer rounding and hard clamping
- Separate AI and AO count range configuration for mixed-resolution card installations
- Built-in hardware watchdog with configurable timeout and safe AO count
- Safe state output on watchdog trip, fault, node close/redeploy
- Derivative low-pass filter (configurable Î±) for ADC quantisation noise mitigation
- `scan_ms` field for loop jitter monitoring
- `fault`, `watchdog_ok`, `mode` fields in every output message
- All `simple-pid-controller` v2.0.0 features: anti-windup, output clamping, deadband, bumpless transfer, runtime gain tuning, reset, settled tolerance
- Node-RED editor with full configuration form and inline help text
- Comprehensive help panel with input/output documentation and platform notes

---

## License

Copyright 2026, Harshad Joshi and Bufferstack.IO Analytics Technology LLP, Pune.

Licensed under the Apache-2.0 License. See `LICENSE` for full terms.