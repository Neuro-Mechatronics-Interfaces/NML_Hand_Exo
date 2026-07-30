---
layout: single
title: "Assembly"
permalink: /assembly/
toc: true
toc_sticky: true
classes: wide
description: "Complete, step-by-step build instructions for the NML Hand Exoskeleton: tools, BOM, print settings, sub-assemblies, wiring, torque specs, and bring-up."
---

> **About this guide**  
> Structure and presentation are inspired by exemplary open-source robotics docs. We keep the content specific to the NML Hand Exo.

## At a Glance

- **Audience:** researchers, graduate students, and advanced makers  
- **Build time:** ~6–10 hours over 2 sessions (mechanics + electronics)  
- **Difficulty:** Intermediate (precision mechanical + light electronics)  
- **Safety:** E-stop accessible; no power during wiring; eye protection

> **Safety first**  
> Always disconnect power before adjusting wiring, and keep fingers clear of force paths. Treat energized actuators as **powered tools**—they can pinch and crush.  
{: .notice--warning}

---

## Bill of Materials (BOM)

Use the table below for quick reference, or download the full spreadsheet.

- **Download:** [`/assets/bom/nml_hand_exo_bom.xlsx`](/assets/bom/nml_hand_exo_bom.xlsx){: .btn .btn--primary }  
- **Vendors & alternates:** see Notes column for drop-in equivalents  
- **Quantities:** per single exo (mirror build requires ×2 for handed parts)

| Group | Part / Spec | Qty | Example PN / Link | Notes |
|---|---|---:|---|---|
| Actuation | Dynamixel (or your model) | 5 | — | Match torque to finger section |
| Control | OpenRB-150 (or compatible) | 1 | — | USB/Dynamixel default baud 1000000 |
| Power | 12 V DC PSU, 5 A | 1 | — | Bench supply OK |
| Fasteners | M2/M3/M4 assortment | — | — | Button/SHCS + nylocs |
| Sensors | (optional) IMU/EMG set | — | — | Skip for basic build |
| Printed | See **Print list** below | — | — | PETG/ABS recommended |
| Cabling | JST-XH/PH, Dupont leads | — | — | Color-code harness |

---

## Tools & Materials

- Metric hex drivers (1.5–3 mm), torque screwdriver (recommended)  
- Side cutters, needle-nose pliers, flush trimmer  
- Soldering iron + heatshrink (for custom harness)  
- Threadlocker (medium), PTFE dry lube for sliding surfaces  
- **ESD** strap/mat if handling IMU/MCU

---

## Print List & Settings

- **Material:** PETG or ABS (PLA acceptable for early protos; avoid for load-bearing parts)  
- **Layer height:** 0.2 mm (0.28 mm for large shells)  
- **Walls:** 3–4; **Infill:** 20–35% (gyroid/cubic)  
- **Orientation:** prioritize **hole alignment & layer-line strength** on living hinges and thin walls  
- **Critical fit:** Ream/clear M3/M4 holes to nominal after print; chase heat-set inserts carefully.

---

## Pre-Assembly: Electronics Preparation

> **Do this BEFORE mechanical assembly!** Testing electronics on the bench prevents frustration later.  
{: .notice--info}

### 1) Flash Controller Firmware

1. Clone the [NML Hand Exo repository](https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo) and navigate to `src/cpp/nml_hand_exo/`.
2. Follow the instructions in the main `README.md` to flash your controller (e.g., OpenRB-150) using Arduino IDE.
3. Connect via USB and note the serial port:

```powershell
# Windows
python examples\tools\hand_exo_cli.py --list-ports
# Look for your device (e.g., COM5)
```

```bash
# macOS
ls /dev/tty.usbmodem* /dev/tty.usbserial*

# Linux
ls /dev/ttyUSB* /dev/ttyACM*
```

4. Test communication:

```powershell
python examples\tools\hand_exo_cli.py --connect COM5 --baud 1000000 --info
```

**Expected:** Firmware version and device info without errors.

---

### 2) Program Motor IDs

> Set unique IDs for each motor **on the bench** before installation.  
{: .notice--warning}

1. Connect one motor at a time to avoid ID conflicts.
2. Use Dynamixel Wizard (or your motor manufacturer's tool) to set IDs:
   - **Thumb:** ID 1
   - **Index:** ID 2
   - **Middle:** ID 3
   - **Ring:** ID 4
   - **Pinky:** ID 5 (if applicable)

3. Test each motor's motion range:
   - Verify smooth rotation/actuation
   - Check current draw is reasonable
   - Confirm no grinding or binding

4. Log motor IDs in `/config/motor_map.yml` for reference.

5. Label each motor cable with its ID (use tape/stickers).

---

### 3) Verify Power Supply

1. Confirm polarity: **+12 V** (center-positive if barrel jack).
2. Test under load: connect controller + one motor; command motion.
3. Verify E-stop or power cutoff is accessible.

**QA checks**  
- [ ] Voltage stable under load (no significant droop)
- [ ] No overheating of PSU or cables
- [ ] Emergency cutoff tested and working

---

## Pre-Assembly Checklist

Before starting mechanical assembly, verify:

- [ ] **Printed parts:** All finished, supports removed, holes cleared and reamed
- [ ] **Controller:** Firmware flashed, serial communication tested
- [ ] **Motors:** IDs programmed (1-5), each motor tested individually on bench
- [ ] **Power supply:** Polarity confirmed, tested under load
- [ ] **Cables:** Labeled by motor ID, connectors verified
- [ ] **Fasteners:** Sorted by size, threadlocker and tools ready
- [ ] **Workspace:** Clean bench, ESD protection if using IMU/sensors

---

## Mechanical Assembly

> **Electronics should already be tested.** Now we're building the mechanical structure.  
{: .notice--success}

### 1) Palm & Base

**Goal:** rigid base plane with aligned inserts and datum surfaces.

1. Heat-set M3 inserts into the palm and base as indicated. Allow to cool.  
2. Dry-fit the palm cover; confirm cable channels are free.  
3. (Optional) Apply thin PTFE dry lube to sliding interfaces.

**QA checks**  
- [ ] Inserts sit *flush*; no proud brass.  
- [ ] Base remains flat on bench; no rock.

---

### 2) Thumb Module

**Goal:** actuated thumb with clean tendon routing.

1. Mount motor to thumb bracket (do not overtighten; see torque table).  
2. Route tendon through guides; leave 30–40 mm service slack.  
3. Verify full travel manually—no scraping or binding.

**QA checks**  
- [ ] Tendon sheaths anchored; no kinks.  
- [ ] Free travel from end-stop to end-stop.

---

### 3) Finger Modules (Index–Ring)

**Goal:** repeatable assembly across fingers.

1. Install proximal links to palm with M3 × (length) screws.  
2. Add intermediate/terminal phalanges, checking hinge motion.  
3. Add return elastic/assist if used.

**QA checks**  
- [ ] All hinges move freely with even resistance.  
- [ ] Parallel fingers when fully extended (±1.5 mm).

---

### 4) Wrist/Back-of-Hand

1. Mount wrist shell and cable retainer.  
2. Stage cable bundle along dorsal channel with velcro ties.  
3. Confirm strain relief at each exit.

---

## Electronics Integration

> **Electronics should already be tested!** Now we integrate into the assembled mechanical structure.  
{: .notice--success}

### 1) Mount Controller

1. Mount the controller (e.g., OpenRB-150) on standoffs in the designated location.
2. Verify clearance below and around the controller.
3. Ensure USB port is accessible for programming/debugging.

---

### 2) Power Distribution

> Power stays **off** until final bring-up!  
{: .notice--danger}

1. Route main power: **+12 V** → fuse/E-stop → distribution board → motor bus.
2. Verify polarity at every connection point.
3. Use proper gauge wire for current requirements (typically 18-20 AWG for 5A).
4. Secure power connections with strain relief.

---

### 3) Motor Wiring

1. Connect motors using the pre-labeled cables (IDs 1-5).
2. Follow this connection order (reduces debugging later):
   - Thumb (ID 1)
   - Index (ID 2)
   - Middle (ID 3)
   - Ring (ID 4)
   - Pinky (ID 5, if applicable)

3. Run communication bus (TTL/RS-485) as a *separate* twisted pair from power.
4. Keep motor signal wires away from high-current power lines.

---

### 4) Cable Management

1. Dress cables along the dorsal (back-of-hand) channel with velcro ties.
2. Keep sharp bends away from moving joints (minimum bend radius: 10× cable diameter).
3. Add strain relief at each cable exit point.
4. Verify cables don't interfere with finger motion through full range.

**Final check before power-up**  
- [ ] All connections tight and verified
- [ ] Polarity checked at every point
- [ ] No pinched or kinked cables
- [ ] E-stop accessible and tested
- [ ] Motor IDs match cable labels

---

## Torque & Fastener Guidance

> Values are **starting points**; verify against your plastics and screws.

| Thread | Into | Start Torque | Notes |
|---:|---|---:|---|
| M2 | plastic | 0.15–0.2 N·m | stop at first solid resistance |
| M3 | plastic | 0.35–0.5 N·m | prefer heat-set inserts where possible |
| M4 | plastic | 0.8–1.0 N·m | washers reduce creep |

Add a tiny amount of **medium threadlocker** on metal-to-metal fasteners only.

---

## System Commissioning

> **This is your first power-up with everything assembled!**  
{: .notice--info}

### 1) Initial Power-Up

1. **Triple-check** all connections one more time.
2. Connect USB cable to PC.
3. **Apply power** to the system (12V supply).
4. Watch for any smoke, unusual sounds, or overheating → **Power off immediately** if detected.

---

### 2) Communication Verification

Open a terminal and verify serial communication:

```powershell
python examples\tools\hand_exo_cli.py --connect COM5 --baud 1000000 --info
```

**Expected output:**
- Firmware version
- Controller type  
- Motor IDs detected (1-5)
- No error messages

---

### 3) LED Sanity Check

Test that commands are being received:

```powershell
python examples\tools\hand_exo_cli.py --connect COM5 --send "led:1:on" --read-for 1
python examples\tools\hand_exo_cli.py --connect COM5 --send "led:1:off" --read-for 1
```

> If the LED toggles, communication is healthy! If not, verify port, baud rate, and power.  
{: .notice--success}

---

### 4) Motor Detection Test

Verify all motors are detected on the bus:

```powershell
# Scan for motors (adjust tool/command as needed)
python examples\tools\hand_exo_cli.py --connect COM5 --send "motor:scan" --read-for 3
```

**Expected:** All programmed motor IDs (1-5) should respond.

**If missing:**
- Check motor power connections
- Verify data bus wiring
- Confirm motor IDs were set correctly

---

## Motion Bring-Up

> **Keep the glove OFF your hand** during first motions!  
{: .notice--warning}

### 1) Individual Motor Tests

Test each motor one at a time:

```powershell
# Test each motor individually (adjust script/commands as needed)
python examples\02_motor_control\example_motor_control.py --connect COM5 --motor 1
python examples\02_motor_control\example_motor_control.py --connect COM5 --motor 2
# ... repeat for motors 3, 4, 5
```

**For each motor, verify:**
- [ ] Smooth motion through full range
- [ ] No binding or grinding sounds
- [ ] Current stays within limits
- [ ] Position tracking is accurate

---

### 2) Homing Procedure

Run the homing sequence:

```powershell
python examples\04_advanced\example_motor_config.py --connect COM5 --home
```

**Checks:**  
- [ ] All motors reach home position
- [ ] No stalls or skipped steps
- [ ] Position offsets are saved correctly

---

### 3) Safety Limit Tests

```powershell
python examples\04_advanced\example_advanced_config.py --connect COM5 --test-limits
```

**Verify:**  
- [ ] Current limits prevent damage at end-stops
- [ ] Motors stop when limits exceeded
- [ ] No unexpected heat buildup (< 50°C motor case)
- [ ] No unusual smells or smoke

---

### 4) Coordinated Motion Test

Once individual motors pass, test coordinated movements:

```powershell
python examples\01_basic\example_connect.py --connect COM5
```

**Checks:**
- [ ] All fingers move smoothly together
- [ ] No mechanical interference between fingers
- [ ] Cables don't snag during motion
- [ ] Timing is synchronized

---

## Fit & Comfort

- Add hook-and-loop straps; verify range does not force hyperextension.  
- Pad contact surfaces (silicone/foam) where needed.  
- Verify don/doff can be done **one-handed** in under 60 s with practice.

---

## Troubleshooting

- **No serial port:** try another cable/port; install USB CDC driver (Windows).  
- **Binding joint:** loosen, re-align hinge, re-ream bushings; re-lube sparingly.  
- **Overcurrent trips:** increase soft-start time; verify tendon path and stops.  
- **Oscillation:** reduce Kp; increase damping; confirm rigid mounts.

---

## What’s Next?

- **Examples:** try *Quick Gestures*, then *Telemetry + GUI* on the [Examples page](/examples/).  
- **API:** integrate the Python SDK for scripted experiments: [/python-api/](/python-api/).  
- **ROS 2:** coming soon—track progress on the roadmap.

---

### Image & File Conventions (for maintainers)

- Place assembly photos under: `/assets/images/assembly/`  
- Name by step: `01_palm_base.jpg`, `02_thumb_stage.jpg`, …  
- Keep max width ≈ 1600 px (JPG, 80–85 % quality); use PNG for line art.  
- CAD & printables live under `/hardware/` with a top-level README.
