# 🧙 Quick Start Wizard — improvements

## Automatic Vbus capture + auto-tune of protection parameters

In the "Power supply and protections" step, a new **⚡ Set voltage** button that:

1. Reads the real `vbus_voltage` from the ODrive via serial
2. Auto-configures every protection parameter around that voltage:
   - `dc_bus_undervoltage_trip_level` (undervoltage trip)
   - `dc_bus_overvoltage_trip_level` (overvoltage trip)
   - `dc_bus_overvoltage_ramp_start` (where the brake starts increasing duty)
   - `dc_bus_overvoltage_ramp_end` (where the brake saturates at 100%)
   - Enables `enable_dc_bus_overvoltage_ramp` automatically

## Dedicated step for encoder Z

- **Step 10 (new): "Configure Z (index pulse)"** — end-to-end wizard for encoder index setup. The motor spins, searches for Z, recalibrates the offset anchored to Z, marks the flags + saves. A "Skip" button is available for setups without a Z wire (just runs a 1-turn offset recalibration).
- **Explicit failure detection:** Z not found, offset cal failed — clear messages pointing to which ODrive error to check.

## Mechanical center capture

- **📍 Capture mechanical center** button in the Encoder tab (replaces the old numeric `index_offset` slider).
- The user rotates the wheel to the neutral position of the assembly and clicks the button. The system captures, writes, saves + reboots, auto-reconnects.

## Analog inputs (Axis processor)

- Configurable low-pass filter + calibration via AMIN/AMAX
- UI in the Inputs tab: dual bar (raw + filtered) for live visualization

---

# 🧲 Onboard AS5047 absolute encoder

This release ships a full rewrite of the AS5047 SPI driver in the ODrive firmware, driven by chronic communication failures observed on the MKS XDrive Mini. The sensor is now stable and usable, and the HTML tool gains a complete visual diagnostic panel.

**What improved in practice:**

- **Reliable communication** — clean, persistent SPI reads with auto-recovery from transient glitches.
- **Encoder diagnostics panel** (Encoder tab): live SPI counters, magnet status with a visual signal strength bar (AGC), and explicit "magnet too far / weak" and "magnet too close / strong" indicators — no more guessing the right mechanical distance for the AS5047.

**The heart of the panel: AGC (Automatic Gain Control)**

The AS5047 continuously reports the gain it had to apply to read the magnetic field. That number tells you exactly how well the magnet is positioned:

- **AGC ≈ 128** → ideal distance, robust signal
- **AGC low (< 30)** → magnet too close, sensor saturating
- **AGC high (> 220)** → magnet too far, weak signal

**Recommended procedure for first install:**

1. Apply the AS5047 preset (button on the Encoder tab) → Save + reboot
2. Run `FULL_CALIBRATION_SEQUENCE` (state 3) on the Debug tab
3. Set `motor.pre_calibrated` and `encoder.pre_calibrated` to `True`, disable `startup_*_calibration` → Save
4. Center the wheel manually and click **"Capture mechanical center"**

---

# 🔬 1 kHz telemetry via HID

`vel_estimate`, `Iq_measured`, `torque_output`, `vbus`, `ibus` and `I_brake` are now embedded in the HID input report at native 1 kHz.

Doesn't conflict with the game — you can use all of it during gameplay.

---

# 🎛️ PiP Overlay — live at 1 kHz, HID-driven (no serial polling)

- **DC Bus panel** (vbus / ibus / Iq / I-brake)
- **Wheel panel** (torque + position)
- **New Spectrum panel (FFT)** optional inside the PiP — continuous spectrum during gameplay
- **P brake (W)** computed at 1 kHz with R · ⟨I²⟩ (used to be polled, missed short bursts)
- **P motor (W) new** — mechanical power + copper losses

---

# 📊 Live FFT (Spectrum Analyzer) — new

Available as a panel in the PiP overlay.

- Continuous FFT at 1 kHz, 1024-sample window, 50% overlap
- **"τ_cmd vs Iq" mode** — overlays what the game asks vs. what the motor delivers
- **"Bode" mode** (Iq/τ_cmd) — FFB chain transfer function extracted from REAL gameplay (no need for a synthetic sweep)
- Educational vertical lines: `f_c_mec`, `f_LR`, current CF filter
- Shared engine between the PT card and the PiP — one FFT, two renderers

---

# 🚀 Performance Test (PT tab) — overhaul

- **Robust J** — computed by the median of `Iq×tc/α` in the stable zone (30–80% of the launch) instead of the noisy `τ_assumed/peak_accel`
- **Quality indicator (CV)** — shows confidence in the measured J
- **Filter recommendation cards** — CF, Damper, Friction, Inertia suggested from the measured J + b + Kt
- **Iq overlay on the charts** during the launch — see when it saturates

---

# 🌀 Coastdown and Frequency Sweep refactored

- Both now use the 1 kHz HID stream (previously serial ~20–150 Hz)
- **Coastdown:** much cleaner `b` measurement, no polling jitter
- **Sweep:** uses the real measured torque (Iq × Kt) instead of the commanded — more faithful Bode

---

# 📈 Frequency Sweep test (new) — Bode of the FFB chain

Available in the Performance Test tab, optional. Runs a real sinusoidal sweep (HID FFB Periodic effect) across several frequencies and measures the wheel position response at each one. Result: the measured Bode curve of your entire FFB chain — from the command to the motor.

### What it does internally

- Injects a torque sine at 0.5 / 1 / 2 / 5 / 10 / 20 / 50 / 100 Hz
- Captures position via HID at 1 kHz during each frequency
- Sinusoidal fit to extract amplitude and phase of the response

---

# Anti-Cogging Experimental

## 🎯 Auto-tune Velocity PI — Experimental

Automatic calibration of the velocity loop gains. Run once, save, anti-cogging consumes.

## 🔬 Anti-cogging via host — Experimental

- **Bidirectional** (5 fwd + 5 rev turns, configurable) — cancels friction bias
- **Save device map (📥)** — exports what's in the firmware (host or native)
- **Load external JSON (📂)** — import any map to apply
