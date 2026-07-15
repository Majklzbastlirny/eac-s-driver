# EAC-S Python driver & control console

Python control for **ET System electronic EAC-S / EAC-3S** AC power sources
over the digital interface (RS232 option; the same ASCII protocol is used by
USB/LAN/RS485 variants). Written for the unit **EAC-S1000/RS232/V700**
(single-phase, U-mode, 0–700 VAC / 0–990 VDC, 5 A max), but the command set
covers the whole series.

## Files

| File              | Purpose                                                    |
|-------------------|------------------------------------------------------------|
| `eac_s.py`        | Driver library (class `EACS`) — import this in your scripts |
| `eac_cli.py`      | Interactive control console + one-shot command-line tool    |
| `eac_gui.py`      | Browser GUI — built-in web server, works on any OS, no extra deps |
| `test_offline.py` | Regression suite against a simulated device — no hardware needed (`python test_offline.py`) |
| `ETS_Manual_EAC-S_EN.pdf` | Manufacturer's manual (ET System electronic GmbH) — source of the protocol |

## Requirements

```
pip install pyserial
```

(Python 3.10+, pyserial 3.x)

## Wiring & serial settings

* RS-232 needs a **crossover (null-modem) cable** (manual ch. 13).
  Device DB9: pin 2 = RxD, 3 = TxD, 5 = GND, 7 = RTS, 8 = CTS.
* **Baud rate is automatic by default**: the driver probes 115200 first
  (where a previous session left the unit), then 9600 (delivery default),
  then other rates. Once found, it switches device + session to **115200**
  via the `PC` command — *not* saved to EEPROM, so a power cycle restores
  the unit's stored rate and the next connect just detects it again.
  Pass a number (`--baud 9600`, `EACS(port, baudrate=9600)`) for a fixed
  rate with no probing or switching; `EACS(port, target_baud=None)` detects
  but stays at the found rate.
* Delivery default: **9600 baud, 8 data bits, no parity**. The manual's
  example shows 2 stop bits with echo ON for the RS232 option; the driver
  auto-detects echo, and 1 stop bit normally works. If you get garbage or
  timeouts, try `--stopbits 2`.
* Forgot the programmed settings? Set **DIP switches 1–5 to ON** and power
  cycle → interface resets to `9600,N,8,1`. Restore the switches afterwards
  (at least one ON, others OFF).
* Higher speed: `PC,115200,N,8,1,N,N` + `SS` (or `EACS.program_rs232(115200)`).

## Quick start — console

```
python eac_cli.py --list          # find your COM port
python eac_cli.py COM3            # open interactive console
```

```
EAC-S> remote            # take remote control (GTR)
EAC-S> u 230             # AC voltage 230 Vrms      (u 10% also works)
EAC-S> i 1               # current limit 1 A
EAC-S> f 50              # 50 Hz
EAC-S> wave sine
EAC-S> on                # enable output (SB,R)
EAC-S> meas              # snapshot: U, I, P, S, Q, PF, f
EAC-S> mon 0.5           # live monitor every 0.5 s, Ctrl+C stops
EAC-S> watch             # live mirror of the device state (works in LOCAL
                         #   mode too - follow what the panel operator does);
                         #   state changes are kept as history lines
EAC-S> watch all         # every available reading; or pick columns:
EAC-S> watch 0.2 u,i,pf,cfu
                         # fields: u udc upk i idc ipk p s q pf cfu cfi f
EAC-S> off               # standby (SB,S)
EAC-S> local             # give the front panel back (GTL)
EAC-S> quit
```

Type `help` in the console for the full command list (status word, limits,
DC offset, phase angle, cycle mode, phase dip, raw commands...).

One-shot mode runs a single command and exits:

```
python eac_cli.py COM3 u 115
python eac_cli.py COM3 on
python eac_cli.py COM3 meas
python eac_cli.py COM3 off
python eac_cli.py COM3 raw STATUS
```

## Quick start — browser GUI

```
python eac_gui.py COM3                  # opens http://127.0.0.1:8432
python eac_gui.py COM3 --host 0.0.0.0   # reachable from LAN (phone/tablet)
```

A single-page control panel served by a built-in web server (Python stdlib
only — no extra dependencies, works on Windows/Linux/macOS and in any
browser, including phones). It shows live readouts (U, Udc, I, P, PF, f,
peaks, crest factors), mode/output/waveform state with warning flags, and
lets you set voltage/current/frequency/phase/waveform, toggle remote/local
and switch the output (with a confirmation dialog showing the set points).

* Any number of browsers can watch; the server owns the serial port and
  serializes all device access.
* Survives a device power cycle: when the unit goes silent, the server
  re-runs baud detection automatically and carries on.
* **No authentication** — the default bind is localhost. Only use
  `--host 0.0.0.0` on a network where everyone may control the PSU.

## Quick start — library

```python
from eac_s import EACS, Waveform

with EACS("COM3") as psu:                 # auto-baud: finds + upgrades to 115200
    print(psu.identify())                 # ID
    print(psu.limits())                   # real device limits (LIM*)

    psu.remote()                          # GTR
    psu.set_voltage_ac(230)               # UAC,230   (also '10%' / 'DEFAULT')
    psu.set_current_limit(1.0)            # IA,1
    psu.set_frequency(50)                 # FA,50
    psu.set_waveform(Waveform.SINE)       # WAVE,1
    psu.output_on()                       # SB,R

    print(psu.measure_voltage(), "V")     # MUA
    print(psu.measure_current(), "A")     # MIA
    print(psu.measure_power(), "W")       # MPA
    print(psu.status())                   # decoded STATUS word

    psu.output_off()                      # SB,S
    psu.local()                           # GTL
```

### User-defined waveform (no SD card needed)

A wave is 3600 samples in −1.0 … +1.0 (one full period), sent over the
interface:

```python
import math
samples = [math.sin(math.radians(i / 10)) ** 3 for i in range(3600)]

psu.upload_wave(samples, dest="OUT")     # volatile, straight to the output
# or: psu.upload_wave(samples, dest="MEM1")   # store in internal memory 1
psu.set_voltage_ac("50%")                # percent is handy for arbitrary waves
```

At 9600 baud the transfer takes ~30 s; raise the baud rate for frequent use.

### Timed / cyclic output

```python
psu.output_pulse(500)          # SB,500  -> output on for 500 ms
psu.cycle_configure(3, 5)      # 3 s on / 5 s off
psu.cycle_start()              # CYCLE,S
psu.cycle_stop()               # CYCLE,R
psu.dip_configure(60)          # 60 ms interruption ...
psu.dip_start()                # ... triggered at next reference zero cross
```

## Firmware quirks (observed on HE-ACS-1000/V700/LTRS, fw V0.72)

* `MPS` (apparent power) and `MPQ` (reactive power) are **not implemented** —
  the driver detects this on first use and reports them as `n/a` from then on.
* `MPF`/`MCU`/`MCI` answer with dashes (`-.----`) when the load is too small
  to calculate — the driver returns `NaN`, the console prints `-`.
* Out-of-range set values (e.g. `u 1000` on a 700 V unit) are silently
  ignored by the device; the console reads the value back after setting and
  warns when the device didn't accept it.
* `GTL` (back to local) **restores the front-panel settings** — e.g. the
  waveform selected by the panel buttons replaces whatever was set remotely.
  Remote selections only persist while in remote mode.
* **Set commands are ignored in LOCAL mode** — including `SB,S` (output off)!
  Send `GTR` first; note that activating remote immediately applies the
  interface set points (so if those are 0 V/standby, `GTR` alone kills the
  output). The CLI's `off` verifies and warns if the output stayed on.
* Set-point queries (`UAC`, `IA`, `FA` without parameter) return the
  **interface's own set points**, not the front-panel knob positions (the
  unit keeps separate set points per control source). The panel state is
  visible indirectly: `STATUS` shows waveform + output on/off, `MFA` the
  frequency, and with the output on `MUA`/`MIA` show the actual output. The
  current-limit knob is invisible until limiting engages (STATUS I-Limit bit).
* During a timed pulse (`SB,<ms>`) the `SB` query keeps reporting standby;
  only the STATUS standby bit tracks the real output state (the driver's
  `get_output()` therefore uses STATUS).
* The square wave has ~20 % edge overshoot/ringing at the output: `MUS`
  (peak) and `MCU` (crest) read ~1.2 instead of the ideal 1.0, while `MUA`
  (rms) is accurate. Physics of the analog output stage, not a protocol bug.
* The peak detector (`MUS`) is generally noisy (±10 % between readings on an
  identical signal) — verify waveform shapes by their **rms** value, not by
  crest factor.
* Selecting a waveform loads a table (~170 ms) during which the device drops
  commands (`SB,R` gets lost, `MWAVE` answers the stale value). The driver's
  `set_waveform()` waits 0.3 s to cover this. Wave storage to `MEM1`-`MEM3`
  and recall via `WAVE,4`-`6` is verified working; text parameters like
  `WAVE,Memory1` also work (`WAVE,MEM1` is a syntax error).
* Waveform 7 is reported as `EN61000-4-11` (not "MMC-Direct") — this is the
  voltage-dips test firmware. Custom wave upload (`WAV,OUT`) **works**
  (verified by crest-factor measurement: sin³ gave CF 1.798 vs. 1.789
  theoretical), but the firmware never sets the "wave ok" confirmation bit
  and raises a spurious STB command-error during the transfer. Therefore
  `upload_wave()` returns a bool instead of raising: `False` just means
  "not confirmed", not "failed".

## Notes & safety

* **`remote()` (GTR) first** — set commands are honored in remote mode.
  `local()` (GTL) hands control back to the front panel.
* The driver queries nothing on its own and never enables the output by
  itself; `output_on()` applies whatever voltage is currently set.
* Values above the device maximum are **silently ignored** by the unit (a
  range-error bit is set in STB). Construct `EACS(..., auto_check=True)` to
  raise `EACSCommandError` after every set command instead, or call
  `check_error()` manually.
* This unit outputs up to **700 VAC / 990 VDC — lethal voltages**. Treat the
  output like mains wiring.
* SD-card features of the manual (data log, script mode) are not applicable
  to this unit and not implemented.
