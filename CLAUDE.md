# EAC-S Driver project

Python control software for the user's AC power source: **ET System
EAC-S1000/RS232/V700** (ID string `HE-ACS-1000/V700/LTRS`, fw V0.72,
0-700 VAC / 0-990 VDC / 5 A, U-mode, single-phase, no SD card).
Protocol source: `ETS_Manual_EAC-S_EN.pdf` ch. 11 (ASCII over RS-232).
GitHub: https://github.com/Majklzbastlirny/eac-s-driver (public, MIT).

## Layout

- `eac_s.py` — driver library (`EACS` class). All protocol knowledge lives here.
- `eac_cli.py` — interactive console + one-shot commands (`watch` = live mirror).
- `eac_gui.py` — browser GUI, stdlib HTTP server on :8432 (`python eac_gui.py COM19`).
- `test_offline.py` — regression suite vs. a simulated device: `python test_offline.py`.
  Run it after any driver change; it must end with `ALL TESTS PASSED`.
- `README.md` — user docs **and the authoritative list of firmware quirks**.
  Read the quirks section before assuming manual-documented behavior; several
  manual claims are wrong for this firmware (LOCAL-mode latching, MPS/MPQ
  missing, status lag after GTR/GTL/SB, wave-table load delay, noisy peak
  detector, GTL restores panel settings...).

## Hardware testing

- PSU is on **COM19** (null-modem RS-232). It may be disconnected — check first.
- The user invites live testing; keep voltages low (<= 20 V) unless a test
  needs more, keep output-on windows brief. A 230 V / 105 W halogen bulb is
  available as load.
- **Ask before anything persistent**: EEPROM writes (`SS`, `*,DEFAULT`,
  `GTR,2`, `LLO,1`) and wave memories (MEM1 holds a sin^3 test wave;
  MEM2/MEM3 contents unknown).
- Serial: auto-baud is the default (device probes 115200 -> 9600 -> ...,
  session upgrades to 115200, EEPROM stays 9600). Echo is ON on this unit.

## Safety-critical knowledge

Commands sent in LOCAL mode **latch** into the interface set-point state and
`GTR` applies them all at once — including a latched output-ON. That's why
GUI/CLI send `SB,S` right before `GTR` and the GUI refuses output-ON in LOCAL.
Never remove these guards.

## Environment

- Windows 11, PowerShell 5.1. Python 3.13, pyserial 3.5 installed.
- git is at `C:\Program Files\Git\cmd` but **not on PATH** — add per session:
  `$env:Path += ";C:\Program Files\Git\cmd"`.
- GitHub CLI: `& "C:\Program Files\GitHub CLI\gh.exe"`, logged in as
  Majklzbastlirny. Commit identity is set repo-locally (noreply email).
- Commit + push after each completed feature/fix.

## Open items (as of 2026-07-15)

- Verify on hardware: standby-before-GTR guard, 1 Hz slow-measurement
  handling (both written after the PSU was disconnected).
- Untested: EEPROM-writing features (`DEFAULT`, `SS`, `GTR,2`/`LLO`
  persistence), MEM2/MEM3 storage.
- User reported "everything I set was gone" mid-session — most likely a PSU
  power cycle (fresh boot = memory 0: 0 V, 0 A, standby); if it recurs
  without a power cycle, investigate.
