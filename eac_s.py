"""
eac_s.py - Python driver for ET System electronic EAC-S / EAC-3S AC power sources.

Written for the ASCII command protocol described in the ETS manual
(chapter 11 "External Control: Computer"), tested against the command set of
firmware ~V2.69. Works over the RS232 option (also USB/LAN/RS485 variants,
since all digital interfaces share the same protocol).

Protocol summary
----------------
* Commands are plain ASCII, terminated with <CR> (or <LF>).
* Commands are not case sensitive.  Example:  "UAC,230.0<CR>"
* Query responses look like:  "UAC,230.0V<CR><LF>"
* The interface may have character ECHO enabled (delivery default for RS232).
  The driver auto-detects this and silently discards echoed lines.
* Numeric parameters may also be given in percent of the range: "UAC,10%".

Typical use (U-mode device such as EAC-S1000/RS232/V700):

    from eac_s import EACS

    with EACS("COM3") as psu:
        print(psu.identify())
        psu.remote()                # GTR
        psu.set_voltage_ac(230)     # UAC,230
        psu.set_current_limit(1)    # IA,1
        psu.set_frequency(50)       # FA,50
        psu.output_on()             # SB,R
        print(psu.measure_voltage(), "V", psu.measure_current(), "A")
        psu.output_off()            # SB,S
        psu.local()                 # GTL

Requires: pyserial
"""

from __future__ import annotations

import time
import warnings
from dataclasses import dataclass
from enum import IntEnum

import serial


__all__ = [
    "EACS", "Waveform", "Status",
    "EACSError", "EACSTimeout", "EACSCommandError",
]


class EACSError(Exception):
    """Base error of this driver."""


class EACSTimeout(EACSError):
    """No (valid) response received from the device."""


class EACSCommandError(EACSError):
    """The device reported an error in its status byte (STB)."""


class Waveform(IntEnum):
    """Parameter values of the WAVE command."""
    EXTERNAL = 0
    SINE = 1
    SQUARE = 2
    TRIANGLE = 3
    MEMORY1 = 4
    MEMORY2 = 5
    MEMORY3 = 6
    DIRECT = 7          # direct mode, used together with WAV,OUT


#: Error code from STB bits D2..D0 (RS232 interface, see manual ch. 13)
STB_ERRORS = {
    0b001: "Syntax error",
    0b010: "Command error",
    0b011: "Range error",
    0b100: "Unit error",
    0b101: "Hardware error",
    0b110: "Read error",
}


@dataclass
class Status:
    """Decoded 16-bit STATUS word (see manual, command STATUS)."""
    raw: str                    # e.g. "0000001000001000", MSB left
    remote: bool                # bit 0: 1 = remote control
    panel_locked: bool          # bit 1: 1 = front panel locked
    standby: bool               # bit 3: 1 = output locked (OFF)
    wave_ok: bool               # bit 4: wave transfer successful
    setpoint_active: bool       # bit 5: set value was put out
    waveform_bits: int          # bits 8..10 raw signal form field
    current_limiting: bool      # bit 13: I-Limit active
    overload_warning: bool      # bit 14: drain > nominal power
    overload_shutdown: bool     # bit 15: drain > peak power

    @property
    def output_on(self) -> bool:
        return not self.standby

    @property
    def waveform(self) -> Waveform:
        return Waveform(self.waveform_bits)

    def __str__(self) -> str:
        parts = [
            "REMOTE" if self.remote else "LOCAL",
            "OUTPUT OFF (standby)" if self.standby else "OUTPUT ON",
            f"wave {self.waveform.name}",
        ]
        if self.panel_locked:
            parts.append("panel locked")
        if self.current_limiting:
            parts.append("I-LIMIT")
        if self.overload_warning:
            parts.append("OVERLOAD warning")
        if self.overload_shutdown:
            parts.append("OVERLOAD SHUTDOWN")
        if self.wave_ok:
            parts.append("wave ok")
        return ", ".join(parts) + f"  [raw {self.raw}]"


def _to_param(value) -> str:
    """Format a set-point parameter (number, '15%' string or 'DEFAULT')."""
    if isinstance(value, str):
        return value.strip()
    if isinstance(value, (int, float)):
        return f"{value:g}"
    raise TypeError(f"unsupported parameter type: {type(value)!r}")


def _parse_number(payload: str) -> float:
    """Parse '230.0V' / '0.8000' / '60 ms' -> float (unit letters ignored).

    The device answers with dashes ('-.----') when a value cannot be
    calculated (e.g. power factor at no load) -> returns NaN.
    """
    payload = payload.strip()
    if payload and set(payload) <= set("-.,"):
        return float("nan")
    num = ""
    for ch in payload:
        if ch.isdigit() or ch in "+-.":
            num += ch
        elif num:
            break
    try:
        return float(num)
    except ValueError:
        raise EACSError(f"cannot parse numeric value from {payload!r}") from None


class EACS:
    """Driver for one EAC-S / EAC-3S source on a serial port.

    Parameters
    ----------
    port      : serial port name, e.g. "COM3" (Windows) or "/dev/ttyUSB0".
    baudrate  : "auto" (default) probes BAUD_CANDIDATES until the device
                answers, then switches device + port to target_baud for this
                session (not saved to EEPROM: a power cycle restores the
                device's stored rate, and auto-detect finds it either way).
                Pass an int for a fixed rate with no probing/switching.
    target_baud: session baud rate to upgrade to in "auto" mode (0/None to
                only detect and stay at the detected rate).
    stopbits  : delivery default of the RS232 option is 2; forcing the DIP
                switches 1-5 to ON resets the interface to 9600,N,8,1.
                1 usually works either way - change if you get garbage.
    timeout   : read timeout in seconds for one response line.
    auto_check: if True, the STB error bits are checked after every set
                command and raise EACSCommandError (extra traffic).
    """

    #: Probe order for baudrate="auto". Highest first for safety: probing a
    #: rate LOWER than the device's smears every sent byte into up to ~12
    #: garbage bytes device-side, which can randomly form valid (state
    #: changing!) commands. High-to-low probing yields at most 1-2 garbage
    #: bytes, discarded by the ESC flush. 115200 also is where a previous
    #: auto session left the device; 9600 is the delivery default.
    BAUD_CANDIDATES = (115200, 9600, 57600, 38400, 19200)

    def __init__(self, port: str, baudrate: int | str = "auto",
                 target_baud: int | None = 115200, parity: str = "N",
                 stopbits: float = 1, bytesize: int = 8,
                 timeout: float = 1.0, auto_check: bool = False):
        self.timeout = timeout
        self.auto_check = auto_check
        self._echo: bool | None = None      # unknown until first query
        self._unsupported: set[str] = set() # commands this firmware ignores
        self._seen_ok: set[str] = set()     # commands that answered once
        auto = isinstance(baudrate, str)
        if auto and baudrate.lower() != "auto":
            raise ValueError(f"baudrate must be an int or 'auto', not {baudrate!r}")
        self._ser = serial.Serial(
            port=port,
            baudrate=self.BAUD_CANDIDATES[0] if auto else int(baudrate),
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=0.05,           # short poll interval; we manage timing
            write_timeout=2.0,
        )
        # A power-up banner or stale bytes may sit in the buffer, and the
        # device side may hold a stale partial command - clear both.
        self._ser.reset_input_buffer()
        self._flush_device_input()
        #: baud rate the device was found at ("auto" mode only)
        self.detected_baud: int | None = None
        self._target_baud = target_baud if auto else None
        if auto:
            self.detected_baud = self._detect_baud()
            if target_baud and target_baud != self.detected_baud:
                self._upgrade_baud(target_baud)

    @property
    def active_baud(self) -> int:
        """Baud rate the connection is currently running at."""
        return self._ser.baudrate

    # ------------------------------------------------------------------ #
    # low level                                                          #
    # ------------------------------------------------------------------ #

    def close(self) -> None:
        if self._ser.is_open:
            self._ser.close()

    def __enter__(self) -> "EACS":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _write(self, cmd: str) -> None:
        self._ser.write(cmd.encode("ascii") + b"\r")

    def _read_line(self, timeout: float | None = None) -> str | None:
        """Read one line terminated by CR and/or LF. None on timeout."""
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        buf = bytearray()
        while time.monotonic() < deadline:
            b = self._ser.read(1)
            if not b:
                continue
            if b in (b"\r", b"\n"):
                if buf:
                    return buf.decode("latin-1")
                continue                    # swallow bare LF after CR etc.
            if b == b"\x00":
                continue                    # stray NUL, never part of the protocol
            buf += b
        return buf.decode("latin-1") if buf else None

    def _command(self, cmd: str) -> None:
        """Send a command that produces no response (except a possible echo)."""
        self._write(cmd)
        if self._echo is not False:
            # Consume the echoed command line if echo is on (or still unknown).
            line = self._read_line(0.3 if self._echo is None else self.timeout)
            if line is not None and line.strip().lower() == cmd.strip().lower():
                self._echo = True
            # anything else (None or unexpected) is ignored here
        if self.auto_check:
            self.check_error()

    def _query(self, cmd: str) -> str:
        """Send a command and return the response line (echo skipped)."""
        self._write(cmd)
        deadline = time.monotonic() + self.timeout + 0.5
        while time.monotonic() < deadline:
            line = self._read_line(max(0.05, deadline - time.monotonic()))
            if line is None:
                break
            if line.strip().lower() == cmd.strip().lower():
                self._echo = True           # that was the echo -> keep reading
                continue
            if self._echo is None:
                self._echo = False
            return line.strip()
        raise EACSTimeout(f"no response to {cmd!r}")

    def _query_value(self, cmd: str) -> float:
        """Query and parse the numeric payload after the first comma."""
        line = self._query(cmd)
        payload = line.split(",", 1)[1] if "," in line else line
        return _parse_number(payload)

    # ------------------------------------------------------------------ #
    # baud detection / session upgrade                                    #
    # ------------------------------------------------------------------ #

    def _flush_device_input(self) -> None:
        """Discard a possibly garbled partial command in the device's buffer.

        Probing at a wrong baud rate leaves garbage bytes in the device's
        command buffer which would corrupt the next real command. Per the
        manual, a command containing <ESC> is discarded at the terminator.
        """
        self._ser.write(b"\x1b\r")
        self._ser.flush()
        time.sleep(0.05)
        self._ser.reset_input_buffer()

    def _probe(self) -> bool:
        """True if the device answers an STB query at the current baud rate.

        At a wrong rate the device either stays silent or (echo on) sends
        back garbage bytes - neither yields a line starting with 'STB'.
        """
        self._flush_device_input()
        saved = self.timeout
        self.timeout = 0.6
        try:
            return self._query("STB").upper().startswith("STB")
        except (EACSError, EACSTimeout):
            return False
        finally:
            self.timeout = saved

    def _detect_baud(self, close_on_fail: bool = True) -> int:
        for baud in self.BAUD_CANDIDATES:
            self._ser.baudrate = baud
            self._ser.reset_input_buffer()
            self._echo = None           # re-detect echo per rate
            if self._probe():
                return baud
        port = self._ser.port
        if close_on_fail:
            self.close()
        raise EACSTimeout(
            f"no device answering on {port} at any of {self.BAUD_CANDIDATES} baud "
            "(check cable/port; a null-modem cable is required)")

    def redetect(self) -> int:
        """Re-run baud detection on the open port and return the active rate.

        For long-running applications: after the device is power cycled it
        falls back to its EEPROM baud rate and the session goes silent -
        call this to find it again (and re-upgrade to the target rate in
        "auto" mode). Raises EACSTimeout if the device stays silent; the
        port remains open so the call can be retried.
        """
        self.detected_baud = self._detect_baud(close_on_fail=False)
        if self._target_baud and self._target_baud != self.detected_baud:
            self._upgrade_baud(self._target_baud)
        return self.active_baud

    def _upgrade_baud(self, target: int) -> None:
        """Switch device + local port to `target` for this session (no SS).

        Falls back to the detected rate if the device does not answer at the
        new one.
        """
        old_baud = self._ser.baudrate
        echo = "E" if self._echo else "N"
        self._write(f"PC,{target},{self._ser.parity},{self._ser.bytesize},"
                    f"{int(self._ser.stopbits)},N,{echo}")
        self._ser.flush()
        time.sleep(0.3)                 # let the device switch over
        self._ser.baudrate = target
        self._ser.reset_input_buffer()
        if not self._probe():
            # device did not follow - drop back and keep working
            self._ser.baudrate = old_baud
            self._ser.reset_input_buffer()
            self._echo = None
            if not self._probe():
                self.close()
                raise EACSError(
                    f"device lost while switching {old_baud} -> {target} baud; "
                    "power cycle the unit to restore its saved interface settings")
            warnings.warn(f"baud upgrade to {target} failed, staying at {old_baud}")

    def raw(self, cmd: str, expect_response: bool = True) -> str | None:
        """Escape hatch: send any command string.

        Returns the response line, or None if the device stayed silent.
        """
        try:
            return self._query(cmd)
        except EACSTimeout:
            if expect_response:
                raise
            return None

    # ------------------------------------------------------------------ #
    # identification / remote control                                    #
    # ------------------------------------------------------------------ #

    def identify(self) -> str:
        """ID / *IDN? - identification string."""
        return self._query("ID")

    def version(self) -> str:
        """*OPT? - hardware/MCU/DSP/interface version."""
        return self._query("*OPT?")

    def remote(self, mode: int | None = None) -> None:
        """GTR - go to remote.

        mode=None  activate remote control now
        mode=0     no automatic switch to remote (manual + readout mode)
        mode=1     remote on first addressing (delivery state)
        mode=2     remote immediately after power-on, LOCAL blocked
        """
        self._command("GTR" if mode is None else f"GTR,{mode}")

    def local(self) -> None:
        """GTL - back to front panel operation (also clears LLO)."""
        self._command("GTL")

    def local_lockout(self, store: int | None = None) -> None:
        """LLO - disable the LOCAL button.  store: None, 0 or 1."""
        self._command("LLO" if store is None else f"LLO,{store}")

    # ------------------------------------------------------------------ #
    # set points                                                         #
    # ------------------------------------------------------------------ #

    def set_voltage_ac(self, volts) -> None:
        """UAC - AC output voltage (rms). Accepts number, '10%' or 'DEFAULT'."""
        self._command(f"UAC,{_to_param(volts)}")

    def get_voltage_ac(self) -> float:
        return self._query_value("UAC")

    def set_voltage_dc(self, volts) -> None:
        """UDC - DC offset voltage. Accepts number, '10%' or 'DEFAULT'."""
        self._command(f"UDC,{_to_param(volts)}")

    def get_voltage_dc(self) -> float:
        return self._query_value("UDC")

    def set_current_limit(self, amps) -> None:
        """IA - current limit (rms). Accepts number, '10%' or 'DEFAULT'."""
        self._command(f"IA,{_to_param(amps)}")

    def get_current_limit(self) -> float:
        return self._query_value("IA")

    def set_frequency(self, hz) -> None:
        """FA - output frequency in Hz."""
        self._command(f"FA,{_to_param(hz)}")

    def get_frequency(self) -> float:
        return self._query_value("FA")

    def set_frequency_default(self, hz=None) -> None:
        """FRQ - like FA; FRQ,DEFAULT stores current value as power-on default."""
        self._command("FRQ,DEFAULT" if hz is None else f"FRQ,{_to_param(hz)}")

    def set_phase(self, degrees) -> None:
        """PHA - activation phase angle 0.0..359.9 deg."""
        self._command(f"PHA,{_to_param(degrees)}")

    def get_phase(self) -> float:
        return self._query_value("PHA")

    # --- I-mode devices (current sources); no effect on U-mode units --- #

    def set_current_ac(self, amps) -> None:
        """IAC - AC output current (I-mode devices)."""
        self._command(f"IAC,{_to_param(amps)}")

    def set_current_dc(self, amps) -> None:
        """IDC - DC output current (I-mode devices)."""
        self._command(f"IDC,{_to_param(amps)}")

    def set_voltage_limit(self, volts) -> None:
        """UA - voltage limit (I-mode devices)."""
        self._command(f"UA,{_to_param(volts)}")

    # ------------------------------------------------------------------ #
    # output                                                             #
    # ------------------------------------------------------------------ #

    def output_on(self) -> None:
        """SB,R - release (enable) the output."""
        self._command("SB,R")

    def output_off(self) -> None:
        """SB,S - standby (disable) the output.

        CAUTION: in LOCAL mode this does not act immediately - it is LATCHED
        into the interface set-point state and applied when remote control
        activates (verified on HE-ACS V0.72). The same holds for output_on()
        and all set commands: GTR applies the whole latched interface state
        at once, including a latched output-ON. Check get_output() when the
        actual output state matters.
        """
        self._command("SB,S")

    def output_pulse(self, milliseconds: int) -> None:
        """SB,<t> - enable the output for t milliseconds (10..32000)."""
        if not 10 <= int(milliseconds) <= 32000:
            raise ValueError("pulse time must be 10..32000 ms")
        self._command(f"SB,{int(milliseconds)}")

    def get_output(self) -> bool:
        """True if the output is currently enabled (STATUS standby bit).

        Deliberately not the SB query: SB reports the *configured* standby
        setting and keeps saying standby while a timed pulse (SB,<ms>) has
        the output temporarily released; the STATUS bit tracks the truth.
        """
        return self.status().output_on

    # ------------------------------------------------------------------ #
    # waveform                                                           #
    # ------------------------------------------------------------------ #

    def set_waveform(self, wave) -> None:
        """WAVE,<nr> - select signal form (Waveform enum, int 0-7 or name).

        Blocks ~0.3 s: loading the wave table takes ~170 ms and the device
        drops commands (e.g. SB,R) and answers stale MWAVE values meanwhile.
        """
        if isinstance(wave, str):
            wave = Waveform[wave.upper()]
        self._command(f"WAVE,{int(wave)}")
        time.sleep(0.3)

    def get_waveform(self) -> str:
        """MWAVE - currently selected signal form (raw device answer)."""
        return self._query("MWAVE")

    def upload_wave(self, samples, dest: str = "OUT",
                    verify: bool = True) -> bool:
        """WAV - upload a user defined wave (3600 samples, -1.0 .. +1.0).

        dest: "OUT"  -> directly to the output table (volatile).
                        Direct mode (WAVE,7) is selected automatically.
              "MEM1".."MEM3" -> stored in internal memory.

        Sending 3600 lines takes ~30 s at 9600 baud - consider raising the
        baud rate (program_rs232) for frequent use.

        Returns True when the device confirmed the transfer (STATUS bit 4).
        Some firmware (e.g. HE-ACS V0.72) never sets that bit even though
        the upload works, so False does NOT mean failure - verify by
        enabling the output and measuring (e.g. crest factor MUS/MUA).
        """
        samples = list(samples)
        if len(samples) != 3600:
            raise ValueError(f"a wave needs exactly 3600 samples, got {len(samples)}")
        if any(s < -1.0 or s > 1.0 for s in samples):
            raise ValueError("samples must be within -1.0 .. +1.0")
        dest = dest.upper()
        if dest not in ("OUT", "MEM1", "MEM2", "MEM3"):
            raise ValueError("dest must be OUT, MEM1, MEM2 or MEM3")

        if dest == "OUT":
            self.set_waveform(Waveform.DIRECT)   # WAVE,7 first (manual)
        self._write(f"WAV,{dest}")
        # ~27 KB take ~30 s at 9600 baud: lift the write timeout and send in
        # chunks, discarding echoed bytes so the input buffer cannot overflow.
        saved_wto = self._ser.write_timeout
        self._ser.write_timeout = None
        try:
            chunk = 200
            for i in range(0, len(samples), chunk):
                data = "".join(f"{s:.4f}\r" for s in samples[i:i + chunk])
                self._ser.write(data.encode("ascii"))
                self._ser.flush()
                self._ser.reset_input_buffer()
        finally:
            self._ser.write_timeout = saved_wto
        # storing to MEM1..3 takes ~170 ms; give the unit a moment,
        # then drop whatever echo is still trailing in.
        time.sleep(0.5)
        self._ser.reset_input_buffer()
        if verify:
            return self.status().wave_ok
        return False

    # ------------------------------------------------------------------ #
    # measurements                                                       #
    # ------------------------------------------------------------------ #

    def measure_voltage(self) -> float:
        """MUA - output voltage, true rms incl. DC part [V]."""
        return self._query_value("MUA")

    def measure_voltage_dc(self) -> float:
        """MUDC - DC output voltage [V]."""
        return self._query_value("MUDC")

    def measure_voltage_peak(self) -> float:
        """MUS - peak output voltage [V]."""
        return self._query_value("MUS")

    def measure_current(self) -> float:
        """MIA - output current, true rms incl. DC part [A]."""
        return self._query_value("MIA")

    def measure_current_dc(self) -> float:
        """MIDC - DC output current [A]."""
        return self._query_value("MIDC")

    def measure_current_peak(self) -> float:
        """MIS - peak output current [A]."""
        return self._query_value("MIS")

    def measure_power(self) -> float:
        """MPA - active power [W]."""
        return self._query_value("MPA")

    def measure_power_apparent(self) -> float:
        """MPS - apparent power [VA]."""
        return self._query_value("MPS")

    def measure_power_reactive(self) -> float:
        """MPQ - reactive power [var]."""
        return self._query_value("MPQ")

    def measure_power_factor(self) -> float:
        """MPF - power factor P/(Ueff*Ieff)."""
        return self._query_value("MPF")

    def measure_frequency(self) -> float:
        """MFA - output frequency [Hz]."""
        return self._query_value("MFA")

    def measure_crest_voltage(self) -> float:
        """MCU - crest factor of the voltage (Umax/Ueff)."""
        return self._query_value("MCU")

    def measure_crest_current(self) -> float:
        """MCI - crest factor of the current (Imax/Ieff)."""
        return self._query_value("MCI")

    #: (label, command) pairs used by measurements()
    _MEAS_TABLE = (
        ("U_rms [V]", "MUA"),
        ("I_rms [A]", "MIA"),
        ("U_dc [V]", "MUDC"),
        ("I_dc [A]", "MIDC"),
        ("P [W]", "MPA"),
        ("S [VA]", "MPS"),      # not implemented on all firmware versions
        ("Q [var]", "MPQ"),     # not implemented on all firmware versions
        ("PF", "MPF"),
        ("f [Hz]", "MFA"),
    )

    def measurements(self) -> dict:
        """One snapshot of the most useful readings.

        Commands the firmware does not answer (e.g. MPS/MPQ on some units)
        are reported as None and skipped on subsequent calls; values the
        device cannot calculate (dashes, e.g. PF at no load) come back NaN.
        A command that answered before is never marked unsupported - a
        timeout then is transient (rms measurements take seconds at very
        low output frequencies) and yields None for this call only.
        """
        result = {}
        for label, cmd in self._MEAS_TABLE:
            if cmd in self._unsupported:
                result[label] = None
                continue
            try:
                result[label] = self._query_value(cmd)
                self._seen_ok.add(cmd)
            except EACSTimeout:
                if cmd not in self._seen_ok:
                    self._unsupported.add(cmd)
                result[label] = None
        return result

    # ------------------------------------------------------------------ #
    # limits / status                                                    #
    # ------------------------------------------------------------------ #

    def limits(self) -> dict:
        """Query the device's real adjustment limits (LIM* commands)."""
        lim = {
            "U_ac_max [V]": self._query_value("LIMUAC"),
            "I_max [A]": self._query_value("LIMIA"),
            "f_min [Hz]": self._query_value("LIMFMIN"),
            "f_max [Hz]": self._query_value("LIMFMAX"),
        }
        try:
            lim["U_dc_max [V]"] = self._query_value("LIMUDC")
        except (EACSTimeout, EACSError):
            pass                     # not available on I-mode devices
        return lim

    def status(self) -> Status:
        """STATUS - decoded 16-bit status word."""
        line = self._query("STATUS")
        bits = line.split(",", 1)[-1].strip()
        if len(bits) != 16 or set(bits) - {"0", "1"}:
            raise EACSError(f"unexpected STATUS answer: {line!r}")
        bit = lambda n: bits[15 - n] == "1"       # MSB is leftmost
        return Status(
            raw=bits,
            remote=bit(0),
            panel_locked=bit(1),
            standby=bit(3),
            wave_ok=bit(4),
            setpoint_active=bit(5),
            waveform_bits=(bit(8) << 0) | (bit(9) << 1) | (bit(10) << 2),
            current_limiting=bit(13),
            overload_warning=bit(14),
            overload_shutdown=bit(15),
        )

    def stb(self) -> str:
        """STB - raw interface status byte/word (bit string)."""
        line = self._query("STB")
        return line.split(",", 1)[-1].strip()

    def check_error(self) -> None:
        """Read STB and raise EACSCommandError if the error bits are set."""
        bits = self.stb()
        try:
            code = int(bits[-3:], 2)              # D2..D0
        except ValueError:
            raise EACSError(f"unexpected STB answer: {bits!r}")
        if code in STB_ERRORS:
            raise EACSCommandError(STB_ERRORS[code])

    def clear_status(self) -> None:
        """CLS - clear the status byte."""
        self._command("CLS")

    # ------------------------------------------------------------------ #
    # cycle mode / phase dip                                             #
    # ------------------------------------------------------------------ #

    def cycle_configure(self, t_on: int, t_off: int) -> None:
        """CYCLE,<Ton>,<Toff> - configure cyclic output switching (1-32767 s)."""
        if not (1 <= int(t_on) <= 32767 and 1 <= int(t_off) <= 32767):
            raise ValueError("cycle times must be 1..32767 s")
        self._command(f"CYCLE,{int(t_on)},{int(t_off)}")

    def cycle_start(self) -> None:
        self._command("CYCLE,S")

    def cycle_stop(self) -> None:
        self._command("CYCLE,R")

    def cycle_status(self) -> str:
        """CYCLE - 'CYCLE,<Ton>s,<Toff>s,<Tonrest>s,<Toffrest>s,{S|R}'."""
        return self._query("CYCLE")

    def dip_configure(self, milliseconds: int) -> None:
        """DIP,<t> - configure a short output interruption (max 30000 ms)."""
        if not 0 < int(milliseconds) <= 30000:
            raise ValueError("dip time must be 1..30000 ms")
        self._command(f"DIP,{int(milliseconds)}")

    def dip_start(self) -> None:
        """DIP,S - trigger the interruption at the next reference zero cross."""
        self._command("DIP,S")

    def dip_status(self) -> str:
        return self._query("DIP")

    # ------------------------------------------------------------------ #
    # setup / misc                                                       #
    # ------------------------------------------------------------------ #

    def save_setup(self) -> None:
        """SS - store port & interface parameters in the EEPROM."""
        self._command("SS")

    def reset(self) -> None:
        """RI / *RST - software reset of the instrument."""
        self._command("RI")

    def device_clear(self) -> None:
        """DCL - reset initialization data (also interface parameters!)."""
        self._command("DCL")

    def sync_input(self, enable: bool) -> None:
        """SYNC,S / SYNC,R - enable/disable the sync input (option)."""
        self._command(f"SYNC,{'S' if enable else 'R'}")

    def program_rs232(self, baud: int, parity: str = "N", databits: int = 8,
                      stopbits: int = 1, handshake: str = "N",
                      echo: str = "N", save: bool = True) -> None:
        """PC - reprogram the RS232 interface and follow with the new baud rate.

        CAUTION: takes effect immediately on the device. This method also
        switches the local serial port to the new settings and, if save=True,
        stores them in the EEPROM (SS).
        """
        self._command(f"PC,{baud},{parity},{databits},{stopbits},{handshake},{echo}")
        time.sleep(0.2)
        self._ser.baudrate = baud
        self._ser.parity = parity
        self._ser.bytesize = databits
        self._ser.stopbits = stopbits
        self._echo = (echo.upper() == "E")
        self._ser.reset_input_buffer()
        if save:
            self.save_setup()
