"""Offline regression suite: simulates an EAC-S on a fake serial port.

Run from the repository root:  python test_offline.py
No hardware needed - covers echo handling, queries, status decoding,
wave upload and the autobaud detect/upgrade logic.
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import time
import eac_s
from eac_s import EACS, Waveform


class FakeSerial:
    """Minimal pyserial stand-in emulating an EAC-S (per the manual)."""

    RESPONSES = {
        "ID": "EAC-S1000/RS232/V700 SN12.42.487",
        "*OPT?": "03/2013 V2.69.42.28.1",
        "UAC": "UAC,230.0V",
        "UDC": "UDC,0.0V",
        "IA": "IA,1.00A",
        "FA": "FA,50.0Hz",
        "PHA": "PHA,0.0",
        "SB": "SB,S",
        "MWAVE": "WAVE,1",
        "STATUS": "STATUS,0000001000011001",  # remote, standby, wave-ok set
        "STB": "STB,0000100000000000",
        "LIMUAC": "LIMUAC,700.0V",
        "LIMUDC": "LIMUDC,990.0V",
        "LIMIA": "LIMIA,5.00A",
        "LIMFMIN": "LIMFMIN,0.1Hz",
        "LIMFMAX": "LIMFMAX,500.0Hz",
        "MUA": "MUA,229.9V",
        "MIA": "MIA,0.567A",
        "MUDC": "MUDC,0.0V",
        "MIDC": "MIDC,0.01A",
        "MUS": "MUS,325.1V",
        "MIS": "MIS,0.80A",
        "MPA": "MPA,105.0W",
        "MPS": "MPS,131.2VA",
        "MPQ": "MPQ,78.7var",
        "MPF": "MPF,0.8000",
        "MFA": "MFA,50.0Hz",
        "CYCLE": "CYCLE,3s,5s,0s,0s,R",
        "DIP": "DIP,60ms",
    }
    NO_RESPONSE_PREFIXES = ("GTR", "GTL", "LLO", "SB,", "UAC,", "UDC,", "IA,",
                            "FA,", "FRQ,", "PHA,", "WAVE,", "WAV,", "CYCLE,",
                            "DIP,", "SS", "RI", "CLS", "DCL", "SYNC,", "PC,")

    def __init__(self, echo=True, device_baud=9600, **kwargs):
        self.echo = echo
        self.is_open = True
        self.write_timeout = 2.0
        self.port = "FAKE"
        self.parity = kwargs.get("parity", "N")
        self.bytesize = kwargs.get("bytesize", 8)
        self.stopbits = kwargs.get("stopbits", 1)
        self.baudrate = kwargs.get("baudrate", 9600)  # host-side setting
        self.device_baud = device_baud                # device-side setting
        self._rx = bytearray()   # bytes the driver will read
        self._cmd = bytearray()  # command being received
        self.log = []

    # --- pyserial API used by the driver ---
    def write(self, data):
        if self.baudrate != self.device_baud:
            # wrong rate: device sees garbage; echo returns garbage bytes
            if self.echo:
                self._rx += b"\xa5" * len(data)
            self._cmd.clear()
            return len(data)
        for i in range(len(data)):
            b = data[i:i+1]
            if self.echo:
                self._rx += b
            if b in (b"\r", b"\n"):
                self._handle(self._cmd.decode())
                self._cmd.clear()
            else:
                self._cmd += b
        return len(data)

    def flush(self):
        pass

    def read(self, n=1):
        time.sleep(0.001)
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def reset_input_buffer(self):
        self._rx.clear()

    def close(self):
        self.is_open = False

    # --- device model ---
    def _handle(self, cmd):
        if not cmd:
            return
        if "\x1b" in cmd or "\x7f" in cmd:
            return          # command aborted by ESC/DEL, per the manual
        self.log.append(cmd)
        up = cmd.upper()
        if up.startswith("PC,"):
            self.device_baud = int(up.split(",")[1])   # rate change, immediate
        elif up in self.RESPONSES:
            self._rx += (self.RESPONSES[up] + "\r\n").encode()
        elif up.startswith(self.NO_RESPONSE_PREFIXES) or up[0].isdigit() or up[0] in "+-0.":
            pass  # set command / wave sample: no reply
        else:
            print(f"  [fake device] unhandled command: {cmd!r}")


def run(echo):
    print(f"\n--- echo {'ON' if echo else 'OFF'} ---")
    fake = FakeSerial(echo=echo)
    eac_s.serial.Serial = lambda **kw: fake
    psu = EACS("FAKE", baudrate=9600, timeout=0.5)   # fixed rate path

    assert psu.identify() == "EAC-S1000/RS232/V700 SN12.42.487"
    print("identify ok:", psu.identify())
    print("version  ok:", psu.version())
    assert psu._echo is echo, f"echo detection failed: {psu._echo}"
    print("echo auto-detected:", psu._echo)

    psu.remote()
    psu.set_voltage_ac(230)
    psu.set_current_limit(1)
    psu.set_frequency(50)
    psu.set_waveform("sine")
    assert "UAC,230" in fake.log and "IA,1" in fake.log and "FA,50" in fake.log
    assert "WAVE,1" in fake.log
    print("set commands sent:", [c for c in fake.log if "," in c][-4:])

    assert psu.get_voltage_ac() == 230.0
    assert psu.get_current_limit() == 1.0
    assert psu.get_output() is False
    print("queries ok: UAC=230.0, IA=1.0, output off")

    st = psu.status()
    assert st.remote and st.standby and not st.output_on
    print("status decode ok:", st)

    lim = psu.limits()
    assert lim["U_ac_max [V]"] == 700.0 and lim["I_max [A]"] == 5.0
    print("limits ok:", lim)

    m = psu.measurements()
    assert m["PF"] == 0.8 and m["U_rms [V]"] == 229.9
    print("measurements ok:", m)

    psu.output_pulse(500)
    assert fake.log[-1] == "SB,500"
    psu.cycle_configure(3, 5)
    assert fake.log[-1] == "CYCLE,3,5"
    print("pulse/cycle ok")

    # wave upload: 3600 samples, STATUS bit4 is set in the fake status word
    assert psu.upload_wave([0.0] * 3600, dest="MEM1") is True
    print("wave upload ok (3600 samples, wave_ok confirmed)")

    assert psu.raw("STATUS").startswith("STATUS,")
    assert psu.raw("GTL", expect_response=False) is None
    print("raw passthrough ok")
    psu.close()


def run_autobaud(device_baud, echo=True):
    print(f"\n--- autobaud: device at {device_baud}, echo {'ON' if echo else 'OFF'} ---")
    def factory(**kw):
        return FakeSerial(echo=echo, device_baud=device_baud, **kw)
    eac_s.serial.Serial = factory
    psu = EACS("FAKE", timeout=0.5)              # default: auto + upgrade
    fake = psu._ser
    assert psu.detected_baud == device_baud, psu.detected_baud
    assert psu.active_baud == 115200, psu.active_baud
    assert fake.device_baud == 115200
    # still fully operational after the switch
    assert psu.identify().startswith("EAC-S1000")
    assert psu.get_voltage_ac() == 230.0
    print(f"detected at {psu.detected_baud}, upgraded to {psu.active_baud}, queries ok")
    psu.close()


def run_autobaud_no_upgrade():
    print("\n--- autobaud: target_baud=None (detect only) ---")
    eac_s.serial.Serial = lambda **kw: FakeSerial(echo=True, device_baud=9600, **kw)
    psu = EACS("FAKE", target_baud=None, timeout=0.5)
    assert psu.detected_baud == 9600 and psu.active_baud == 9600
    assert psu._ser.device_baud == 9600
    print("stays at detected 9600, ok")
    psu.close()


run(echo=True)
run(echo=False)
run_autobaud(9600)            # delivery state -> detect + upgrade
run_autobaud(115200)          # left at 115200 by a previous session
run_autobaud(9600, echo=False)
run_autobaud_no_upgrade()
print("\nALL TESTS PASSED")
