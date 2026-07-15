"""
eac_cli.py - Control console for ET System EAC-S AC power sources.

One-shot usage:
    python eac_cli.py COM3 id
    python eac_cli.py COM3 u 230
    python eac_cli.py COM3 on
    python eac_cli.py COM3 meas
    python eac_cli.py COM3 off

Interactive console:
    python eac_cli.py COM3

List serial ports:
    python eac_cli.py --list

Options:
    --baud 9600   --stopbits 1|2   --parity N|E|O   --timeout 1.0
"""

from __future__ import annotations

import argparse
import sys
import time

from eac_s import EACS, EACSError, EACSTimeout, Waveform

HELP = """\
Set points                        Output
  u [V|x%]     AC voltage (UAC)     on           enable output (SB,R)
  udc [V]      DC offset (UDC)      off          disable output (SB,S)
  i [A|x%]     current limit (IA)   pulse <ms>   output on for <ms> (10-32000)
  f [Hz]       frequency (FA)
  pha [deg]    phase angle (PHA)  Waveform
                                    wave [sine|square|triangle|external|
Readings                                  mem1|mem2|mem3|0-7]
  meas         one measurement snapshot
  mon [sec]    live monitor table, Ctrl+C stops (default 1 s)
  watch [sec] [fields|all]   mirror the device state live; state changes
               are kept as history lines, Ctrl+C stops. Default columns:
               u,udc,i,p,f - pick your own (e.g. 'watch 0.2 u,i,pf,cfu')
               or 'watch all'. Fields: u udc upk i idc ipk p s q pf cfu cfi f
  status       decoded STATUS word
  limits       device adjustment limits (LIM*)

Cycle / dip
  cycle <ton> <toff>   configure cyclic switching [s]
  cycle start|stop|?   control / query cycle mode
  dip <ms> | dip start | dip ?      short output interruption

Device
  id           identification       remote [0|1|2]   go to remote (GTR)
  ver          firmware (*OPT?)     local            front panel (GTL)
  save         save setup (SS)      reset            instrument reset (RI)
  raw <cmd>    send anything, print reply (if any)

  help         this text            quit / exit      leave console
Commands without a value print the current set point (e.g. just 'u').
"""

WAVE_ALIASES = {
    "sine": Waveform.SINE, "sin": Waveform.SINE,
    "square": Waveform.SQUARE, "rect": Waveform.SQUARE,
    "triangle": Waveform.TRIANGLE, "tri": Waveform.TRIANGLE,
    "external": Waveform.EXTERNAL, "ext": Waveform.EXTERNAL,
    "mem1": Waveform.MEMORY1, "mem2": Waveform.MEMORY2, "mem3": Waveform.MEMORY3,
    "direct": Waveform.DIRECT,
}


def fmt_value(val) -> str:
    """None -> not supported by this firmware, NaN -> device shows dashes."""
    if val is None:
        return "n/a"
    if val != val:                       # NaN
        return "-"
    return f"{val:g}"


def print_meas(psu: EACS) -> None:
    m = psu.measurements()
    width = max(len(k) for k in m)
    for key, val in m.items():
        print(f"  {key:<{width}} : {fmt_value(val)}")


def monitor(psu: EACS, interval: float) -> None:
    print("time      U_rms[V]   I_rms[A]     P[W]     PF      f[Hz]   (Ctrl+C stops)")
    try:
        while True:
            u = psu.measure_voltage()
            i = psu.measure_current()
            p = psu.measure_power()
            pf = psu.measure_power_factor()
            f = psu.measure_frequency()
            stamp = time.strftime("%H:%M:%S")
            pf_s = "  -  " if pf != pf else f"{pf:5.3f}"
            print(f"{stamp}  {u:8.2f}   {i:8.3f}   {p:8.1f}   {pf_s}   {f:6.2f}")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nmonitor stopped")


#: watch columns: field key -> (query command, format string)
WATCH_FIELDS = {
    "u":   ("MUA",  "{:7.1f} V"),
    "udc": ("MUDC", "{:+6.1f} Vdc"),
    "upk": ("MUS",  "{:6.1f} Vpk"),
    "i":   ("MIA",  "{:6.3f} A"),
    "idc": ("MIDC", "{:+6.3f} Adc"),
    "ipk": ("MIS",  "{:6.3f} Apk"),
    "p":   ("MPA",  "{:7.1f} W"),
    "s":   ("MPS",  "{:7.1f} VA"),
    "q":   ("MPQ",  "{:7.1f} var"),
    "pf":  ("MPF",  "PF {:6.4f}"),
    "cfu": ("MCU",  "CFu {:5.3f}"),
    "cfi": ("MCI",  "CFi {:5.3f}"),
    "f":   ("MFA",  "{:6.1f} Hz"),
}
WATCH_DEFAULT = ("u", "udc", "i", "p", "f")


def watch_line(psu: EACS, keys=WATCH_DEFAULT, unsupported: set | None = None,
               seen_ok: set | None = None) -> tuple[tuple, str]:
    """One state sample -> (change-key, display line).

    Values the device cannot calculate (dashes) show as '-', commands this
    firmware lacks (learned via `unsupported`) show as 'n/a'. A command that
    answered before is never marked unsupported - a timeout then is
    transient (rms readings take seconds at very low output frequencies).
    """
    if unsupported is None:
        unsupported = set()
    if seen_ok is None:
        seen_ok = set()
    st = psu.status()
    flags = []
    if st.current_limiting:
        flags.append("I-LIMIT")
    if st.overload_warning:
        flags.append("OVERLOAD")
    if st.overload_shutdown:
        flags.append("SHUTDOWN")
    parts = [
        time.strftime("%H:%M:%S"),
        f"{'REMOTE' if st.remote else 'LOCAL':<6}",
        f"{'OUTPUT ON' if st.output_on else 'standby':<9}",
        f"{st.waveform.name.lower():<9}",
    ]
    for key in keys:
        cmd, fmt = WATCH_FIELDS[key]
        width = len(fmt.format(0.0))
        if cmd in unsupported:
            parts.append("n/a".rjust(width))
            continue
        try:
            val = psu._query_value(cmd)
            seen_ok.add(cmd)
        except EACSTimeout:
            if cmd in seen_ok:              # transient (slow measurement)
                parts.append("?".rjust(width))
            else:
                unsupported.add(cmd)
                parts.append("n/a".rjust(width))
            continue
        except EACSError:                   # garbled reply: transient, retry next tick
            parts.append("?".rjust(width))
            continue
        parts.append(fmt.format(val) if val == val else "-".rjust(width))
    line = "  ".join(parts)
    if flags:
        line += "  [" + " ".join(flags) + "]"
    return (st.remote, st.output_on, st.waveform_bits, bool(flags)), line


def watch(psu: EACS, interval: float, keys=None) -> None:
    """Live mirror of the device state; new line on every state change."""
    if keys is None:
        keys = WATCH_DEFAULT
    bad = [k for k in keys if k not in WATCH_FIELDS]
    if bad:
        print(f"unknown field(s) {','.join(bad)} - valid: "
              f"{','.join(WATCH_FIELDS)} (or 'all')")
        return
    print("watching device state - Ctrl+C stops")
    last_key, width, unsupported, seen_ok = None, 0, set(), set()
    try:
        while True:
            key, line = watch_line(psu, keys, unsupported, seen_ok)
            if last_key is not None and key != last_key:
                sys.stdout.write("\n")          # keep the old state as history
            last_key = key
            width = max(width, len(line))
            sys.stdout.write("\r" + line.ljust(width))
            sys.stdout.flush()
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nwatch stopped")


def setpoint(psu: EACS, args: list[str],
             setter, getter, unit: str, name: str) -> None:
    """Handle 'u', 'i', 'f', ... : set when a value is given, else query.

    After setting, the value is read back: the device silently ignores
    out-of-range values (only a range-error bit is set), so the read-back
    shows what was actually accepted.
    """
    if args:
        setter(args[0])
        try:
            actual = getter()
        except EACSError:
            print(f"{name} set to {args[0]} (read-back failed)")
            return
        print(f"{name} = {actual:g} {unit}")
        try:
            requested = float(args[0].rstrip("%"))
        except ValueError:
            return                      # 'DEFAULT' etc. - nothing to compare
        if not args[0].endswith("%") and abs(actual - requested) > 0.5:
            print(f"  note: device did not accept {args[0]} (out of range?)")
    else:
        print(f"{name}: {getter():g} {unit}")


def execute(psu: EACS, tokens: list[str]) -> bool:
    """Run one console command. Returns False when the console should exit."""
    cmd, *args = tokens
    cmd = cmd.lower()

    if cmd in ("quit", "exit", "q"):
        try:
            if psu.get_output():
                print("note: the OUTPUT IS STILL ON ('off' disables it).")
        except EACSError:
            pass
        return False

    elif cmd in ("help", "h", "?"):
        print(HELP)

    elif cmd == "id":
        print(psu.identify())
    elif cmd == "ver":
        print(psu.version())
    elif cmd == "remote":
        if args:
            psu.remote(int(args[0]))
            print(f"GTR,{args[0]} sent")
        else:
            # Commands sent during LOCAL latch into the interface state and
            # GTR applies them all at once - latch standby first so remote
            # control always starts with the output off.
            psu.output_off()
            psu.remote()
            print("remote control active (output starts in standby)")
    elif cmd == "local":
        psu.local()
        print("front panel control active")

    elif cmd == "u":
        setpoint(psu, args, psu.set_voltage_ac, psu.get_voltage_ac, "V", "U_ac")
    elif cmd == "udc":
        setpoint(psu, args, psu.set_voltage_dc, psu.get_voltage_dc, "V", "U_dc")
    elif cmd == "i":
        setpoint(psu, args, psu.set_current_limit, psu.get_current_limit, "A", "I_limit")
    elif cmd == "f":
        setpoint(psu, args, psu.set_frequency, psu.get_frequency, "Hz", "f")
    elif cmd == "pha":
        setpoint(psu, args, psu.set_phase, psu.get_phase, "deg", "phase")

    elif cmd == "wave":
        if not args:
            print(psu.get_waveform())
        else:
            arg = args[0].lower()
            wave = WAVE_ALIASES.get(arg)
            if wave is None:
                try:
                    wave = Waveform(int(arg))
                except ValueError:
                    print(f"unknown waveform {args[0]!r}; try sine/square/triangle/ext/mem1-3 or 0-7")
                    return True
            psu.set_waveform(wave)
            print(f"waveform set to {Waveform(wave).name}")

    elif cmd == "on":
        try:
            u = psu.get_voltage_ac()
            i = psu.get_current_limit()
            f = psu.get_frequency()
            print(f"enabling output: {u:g} V, {i:g} A limit, {f:g} Hz")
        except EACSError:
            print("enabling output")
        psu.output_on()
    elif cmd == "off":
        psu.output_off()
        try:
            still_on = psu.get_output()
        except EACSError:
            still_on = False
        if still_on:
            print("WARNING: output is STILL ON - the unit is in LOCAL mode; the")
            print("panel controls it. Type 'remote' (enters standby) or use the panel.")
        else:
            print("output disabled (standby)")
    elif cmd == "pulse":
        if not args:
            print("usage: pulse <ms>   (10-32000)")
        else:
            psu.output_pulse(int(args[0]))
            print(f"output pulsed for {args[0]} ms")

    elif cmd == "meas":
        print_meas(psu)
    elif cmd == "mon":
        monitor(psu, float(args[0]) if args else 1.0)
    elif cmd == "watch":
        interval, keys = 0.5, None
        for a in args:
            try:
                interval = float(a)
                continue
            except ValueError:
                pass
            keys = list(WATCH_FIELDS) if a.lower() == "all" \
                else [k.strip() for k in a.lower().split(",") if k.strip()]
        watch(psu, interval, keys)
    elif cmd == "status":
        print(psu.status())
    elif cmd == "limits":
        for key, val in psu.limits().items():
            print(f"  {key:<13} : {val:g}")

    elif cmd == "cycle":
        if not args or args[0] == "?":
            print(psu.cycle_status())
        elif args[0].lower() == "start":
            psu.cycle_start()
            print("cycle mode started")
        elif args[0].lower() == "stop":
            psu.cycle_stop()
            print("cycle mode stopped")
        elif len(args) == 2:
            psu.cycle_configure(int(args[0]), int(args[1]))
            print(f"cycle configured: {args[0]} s on / {args[1]} s off ('cycle start' begins)")
        else:
            print("usage: cycle <ton> <toff> | cycle start | cycle stop | cycle ?")

    elif cmd == "dip":
        if not args or args[0] == "?":
            print(psu.dip_status())
        elif args[0].lower() == "start":
            psu.dip_start()
            print("dip triggered")
        else:
            psu.dip_configure(int(args[0]))
            print(f"dip time set to {args[0]} ms ('dip start' triggers it)")

    elif cmd == "save":
        psu.save_setup()
        print("setup saved to EEPROM")
    elif cmd == "reset":
        psu.reset()
        print("instrument reset sent")

    elif cmd == "raw":
        if not args:
            print("usage: raw <command string>")
        else:
            reply = psu.raw(" ".join(args), expect_response=False)
            print(reply if reply is not None else "(no response)")

    else:
        print(f"unknown command {cmd!r} - type 'help'")
    return True


def console(psu: EACS) -> None:
    print("EAC-S control console - type 'help' for commands, 'quit' to exit.")
    try:
        print(f"connected to: {psu.identify()}")
    except EACSTimeout:
        print("warning: device did not answer the ID query - check port/baud/cable.")
    while True:
        try:
            line = input("EAC-S> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        try:
            if not execute(psu, line.split()):
                break
        except EACSTimeout as exc:
            print(f"timeout: {exc}")
        except (EACSError, ValueError) as exc:
            print(f"error: {exc}")


def list_ports() -> None:
    from serial.tools import list_ports as lp
    ports = lp.comports()
    if not ports:
        print("no serial ports found")
    for p in ports:
        print(f"  {p.device:<8} {p.description}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Control console for ET System EAC-S AC sources.",
        epilog="Run without a command for the interactive console.")
    parser.add_argument("port", nargs="?", help="serial port, e.g. COM3")
    parser.add_argument("command", nargs="*", help="one-shot command (same syntax as console)")
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("--baud", default="auto",
                        help="'auto' (default): find the device and switch the session "
                             "to 115200; or a fixed rate like 9600 (no switching)")
    parser.add_argument("--stopbits", type=float, default=1, choices=[1, 2])
    parser.add_argument("--parity", default="N", choices=["N", "E", "O"])
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()

    if args.list:
        list_ports()
        return
    if not args.port:
        parser.error("a serial port is required (or use --list)")

    baud = args.baud if args.baud.lower() == "auto" else int(args.baud)
    try:
        psu = EACS(args.port, baudrate=baud, parity=args.parity,
                   stopbits=args.stopbits, timeout=args.timeout)
    except Exception as exc:
        sys.exit(f"cannot open {args.port}: {exc}")
    if psu.detected_baud is not None:
        note = "" if psu.active_baud == psu.detected_baud \
            else f", session switched to {psu.active_baud}"
        print(f"device found at {psu.detected_baud} baud{note}")

    with psu:
        if args.command:
            try:
                execute(psu, args.command)
            except (EACSError, ValueError) as exc:
                sys.exit(f"error: {exc}")
        else:
            console(psu)


if __name__ == "__main__":
    main()
