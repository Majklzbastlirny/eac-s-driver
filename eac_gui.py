"""
eac_gui.py - Browser GUI for ET System EAC-S AC power sources.

A small built-in web server (Python stdlib only) that serves a single-page
control panel and a JSON API on top of the eac_s driver.

    python eac_gui.py COM19                  # opens http://127.0.0.1:8432
    python eac_gui.py COM19 --host 0.0.0.0   # reachable from LAN/phone
    python eac_gui.py COM19 --port 9000 --no-browser

The server owns the serial connection; any number of browsers can watch,
every control action is executed under a lock.

SECURITY NOTE: there is no authentication. The default bind is localhost;
only use --host 0.0.0.0 on a network where everyone may control the PSU.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from eac_s import EACS, EACSError, EACSTimeout, Waveform
from eac_cli import WATCH_FIELDS

# ----------------------------------------------------------------------- #
# device access (single serial connection, one lock)                      #
# ----------------------------------------------------------------------- #

psu: EACS | None = None
lock = threading.Lock()
unsupported: set[str] = set()


def recovering(fn):
    """Run fn(); if the device has gone silent (e.g. power cycled back to
    its EEPROM baud rate), re-detect the baud once and retry."""
    try:
        return fn()
    except EACSTimeout:
        with lock:
            baud = psu.redetect()          # raises EACSTimeout if still dead
        print(f"device re-detected after silence, now at {baud} baud")
        return fn()

SETPOINTS = {          # GUI field -> (query cmd, set method name)
    "uac": "UAC",
    "udc": "UDC",
    "ia": "IA",
    "fa": "FA",
    "pha": "PHA",
}


def read_state() -> dict:
    with lock:
        st = psu.status()
        meas = {}
        for key, (cmd, _fmt) in WATCH_FIELDS.items():
            if cmd in unsupported:
                meas[key] = None
                continue
            try:
                val = psu._query_value(cmd)
            except EACSTimeout:
                unsupported.add(cmd)
                val = None
            except EACSError:
                val = None                    # garbled reply: blank this tick
            meas[key] = None if val is None or val != val else val
    flags = []
    if st.current_limiting:
        flags.append("I-LIMIT")
    if st.overload_warning:
        flags.append("OVERLOAD")
    if st.overload_shutdown:
        flags.append("SHUTDOWN")
    return {
        "remote": st.remote,
        "output": st.output_on,
        "wave": st.waveform.name,
        "flags": flags,
        "meas": meas,
    }


def read_setpoints() -> dict:
    out = {}
    with lock:
        for key, cmd in SETPOINTS.items():
            try:
                out[key] = psu._query_value(cmd)
            except EACSTimeout:
                raise                      # dead device: trigger recovery
            except EACSError:              # garbled single reply
                out[key] = None
    return out


def read_info() -> dict:
    with lock:
        ident = psu.identify()
        limits = psu.limits()
    return {
        "id": ident,
        "port": psu._ser.port,
        "baud": psu.active_baud,
        "limits": {
            "uac": limits.get("U_ac_max [V]"),
            "udc": limits.get("U_dc_max [V]"),
            "ia": limits.get("I_max [A]"),
            "fmin": limits.get("f_min [Hz]"),
            "fmax": limits.get("f_max [Hz]"),
        },
    }


def apply_set(field: str, value) -> dict:
    field = field.lower()
    with lock:
        if field == "wave":
            psu.set_waveform(str(value))
        elif field in SETPOINTS:
            psu._command(f"{SETPOINTS[field]},{float(value):g}")
        else:
            raise ValueError(f"unknown field {field!r}")
        remote = psu.status().remote
    return {"ok": True, "remote": remote}


def _settle(pred, timeout: float = 1.2):
    """Poll STATUS (caller holds the lock) until pred(status) or timeout.

    The device's status word lags a few hundred ms behind GTR/GTL/SB
    commands; reading immediately reports the pre-command state.
    """
    deadline = time.monotonic() + timeout
    while True:
        st = psu.status()
        if pred(st) or time.monotonic() >= deadline:
            return st
        time.sleep(0.1)


def apply_output(on: bool) -> dict:
    with lock:
        if on:
            psu.output_on()
        else:
            psu.output_off()
        st = _settle(lambda s: s.output_on == on)
    return {"output": st.output_on, "remote": st.remote}


def apply_mode(remote: bool) -> dict:
    with lock:
        if remote:
            psu.remote()
        else:
            psu.local()
        st = _settle(lambda s: s.remote == remote)
    return {"remote": st.remote, "output": st.output_on}


# ----------------------------------------------------------------------- #
# HTTP layer                                                              #
# ----------------------------------------------------------------------- #

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):               # keep the console quiet
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        try:
            if self.path in ("/", "/index.html"):
                body = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/api/state":
                self._json(recovering(read_state))
            elif self.path == "/api/setpoints":
                self._json(recovering(read_setpoints))
            elif self.path == "/api/info":
                self._json(recovering(read_info))
            else:
                self._json({"error": "not found"}, 404)
        except (EACSError, OSError) as exc:
            self._json({"error": str(exc)}, 500)

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(length) or b"{}")
            if self.path == "/api/set":
                self._json(recovering(lambda: apply_set(data["field"], data["value"])))
            elif self.path == "/api/output":
                self._json(recovering(lambda: apply_output(bool(data["on"]))))
            elif self.path == "/api/mode":
                self._json(recovering(lambda: apply_mode(bool(data["remote"]))))
            else:
                self._json({"error": "not found"}, 404)
        except (KeyError, ValueError) as exc:
            self._json({"error": f"bad request: {exc}"}, 400)
        except (EACSError, OSError) as exc:
            self._json({"error": str(exc)}, 500)


# ----------------------------------------------------------------------- #
# the page                                                                #
# ----------------------------------------------------------------------- #

PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>EAC-S Control</title>
<style>
:root {
  --page: #f9f9f7; --surface: #fcfcfb; --ink: #0b0b0b; --ink-2: #555550;
  --ink-3: #8a8a82; --line: rgba(11,11,11,0.10); --accent: #2563c4;
  --good: #0ca30c; --warning: #fab219; --serious: #ec835a; --critical: #d03b3b;
  --on-strong: #fff;
}
@media (prefers-color-scheme: dark) {
  :root {
    --page: #0d0d0d; --surface: #1a1a19; --ink: #ffffff; --ink-2: #b9b9b2;
    --ink-3: #7c7c75; --line: rgba(255,255,255,0.10); --accent: #6ba1e8;
  }
}
* { box-sizing: border-box; margin: 0; }
body {
  background: var(--page); color: var(--ink);
  font: 15px/1.45 system-ui, "Segoe UI", sans-serif;
  padding: 16px; max-width: 1080px; margin: 0 auto;
}
h1 { font-size: 17px; font-weight: 650; }
.sub { color: var(--ink-3); font-size: 12.5px; }
header { display: flex; flex-wrap: wrap; gap: 8px 16px; align-items: baseline;
         margin-bottom: 14px; }
header .spacer { flex: 1; }
.card { background: var(--surface); border: 1px solid var(--line);
        border-radius: 10px; padding: 14px 16px; margin-bottom: 12px; }
.row { display: flex; flex-wrap: wrap; gap: 10px; align-items: center; }
.badge { display: inline-flex; align-items: center; gap: 6px; padding: 3px 10px;
         border-radius: 999px; font-size: 12.5px; font-weight: 600;
         border: 1px solid var(--line); }
.badge .dot { width: 8px; height: 8px; border-radius: 50%; background: var(--ink-3); }
.badge.live .dot { background: var(--critical); }
.badge.ok .dot { background: var(--good); }
.badge.warn { border-color: var(--warning); }
.flag { color: var(--ink); font-weight: 700; font-size: 12px; padding: 2px 8px;
        border-radius: 5px; border: 1.5px solid; }
.flag.ilimit { border-color: var(--warning); }
.flag.overload { border-color: var(--serious); }
.flag.shutdown { border-color: var(--critical); }
.tiles { display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr));
         gap: 10px; }
.tile { background: var(--surface); border: 1px solid var(--line);
        border-radius: 10px; padding: 10px 12px; }
.tile .lbl { font-size: 11.5px; color: var(--ink-3); text-transform: uppercase;
             letter-spacing: 0.4px; }
.tile .val { font-size: 26px; font-weight: 650; font-variant-numeric: tabular-nums;
             line-height: 1.15; white-space: nowrap; }
.tile .val small { font-size: 14px; color: var(--ink-2); font-weight: 500; }
.tile.minor .val { font-size: 17px; }
button {
  font: inherit; font-weight: 600; color: var(--ink);
  background: var(--surface); border: 1px solid var(--line);
  border-radius: 8px; padding: 7px 14px; cursor: pointer; min-height: 38px;
}
button:hover { border-color: var(--ink-3); }
button.primary { background: var(--accent); border-color: var(--accent); color: #fff; }
button.danger { background: var(--critical); border-color: var(--critical); color: #fff; }
button.sel { outline: 2px solid var(--accent); outline-offset: -2px; }
button:disabled { opacity: 0.45; cursor: default; }
input[type=number] {
  font: inherit; font-variant-numeric: tabular-nums; color: var(--ink);
  background: var(--page); border: 1px solid var(--line); border-radius: 8px;
  padding: 7px 10px; width: 110px;
}
.setrow { display: grid; grid-template-columns: 150px 160px auto 1fr;
          gap: 8px; align-items: center; padding: 4px 0; }
.setrow .cur { color: var(--ink-3); font-size: 12.5px; font-variant-numeric: tabular-nums; }
.setrow label { font-size: 13.5px; color: var(--ink-2); }
.inwrap { display: flex; align-items: center; gap: 6px; }
.inwrap .unit { color: var(--ink-2); font-size: 13px; min-width: 24px; }
#banner { display: none; border: 1.5px solid var(--warning); border-radius: 8px;
          padding: 8px 12px; margin-bottom: 12px; font-size: 13.5px; }
#banner.error { border-color: var(--critical); }
@media (max-width: 560px) {
  .setrow { grid-template-columns: 1fr 1fr; }
  .setrow .cur { grid-column: 2; text-align: right; }
}
</style>
</head>
<body>
<header>
  <h1>EAC-S Control</h1>
  <span class="sub" id="devid">connecting…</span>
  <span class="spacer"></span>
  <span class="badge" id="mode"><span class="dot"></span><span id="modetxt">–</span></span>
  <span class="badge" id="outbadge"><span class="dot"></span><span id="outtxt">–</span></span>
  <span id="flags"></span>
</header>

<div id="banner"></div>

<div class="card">
  <div class="row">
    <button id="btn-remote">Remote</button>
    <button id="btn-local">Local (panel)</button>
    <span class="spacer" style="flex:1"></span>
    <button id="btn-on" class="danger">Output ON</button>
    <button id="btn-off" class="primary">Output OFF</button>
  </div>
</div>

<div class="tiles" id="tiles-main">
  <div class="tile"><div class="lbl">U rms</div><div class="val"><span id="m-u">–</span> <small>V</small></div></div>
  <div class="tile"><div class="lbl">U dc</div><div class="val"><span id="m-udc">–</span> <small>V</small></div></div>
  <div class="tile"><div class="lbl">I rms</div><div class="val"><span id="m-i">–</span> <small>A</small></div></div>
  <div class="tile"><div class="lbl">Power</div><div class="val"><span id="m-p">–</span> <small>W</small></div></div>
  <div class="tile"><div class="lbl">Frequency</div><div class="val"><span id="m-f">–</span> <small>Hz</small></div></div>
  <div class="tile"><div class="lbl">Power factor</div><div class="val"><span id="m-pf">–</span></div></div>
</div>
<div class="tiles" style="margin-top:10px">
  <div class="tile minor"><div class="lbl">U peak</div><div class="val"><span id="m-upk">–</span> <small>V</small></div></div>
  <div class="tile minor"><div class="lbl">I dc</div><div class="val"><span id="m-idc">–</span> <small>A</small></div></div>
  <div class="tile minor"><div class="lbl">I peak</div><div class="val"><span id="m-ipk">–</span> <small>A</small></div></div>
  <div class="tile minor"><div class="lbl">Crest U</div><div class="val"><span id="m-cfu">–</span></div></div>
  <div class="tile minor"><div class="lbl">Crest I</div><div class="val"><span id="m-cfi">–</span></div></div>
  <div class="tile minor"><div class="lbl">Waveform</div><div class="val" style="font-size:17px" id="m-wave">–</div></div>
</div>

<div class="card" style="margin-top:12px">
  <div class="sub" style="margin-bottom:6px">SET POINTS (interface)</div>
  <div class="setrow"><label>AC voltage</label>
    <div class="inwrap"><input type="number" id="sp-uac" step="0.1" min="0"><span class="unit">V</span></div>
    <button data-set="uac">Apply</button>
    <span class="cur" id="cur-uac"></span></div>
  <div class="setrow"><label>DC offset</label>
    <div class="inwrap"><input type="number" id="sp-udc" step="0.1"><span class="unit">V</span></div>
    <button data-set="udc">Apply</button>
    <span class="cur" id="cur-udc"></span></div>
  <div class="setrow"><label>Current limit</label>
    <div class="inwrap"><input type="number" id="sp-ia" step="0.01" min="0"><span class="unit">A</span></div>
    <button data-set="ia">Apply</button>
    <span class="cur" id="cur-ia"></span></div>
  <div class="setrow"><label>Frequency</label>
    <div class="inwrap"><input type="number" id="sp-fa" step="0.1" min="0"><span class="unit">Hz</span></div>
    <button data-set="fa">Apply</button>
    <span class="cur" id="cur-fa"></span></div>
  <div class="setrow"><label>Phase</label>
    <div class="inwrap"><input type="number" id="sp-pha" step="0.1" min="0" max="359.9"><span class="unit">&deg;</span></div>
    <button data-set="pha">Apply</button>
    <span class="cur" id="cur-pha"></span></div>
  <div class="row" style="margin-top:8px">
    <span class="sub" style="min-width:142px">Waveform</span>
    <button data-wave="SINE">Sine</button>
    <button data-wave="SQUARE">Square</button>
    <button data-wave="TRIANGLE">Triangle</button>
    <button data-wave="MEMORY1">Mem 1</button>
    <button data-wave="MEMORY2">Mem 2</button>
    <button data-wave="MEMORY3">Mem 3</button>
  </div>
</div>

<script>
"use strict";
const $ = id => document.getElementById(id);
const UNITS = { uac: "V", udc: "V", ia: "A", fa: "Hz", pha: "°" };
let limits = {}, lastSp = {};

function banner(msg, error) {
  const b = $("banner");
  if (!msg) { b.style.display = "none"; return; }
  b.textContent = msg;
  b.className = error ? "error" : "";
  b.style.display = "block";
}

async function api(path, body) {
  const opt = body ? { method: "POST", body: JSON.stringify(body) } : {};
  const res = await fetch(path, opt);
  const data = await res.json();
  if (!res.ok) throw new Error(data.error || res.statusText);
  return data;
}

function fmt(v, digits) {
  return v === null || v === undefined ? "–" : v.toFixed(digits);
}

function render(s) {
  $("modetxt").textContent = s.remote ? "REMOTE" : "LOCAL";
  $("mode").className = "badge " + (s.remote ? "ok" : "");
  $("outtxt").textContent = s.output ? "OUTPUT ON" : "standby";
  $("outbadge").className = "badge " + (s.output ? "live" : "");
  $("flags").innerHTML = s.flags.map(f =>
    `<span class="flag ${f.replace("-","").toLowerCase()}">&#9888; ${f}</span>`).join(" ");
  const m = s.meas;
  $("m-u").textContent   = fmt(m.u, 1);
  $("m-udc").textContent = m.udc === null ? "–" : (m.udc >= 0 ? "+" : "") + m.udc.toFixed(1);
  $("m-i").textContent   = fmt(m.i, 3);
  $("m-p").textContent   = fmt(m.p, 1);
  $("m-f").textContent   = fmt(m.f, 1);
  $("m-pf").textContent  = fmt(m.pf, 4);
  $("m-upk").textContent = fmt(m.upk, 1);
  $("m-idc").textContent = fmt(m.idc, 3);
  $("m-ipk").textContent = fmt(m.ipk, 3);
  $("m-cfu").textContent = fmt(m.cfu, 3);
  $("m-cfi").textContent = fmt(m.cfi, 3);
  $("m-wave").textContent = s.wave.toLowerCase();
  document.querySelectorAll("[data-wave]").forEach(b =>
    b.classList.toggle("sel", b.dataset.wave === s.wave));
  if (!s.remote)
    banner("Unit is in LOCAL mode - set commands are ignored by the device. " +
           "Click Remote to take control.");
  else banner(null);
}

async function poll() {
  try {
    render(await api("/api/state"));
  } catch (e) {
    banner("Connection problem: " + e.message, true);
  }
  setTimeout(poll, document.hidden ? 2000 : 500);
}

async function refreshSetpoints() {
  try {
    lastSp = await api("/api/setpoints");
    for (const k of ["uac", "udc", "ia", "fa", "pha"]) {
      $("cur-" + k).textContent =
        lastSp[k] === null ? "" : `device: ${lastSp[k]} ${UNITS[k]}`;
      if (document.activeElement !== $("sp-" + k) && lastSp[k] !== null)
        $("sp-" + k).value = lastSp[k];
    }
  } catch (e) { /* transient */ }
}

async function doSet(field, value) {
  try {
    const r = await api("/api/set", { field, value });
    if (!r.remote)
      banner("Sent, but the unit is in LOCAL mode - it ignored the command.");
    await refreshSetpoints();
  } catch (e) { banner("Set failed: " + e.message, true); }
}

document.querySelectorAll("[data-set]").forEach(b => {
  const f = b.dataset.set;
  b.onclick = () => {
    const v = parseFloat($("sp-" + f).value);
    if (!isNaN(v)) doSet(f, v);
  };
  $("sp-" + f).addEventListener("keydown", e => {
    if (e.key === "Enter") { e.preventDefault(); b.click(); }
  });
});
document.querySelectorAll("[data-wave]").forEach(b => b.onclick = () =>
  doSet("wave", b.dataset.wave));

$("btn-remote").onclick = () => api("/api/mode", { remote: true })
  .then(refreshSetpoints).catch(e => banner(e.message, true));
$("btn-local").onclick = () => api("/api/mode", { remote: false })
  .catch(e => banner(e.message, true));
$("btn-off").onclick = () => api("/api/output", { on: false })
  .then(r => { if (r.output) banner("Output is STILL ON - unit is in LOCAL mode. " +
    "Click Remote first (that alone applies interface set points and standby).", true); })
  .catch(e => banner(e.message, true));
$("btn-on").onclick = () => {
  const msg = `Enable the output?\n\nAC ${lastSp.uac ?? "?"} V   DC ${lastSp.udc ?? "?"} V` +
              `\nLimit ${lastSp.ia ?? "?"} A   ${lastSp.fa ?? "?"} Hz`;
  if (confirm(msg))
    api("/api/output", { on: true }).catch(e => banner(e.message, true));
};

api("/api/info").then(info => {
  $("devid").textContent =
    `${info.id} - ${info.port} @ ${info.baud} Bd - limits ${info.limits.uac} V / ${info.limits.ia} A`;
  limits = info.limits;
  $("sp-uac").max = info.limits.uac;
  $("sp-ia").max = info.limits.ia;
  $("sp-fa").min = info.limits.fmin; $("sp-fa").max = info.limits.fmax;
  if (info.limits.udc) { $("sp-udc").min = -info.limits.udc; $("sp-udc").max = info.limits.udc; }
}).catch(e => banner(e.message, true));

refreshSetpoints();
setInterval(refreshSetpoints, 5000);
poll();
</script>
</body>
</html>
"""


# ----------------------------------------------------------------------- #
# entry point                                                             #
# ----------------------------------------------------------------------- #

def main() -> None:
    parser = argparse.ArgumentParser(description="Browser GUI for EAC-S sources.")
    parser.add_argument("serial_port", help="serial port, e.g. COM3 or /dev/ttyUSB0")
    parser.add_argument("--baud", default="auto",
                        help="'auto' (default) or a fixed rate like 9600")
    parser.add_argument("--host", default="127.0.0.1",
                        help="bind address (0.0.0.0 exposes the GUI to your network!)")
    parser.add_argument("--port", type=int, default=8432, help="HTTP port")
    parser.add_argument("--no-browser", action="store_true",
                        help="don't open the browser automatically")
    args = parser.parse_args()

    global psu
    baud = args.baud if str(args.baud).lower() == "auto" else int(args.baud)
    try:
        psu = EACS(args.serial_port, baudrate=baud)
    except Exception as exc:
        sys.exit(f"cannot open {args.serial_port}: {exc}")
    print(f"device: {psu.identify()} at {psu.active_baud} baud")

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    url = f"http://{'127.0.0.1' if args.host == '0.0.0.0' else args.host}:{args.port}"
    print(f"GUI: {url}  (Ctrl+C stops)")
    if args.host == "0.0.0.0":
        print("NOTE: no authentication - anyone on your network can control the PSU.")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down")
    finally:
        server.server_close()
        psu.close()


if __name__ == "__main__":
    main()
