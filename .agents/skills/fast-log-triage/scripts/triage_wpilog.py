#!/usr/bin/env python3
"""Fast triage of Spectrum 3847 robot WPILOG files.

One pass over the file, a ranked list of likely causes with timestamps, and a list of
what looked clean. Built for the pit between matches: no Java, no WPILib install, no
AdvantageScope. Python 3.9+.

    python3 triage_wpilog.py <log.wpilog | dir> [more logs...] [--all] [--json]

Topic names are matched on the DogLog key (``Launcher/RPM``); the ``/Robot/`` prefix
DogLog adds is stripped. Detectors are heuristics tuned to this robot's keys; see
SKILL.md for what each one means and what to check on the robot.
"""

from __future__ import annotations

import argparse
import bisect
import json
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

# The report uses arrows and other non-ASCII glyphs. On Windows the console encoding is
# cp1252 and print() raised UnicodeEncodeError on the very first real match log
# (2026-09-19), so the output is forced to UTF-8 rather than depending on the locale.
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8", errors="replace")

from wpilog_fast import Entry, LogFile, decode, find_logs, iter_records, open_log, short_name

SEVERITY_ORDER = {"CRITICAL": 0, "HIGH": 1, "MEDIUM": 2, "LOW": 3}

# roboRIO 2 brownout threshold is 6.75 V; the RIO holds outputs off until it recovers.
BROWNOUT_V = 6.8
LOW_BATTERY_V = 9.0
LOW_BATTERY_HOLD_S = 0.5
TEMP_HIGH_C = 80.0
TEMP_WARM_C = 65.0
STATOR_HIGH_A = 80.0
STATOR_HOLD_S = 1.0
LOOP_MAX_MS = 60.0
OVERRUN_PCT = 5.0
CPU_PCT = 92.0
MEM_LOW_MB = 25.0
GAP_S = 0.3
POSE_JUMP_M = 1.0
POSE_JUMP_WINDOW_S = 0.15
STATE_MISMATCH_S = 2.0

TEXT_STREAMS = {"messages", "Prints", "Alerts", "Console"}
TEXT_BAD = re.compile(
    r"(?i)\b(error|exception|brownout|browned|timeout|timed out|overrun|loop time of|unable|"
    r"fail|not found|disconnect|stale|watchdog|can bus|can id|phoenix|status frame|"
    r"nullpointer|indexoutofbounds|estop|e-stop|no joystick|joystick.*missing|"
    r"could not|refused|rejected|unresponsive|reset)"
    r"|\b[A-Z][A-Z0-9]{2,}(?:_[A-Z0-9]+)+\b"  # Telemetry.Fault names: CAMERA_OFFLINE, AUTO_SHOT_TIMEOUT_TRIGGERED, BROWNOUT
)
TEXT_IGNORE = re.compile(r"(?i)(^\s*$|Auton Warmed Up|Initialized|Subsystem Initialized)")

SETPOINT_PAIRS = [
    # (commanded key, measured key, absolute tol, relative tol, hold s, min |cmd| to care)
    ("Launcher/CommandedRPM", "Launcher/RPM", 150.0, 0.08, 1.5, 500.0),
    ("LauncherTower/CommandedRPM", "LauncherTower/RPM", 150.0, 0.08, 1.5, 500.0),
    ("Hood/CommandedDegrees", "Hood/PositionDegrees", 2.0, 0.0, 1.0, -1.0),
    ("Turret/CommandedDegrees", "Turret/PositionDegrees", 5.0, 0.0, 1.0, -1.0),
]

# The ShotReady/* keys that feed Composite; the rest are outputs of it or the operator override.
SHOT_GATE_INPUTS = {"LauncherAtSpeed", "HoodAtAngle", "TurretOnTarget", "RangeOk", "PoseTrusted", "ShotInRange"}
# Launcher/SystemState values in which isAtSpeed() can be true at all.
LAUNCH_STATES = {"LAUNCH", "SET_SHOT"}

STALL_COUNTERS = [
    "DyeRotor/RotorStallCount",
    "IntakeExtension/Agitate/StalledPulls",
    "IntakeExtension/Agitate/SkewHoldTimeouts",
    "IntakeExtension/PastMaxClamps",
    "IntakeExtension/OutPointRelearns",
    "IntakeExtension/RestResyncs",
    "Turret/StallLatchCount",
    "Turret/PositionStepsRejected",
    "Turret/PositionStepsAccepted",
    "CANConfig/FailedCalls",
]
STALL_FLAGS = [
    "FuelIntake/KickerStallLatched",
    "DyeRotor/RotorStallBackoff",
    "Turret/StallLatched",
    "Turret/PositionSuspect",
    "Turret/AngleOutsideEnvelope",
    "CANConfig/BudgetExhausted",
]

STATE_PAIRS = [
    ("SuperStructure/WantedSuperState", "SuperStructure/CurrentSuperState"),
    ("Swerve/WantedState", "Swerve/SystemState"),
    ("Launcher/WantedState", "Launcher/SystemState"),
    ("LauncherTower/WantedState", "LauncherTower/SystemState"),
    ("Hood/WantedState", "Hood/SystemState"),
    ("Turret/WantedState", "Turret/SystemState"),
    ("FuelIntake/WantedState", "FuelIntake/SystemState"),
    ("IntakeExtension/WantedState", "IntakeExtension/SystemState"),
    ("DyeRotor/WantedState", "DyeRotor/SystemState"),
]

DS_KEYS = {
    "DS:enabled",
    "DS:autonomous",
    "DS:test",
    "DS:estop",
    "DS:eventName",
    "DS:matchType",
    "DS:matchNumber",
    "DS:replayNumber",
    "DS:alliance",
    "DS:station",
    "DS:gameSpecificMessage",
}


@dataclass
class Finding:
    severity: str
    title: str
    t_start_us: int | None
    t_end_us: int | None
    evidence: str
    check: str
    detector: str

    def sort_key(self):
        return (SEVERITY_ORDER[self.severity], self.t_start_us or 0)


@dataclass
class Series:
    """Compact time series for a topic: parallel lists of timestamps (us) and values."""

    ts: list = field(default_factory=list)
    vs: list = field(default_factory=list)

    def add(self, t, v):
        self.ts.append(t)
        self.vs.append(v)

    def __len__(self):
        return len(self.ts)

    def value_at(self, t_us):
        """Last value at or before t_us (DogLog skips unchanged values, so hold the last one)."""
        i = bisect.bisect_right(self.ts, t_us) - 1
        return self.vs[i] if i >= 0 else None


@dataclass
class Window:
    kind: str  # auto / teleop / test
    start_us: int
    end_us: int

    @property
    def dur_s(self):
        return (self.end_us - self.start_us) / 1e6


# ── helpers ───────────────────────────────────────────────────────────────


def fmt_t(t_us: int | None, windows: list[Window]) -> str:
    if t_us is None:
        return "?"
    s = f"{t_us / 1e6:7.1f}s"
    for w in windows:
        if w.start_us <= t_us <= w.end_us:
            return f"{s} ({w.kind}+{(t_us - w.start_us) / 1e6:.1f}s)"
    return f"{s} (disabled)"


def in_windows(t_us: int, windows: list[Window]) -> Window | None:
    for w in windows:
        if w.start_us <= t_us <= w.end_us:
            return w
    return None


def true_segments(series: Series, end_us: int, predicate=lambda v: bool(v)):
    """Yield (start_us, end_us) spans where predicate(value) holds, extending the last to end_us."""
    start = None
    for t, v in zip(series.ts, series.vs):
        ok = predicate(v)
        if ok and start is None:
            start = t
        elif not ok and start is not None:
            yield start, t
            start = None
    if start is not None:
        yield start, end_us


def overlap_s(a0, a1, windows):
    tot = 0
    for w in windows:
        lo, hi = max(a0, w.start_us), min(a1, w.end_us)
        if hi > lo:
            tot += hi - lo
    return tot / 1e6


def normalize_msg(m: str) -> str:
    m = re.sub(r"TIME:\s*[\d.]+\s*\|\|\s*", "", m)
    m = re.sub(r"\d+(\.\d+)?", "#", m)
    return m.strip()[:160]


# ── main analysis ─────────────────────────────────────────────────────────


class Triage:
    def __init__(self, path: Path):
        self.path = path
        self.findings: list[Finding] = []
        self.clean: list[str] = []
        self.series: dict[str, Series] = defaultdict(Series)
        self.text: dict[str, list[tuple[int, str]]] = defaultdict(list)
        self.windows: list[Window] = []
        self.no_ds = False  # no DS:enabled topic: the whole log is treated as one window
        self.meta: dict[str, str] = {}
        self.log: LogFile | None = None

    # what to decode on the single pass
    def _want(self, e: Entry) -> bool:
        n = short_name(e.name)
        if n in DS_KEYS or n.startswith("DS:"):
            return n in DS_KEYS
        leaf = n.rsplit("/", 1)[-1]
        if leaf in TEXT_STREAMS and e.type == "string":
            return True
        if n.startswith(("BuildConstants/", "BatteryLogger/", "System/", "Scheduler/", "SystemStats/", "CANConfig/")):
            return True
        if leaf in {"MotorConnected", "StatorCurrent", "SupplyCurrent", "Temp", "Voltage"}:
            return True
        if n in {k for pair in SETPOINT_PAIRS for k in pair[:2]}:
            return True
        if n in STALL_COUNTERS or n in STALL_FLAGS:
            return True
        if n in {k for pair in STATE_PAIRS for k in pair}:
            return True
        if n.startswith(("Vision/", "Auton", "SuperStructure/ShotReady/", "Turret/")):
            return True
        if n == "Swerve/State/Pose":
            return True
        return False

    def run(self):
        log, data, pos = open_log(self.path)
        self.log = log
        for e, ts, payload in iter_records(log, data, pos, self._want, gap_us=int(GAP_S * 1e6)):
            n = short_name(e.name)
            leaf = n.rsplit("/", 1)[-1]
            v = decode(e.type, payload)
            if v is None:
                continue
            if leaf in TEXT_STREAMS and e.type == "string":
                self.text[leaf].append((ts, v))
            else:
                self.series[n].add(ts, v)
        del data
        self._windows()
        self._meta()
        self._ds()
        self._log_end()
        self._motors()
        self._power()
        self._loop()
        self._gaps()
        self._text()
        self._setpoints()
        self._counters()
        self._states()
        self._vision()
        self._auton()
        self.findings.sort(key=Finding.sort_key)

    # ── detectors ──

    def _windows(self):
        en = self.series.get("DS:enabled")
        auto = self.series.get("DS:autonomous")
        test = self.series.get("DS:test")
        end = self.log.last_us or 0
        if not en:
            self.no_ds = True
            if self.log.first_us is not None:
                self.windows.append(Window("log", self.log.first_us, end))
            return
        for s, e in true_segments(en, end):
            a = auto.value_at(s) if auto else False
            t = test.value_at(s) if test else False
            kind = "test" if t else ("auto" if a else "teleop")
            self.windows.append(Window(kind, s, e))

    def _meta(self):
        for k in ("BuildConstants/GitBranch", "BuildConstants/GitSHA", "BuildConstants/BuildDate", "BuildConstants/GitDate"):
            s = self.series.get(k)
            if s:
                self.meta[k.split("/")[1]] = str(s.vs[-1])
        for k in ("DS:eventName", "DS:matchType", "DS:matchNumber", "DS:alliance", "DS:station"):
            s = self.series.get(k)
            if s and s.vs:
                v = s.vs[-1]
                if isinstance(v, (bytes, bytearray)):
                    v = int.from_bytes(v, "little") if len(v) <= 8 else v.hex()
                self.meta[k[3:]] = str(v)
        # Robot-side copy of the same facts (Robot.logCanBusStatus, added 2026-09-19): DogLog's
        # DS capture never carried the match number in the 2026 logs.
        for k, name in (
            ("Match Data/EventName", "eventName"),
            ("Match Data/MatchType", "matchType"),
            ("Match Data/MatchNumber", "matchNumber"),
            ("Match Data/Alliance", "alliance"),
            ("Match Data/Station", "station"),
        ):
            s = self.series.get(k)
            if s and s.vs and name not in self.meta:
                self.meta[name] = str(s.vs[-1])

    def _ds(self):
        es = self.series.get("DS:estop")
        if es:
            # One E-stop is one event. After the stop the DS toggles the flag every time the
            # driver tries to re-enable, and the 2026-08-01 TXDRI1 Q20 log reported a single stop
            # at teleop+54 s as eight CRITICAL findings. Toggles that start while already disabled
            # are folded into the event before them; a stop with no enabled period near it is
            # the DS being reset on the cart, and only worth a LOW.
            groups: list[list] = []
            for s, e in true_segments(es, self.log.last_us):
                if groups and not in_windows(s, self.windows) and s - groups[-1][1] < 30_000_000:
                    groups[-1][1] = e
                    groups[-1][2] += 1
                else:
                    groups.append([s, e, 1])
            for s, e, n in groups:
                live = in_windows(s, self.windows) or any(abs(s - w.end_us) < 2_000_000 for w in self.windows if w.kind != "log")
                self.findings.append(
                    Finding(
                        "CRITICAL" if live else "LOW",
                        "Robot was E-STOPPED" if live else "E-stop toggled while disabled",
                        s, e,
                        "DS:estop true" + (f", toggled {n}x" if n > 1 else ""),
                        "Ask the drive team / FTA what happened; check for a robot fault that made them stop it." if live else "Driver Station reset, not a robot event.",
                        "ds",
                    )
                )
        # disable + re-enable inside a match (FMS match info present)
        in_match = self.meta.get("matchNumber") not in (None, "0", "")
        if len(self.windows) >= 2 and in_match:
            for a, b in zip(self.windows, self.windows[1:]):
                gap = (b.start_us - a.end_us) / 1e6
                if a.kind == b.kind and 0 < gap < 60:
                    self.findings.append(
                        Finding(
                            "CRITICAL",
                            f"Robot disabled mid-{a.kind} for {gap:.1f} s then re-enabled",
                            a.end_us,
                            b.start_us,
                            "DS:enabled dropped and came back inside the same match period",
                            "Comms drop (radio power, Ethernet to radio, RIO reboot) or the DS disabled it. Look for a log gap or console restart at this time.",
                            "ds",
                        )
                    )

    def _log_end(self):
        if not self.windows or self.no_ds:
            return
        last = self.log.last_us
        w = self.windows[-1]
        if w.end_us >= last - 200_000:  # log ends while still enabled
            self.findings.append(
                Finding(
                    "CRITICAL",
                    f"Log ends while robot still ENABLED in {w.kind}",
                    last,
                    None,
                    f"last record at {last / 1e6:.1f}s, DS:enabled never went false; file {'truncated' if self.log.truncated else 'closed cleanly'}",
                    "Robot code crashed or the RIO rebooted/lost power. Check the console tail below, battery voltage, and the RIO power connector.",
                    "log",
                )
            )
        auto = [w for w in self.windows if w.kind == "auto"]
        if auto and 0 < auto[0].dur_s < 14.0 and self.meta.get("matchNumber") not in (None, "0", ""):
            self.findings.append(
                Finding("HIGH", f"Auto only lasted {auto[0].dur_s:.1f} s", auto[0].start_us, auto[0].end_us, "DS:enabled+autonomous window shorter than a 15 s auto period", "Was the robot disabled early? Look for E-stop, comms loss, or the log ending.", "ds")
            )

    def _motors(self):
        found = False
        for n, s in self.series.items():
            if not n.endswith("/MotorConnected"):
                continue
            found = True
            mech = n[: -len("/MotorConnected")]
            for a, b in true_segments(s, self.log.last_us, predicate=lambda v: v is False):
                dur = (b - a) / 1e6
                if dur < 0.1:
                    continue
                en = overlap_s(a, b, self.windows)
                sev = "CRITICAL" if en > 0 else "HIGH"
                self.findings.append(
                    Finding(
                        sev,
                        f"{mech} motor DISCONNECTED for {dur:.1f} s" + (" while enabled" if en else " (disabled)"),
                        a,
                        b,
                        f"{n} false",
                        f"CAN wiring/power to the {mech} motor; check its CAN ID in the robot config and the device list in Phoenix Tuner X.",
                        "motors",
                    )
                )
        if found and not any(f.detector == "motors" for f in self.findings):
            self.clean.append("all motors stayed connected")
        # temps and stator currents
        hot = []
        for n, s in self.series.items():
            if n.endswith("/Temp") and s.vs:
                mx = max(v for v in s.vs if isinstance(v, (int, float)))
                if mx >= TEMP_HIGH_C:
                    i = s.vs.index(mx)
                    self.findings.append(Finding("HIGH", f"{n[:-5]} motor HOT: {mx:.0f} °C", s.ts[i], None, f"{n} peaked at {mx:.0f} °C", "Motor is thermally limiting; let it cool, check for a mechanical bind or a stalled mechanism.", "motors"))
                elif mx >= TEMP_WARM_C:
                    hot.append(f"{n[:-5]} {mx:.0f} °C")
        if hot:
            self.findings.append(Finding("LOW", "Warm motors: " + ", ".join(hot), None, None, f"above {TEMP_WARM_C:.0f} °C, below {TEMP_HIGH_C:.0f} °C", "Fine for now; watch it if this mechanism is misbehaving.", "motors"))
        for n, s in self.series.items():
            if not n.endswith("/StatorCurrent") or not s.vs:
                continue
            for a, b in true_segments(s, self.log.last_us, predicate=lambda v: isinstance(v, (int, float)) and v >= STATOR_HIGH_A):
                if (b - a) / 1e6 >= STATOR_HOLD_S:
                    self.findings.append(
                        Finding("MEDIUM", f"{n[:-14]} stator current ≥ {STATOR_HIGH_A:.0f} A for {(b - a) / 1e6:.1f} s", a, b, f"{n} sustained high", "Likely a stall or jam; pair with the RPM/position of that mechanism at this time.", "motors")
                    )

    def _power(self):
        v = self.series.get("BatteryLogger/BatteryVoltage") or self.series.get("SystemStats/BatteryVoltage")
        bo = self.series.get("SystemStats/BrownedOut")
        if bo:
            for a, b in true_segments(bo, self.log.last_us):
                self.findings.append(Finding("CRITICAL", f"RIO BROWNOUT for {(b - a) / 1e6:.2f} s", a, b, "SystemStats/BrownedOut true", "Battery, main breaker, or battery leads. Swap the battery and check the connector crimps.", "power"))
        if not v:
            self.clean.append("no battery voltage topic logged (BatteryLogger)")
            return
        nums = [(t, x) for t, x in zip(v.ts, v.vs) if isinstance(x, (int, float))]
        if not nums:
            return
        mn_t, mn = min(nums, key=lambda p: p[1])
        enabled_nums = [(t, x) for t, x in nums if in_windows(t, self.windows)] or nums
        e_t, e_mn = min(enabled_nums, key=lambda p: p[1])
        if mn <= BROWNOUT_V and not bo:
            self.findings.append(Finding("CRITICAL", f"Battery hit {mn:.2f} V (brownout range)", mn_t, None, "BatteryLogger/BatteryVoltage", "RIO brownout: motors cut out. Swap battery; check main breaker and battery leads; look for a stalled mechanism drawing current at this moment.", "power"))
        else:
            dips = [(a, b) for a, b in true_segments(Series([t for t, _ in nums], [x for _, x in nums]), self.log.last_us, predicate=lambda x: x < LOW_BATTERY_V) if overlap_s(a, b, self.windows) > 0]
            if dips:
                a, b = max(dips, key=lambda d: d[1] - d[0])
                hold = (b - a) / 1e6
                sev = "HIGH" if hold >= LOW_BATTERY_HOLD_S else "MEDIUM"
                self.findings.append(Finding(sev, f"Battery dipped below {LOW_BATTERY_V:.0f} V {len(dips)}x while enabled (min {e_mn:.2f} V, longest {hold:.1f} s)", a, b, "BatteryLogger/BatteryVoltage", "Weak battery or a heavy current spike. Swap the battery; pair the time with stator-current findings to see what was drawing.", "power"))
            else:
                self.clean.append(f"battery ok (min {e_mn:.2f} V while enabled)")
        cur = self.series.get("BatteryLogger/Current")
        if cur and cur.vs:
            mx = max((x for x in cur.vs if isinstance(x, (int, float))), default=0)
            if mx > 250:
                i = cur.vs.index(mx)
                self.findings.append(Finding("MEDIUM", f"Total current peaked at {mx:.0f} A", cur.ts[i], None, "BatteryLogger/Current", "Pair with stator currents above to find the mechanism.", "power"))

    def _loop(self):
        issues = 0
        mx = self.series.get("System/Loop/MaxPeriodMs")
        if mx and mx.vs:
            worst = max(mx.vs)
            n_bad = sum(1 for x in mx.vs if x >= LOOP_MAX_MS)
            if n_bad:
                i = mx.vs.index(worst)
                issues += 1
                self.findings.append(Finding("HIGH" if worst >= 200 else "MEDIUM", f"Loop overruns: worst period {worst:.0f} ms ({n_bad} buckets ≥ {LOOP_MAX_MS:.0f} ms)", mx.ts[i], None, "System/Loop/MaxPeriodMs", "Code stalls: check the console for 'Loop time of' warnings and exceptions; check CPU/GC below; a Phoenix blocking call or a slow periodic() is the usual cause.", "loop"))
        ov = self.series.get("System/Loop/OverrunPercent")
        if ov and ov.vs:
            worst = max(ov.vs)
            if worst >= OVERRUN_PCT:
                i = ov.vs.index(worst)
                issues += 1
                self.findings.append(Finding("MEDIUM", f"Loop overrun rate up to {worst:.0f}% of loops", ov.ts[i], None, "System/Loop/OverrunPercent", "Sustained slow loop; see CPU and GC findings.", "loop"))
        cpu = self.series.get("System/CpuPercent")
        if cpu and cpu.vs:
            for a, b in true_segments(cpu, self.log.last_us, predicate=lambda x: isinstance(x, (int, float)) and x >= CPU_PCT):
                if (b - a) / 1e6 >= 5:
                    issues += 1
                    self.findings.append(Finding("MEDIUM", f"RIO CPU ≥ {CPU_PCT:.0f}% for {(b - a) / 1e6:.0f} s", a, b, "System/CpuPercent", "Logging/NT load or a busy periodic(); was Telemetry/MirrorLogsToNT left on?", "loop"))
                    break
        mem = self.series.get("System/MemAvailableMB")
        if mem and mem.vs:
            mn = min(mem.vs)
            if mn <= MEM_LOW_MB:
                issues += 1
                self.findings.append(Finding("HIGH", f"RIO memory down to {mn:.0f} MB free", mem.ts[mem.vs.index(mn)], None, "System/MemAvailableMB", "Memory leak or heap pressure; code may be about to crash. Reboot RIO before the match; look for growing collections.", "loop"))
        gc = self.series.get("System/Gc/MsPerSecond")
        if gc and gc.vs:
            mxg = max(gc.vs)
            if mxg >= 200:
                issues += 1
                self.findings.append(Finding("MEDIUM", f"GC taking {mxg:.0f} ms per second", gc.ts[gc.vs.index(mxg)], None, "System/Gc/MsPerSecond", "Allocation churn in a periodic(); pairs with loop overruns.", "loop"))
        for n, s in self.series.items():
            if n.startswith("Scheduler/") and s.vs:
                worst = max((x for x in s.vs if isinstance(x, (int, float))), default=0)
                if worst >= 0.1:
                    issues += 1
                    self.findings.append(Finding("MEDIUM", f"{n} took {worst * 1000:.0f} ms once", s.ts[s.vs.index(worst)], None, f"{n} (seconds)", "That phase of the loop stalled; find the exception or blocking call near this time.", "loop"))
        if not issues and (mx or cpu):
            self.clean.append("loop timing / CPU / memory ok")

    def _gaps(self):
        """A stream that normally changes every loop but goes quiet while enabled = code stall or log-thread starvation."""
        if not self.windows:
            return
        dur = (self.log.last_us - self.log.first_us) / 1e6 or 1
        candidates = [
            e
            for e in self.log.entries.values()
            if e.type == "double" and e.count / dur >= 20 and not short_name(e.name).startswith("DS:")
        ]
        candidates.sort(key=lambda e: -e.count)
        candidates = candidates[:3]
        if not candidates:
            return
        gaps = [
            (a, b, short_name(e.name))
            for e in candidates
            for a, b in e.gaps
            if in_windows(a, self.windows) and in_windows(b, self.windows)
        ]
        if gaps:
            gaps.sort()
            merged: list[list] = []
            for a, b, name in gaps:
                if merged and a <= merged[-1][1] + 20_000:
                    merged[-1][1] = max(merged[-1][1], b)
                    merged[-1][2].add(name)
                else:
                    merged.append([a, b, {name}])
            worst = max(merged, key=lambda m: m[1] - m[0])
            self.findings.append(
                Finding(
                    "HIGH" if (worst[1] - worst[0]) / 1e6 >= 1.0 else "MEDIUM",
                    f"Robot code went quiet {len(merged)}x while enabled (longest {(worst[1] - worst[0]) / 1e6:.2f} s)",
                    worst[0],
                    worst[1],
                    "no records on loop-rate topics " + ", ".join(sorted(worst[2])),
                    "Code stall (blocking call, GC pause, exception storm) or log thread starved. Check console at this time and the loop findings.",
                    "gaps",
                )
            )
        else:
            self.clean.append("no data gaps on loop-rate topics while enabled")

    def _text(self):
        groups: dict[tuple[str, str], list[int]] = defaultdict(list)
        for stream, items in self.text.items():
            for t, msg in items:
                if TEXT_IGNORE.search(msg):
                    continue
                if stream == "Alerts" or TEXT_BAD.search(msg):
                    groups[(stream, normalize_msg(msg))].append(t)
        if not groups:
            if self.text:
                self.clean.append("no errors/warnings in console, Prints, or Alerts")
            return
        for (stream, msg), ts in sorted(groups.items(), key=lambda kv: kv[1][0]):
            low = msg.lower()
            if stream == "Alerts":
                sev = "HIGH" if low.startswith("error") else ("MEDIUM" if low.startswith("warning") else "LOW")
            elif "exception" in low or "error" in low or "brownout" in low or "estop" in low:
                sev = "HIGH"
            elif "loop time of" in low or "overrun" in low:
                sev = "LOW" if len(ts) < 20 else "MEDIUM"
            else:
                sev = "MEDIUM"
            enabled_hits = sum(1 for t in ts if in_windows(t, self.windows))
            count = f"{len(ts)}x" + (f", {enabled_hits} while enabled" if self.windows else "")
            self.findings.append(Finding(sev, f"[{stream}] {msg[:110]}", ts[0], ts[-1] if len(ts) > 1 else None, count, "Read the surrounding console lines with --tail or the wpilog-decode skill.", "text"))

    def _setpoints(self):
        any_pair = False
        for cmd_k, meas_k, tol_abs, tol_rel, hold, min_cmd in SETPOINT_PAIRS:
            cmd, meas = self.series.get(cmd_k), self.series.get(meas_k)
            if not cmd or not meas:
                continue
            any_pair = True
            # walk measured samples, hold last commanded
            start = None
            worst = 0.0
            spans = []
            prev_t = None
            for t, m in zip(meas.ts, meas.vs):
                c = cmd.value_at(t)
                # A gap in the measured samples is a disable (logDash keys stop while disabled), so
                # a span must not bridge it: the launcher's "72.8 s" miss in the 2026-09-19 practice
                # log was a 6 s burst, a 67 s disable, and the next enable's first sample.
                if prev_t is not None and t - prev_t > 1_000_000 and start is not None:
                    spans.append((start, prev_t, worst))
                    start, worst = None, 0.0
                prev_t = t
                if c is None or not isinstance(m, (int, float)) or not isinstance(c, (int, float)) or abs(c) < min_cmd or not in_windows(t, self.windows):
                    if start is not None:
                        spans.append((start, t, worst))
                        start, worst = None, 0.0
                    continue
                err = abs(c - m)
                tol = max(tol_abs, tol_rel * abs(c))
                if err > tol:
                    if start is None:
                        start = t
                    worst = max(worst, err)
                elif start is not None:
                    spans.append((start, t, worst))
                    start, worst = None, 0.0
            if start is not None:
                spans.append((start, meas.ts[-1], worst))
            bad = [s for s in spans if (s[1] - s[0]) / 1e6 >= hold]
            if bad:
                a, b, w = max(bad, key=lambda s: s[1] - s[0])
                mech = cmd_k.split("/")[0]
                self.findings.append(
                    Finding(
                        "HIGH",
                        f"{mech} not reaching setpoint: {len(bad)} spans ≥ {hold:.1f} s (longest {(b - a) / 1e6:.1f} s, error {w:.0f})",
                        a,
                        b,
                        f"{meas_k} vs {cmd_k}",
                        f"{mech} motor disconnected/stalled, a mechanical bind, low battery, or a follower fighting the leader. Check the {mech} current and MotorConnected at this time.",
                        "setpoint",
                    )
                )
        if any_pair and not any(f.detector == "setpoint" for f in self.findings):
            self.clean.append("launcher / hood / turret tracked their setpoints")
        te = self.series.get("Turret/TrackingErrorDegrees")
        if te and te.vs:
            for a, b in true_segments(te, self.log.last_us, predicate=lambda x: isinstance(x, (int, float)) and abs(x) > 5):
                if (b - a) / 1e6 >= 1.0 and overlap_s(a, b, self.windows) > 0:
                    self.findings.append(Finding("MEDIUM", f"Turret tracking error > 5° for {(b - a) / 1e6:.1f} s", a, b, "Turret/TrackingErrorDegrees", "Turret lagging its target: check Turret/MotorConnected, Turret/Unwrapping, and the vision pose trust at this time.", "setpoint"))
                    break

    def _counters(self):
        for k in STALL_COUNTERS:
            s = self.series.get(k)
            if not s or not s.vs:
                continue
            nums = [x for x in s.vs if isinstance(x, (int, float))]
            if len(nums) < 2:
                continue
            delta = nums[-1] - nums[0]
            if delta > 0:
                sev = "HIGH" if k.startswith("CANConfig") else "MEDIUM"
                self.findings.append(Finding(sev, f"{k} increased by {delta:g} (to {nums[-1]:g})", s.ts[1] if len(s.ts) > 1 else s.ts[0], s.ts[-1], f"first change at {s.ts[1] / 1e6:.1f}s" if len(s.ts) > 1 else "", "Stall/retry counter moved: check that mechanism for a jam or a fuel piece stuck; CANConfig counters mean device configs failed to apply (CAN bus/device power).", "counters"))
        for k in STALL_FLAGS:
            s = self.series.get(k)
            if not s:
                continue
            for a, b in true_segments(s, self.log.last_us):
                self.findings.append(Finding("MEDIUM", f"{k} was true for {(b - a) / 1e6:.1f} s", a, b, k, "Stall protection latched; the mechanism was fighting something.", "counters"))
                break

    def _states(self):
        for wk, ck in STATE_PAIRS:
            w, c = self.series.get(wk), self.series.get(ck)
            if not w or not c:
                continue
            # sample at every change of either
            times = sorted(set(w.ts) | set(c.ts))
            start = None
            worst = None
            for t in times:
                wv, cv = w.value_at(t), c.value_at(t)
                mism = wv is not None and cv is not None and str(wv) != str(cv) and in_windows(t, self.windows)
                if mism and start is None:
                    start = t
                    worst = (wv, cv)
                elif not mism and start is not None:
                    if (t - start) / 1e6 >= STATE_MISMATCH_S:
                        self.findings.append(Finding("MEDIUM", f"{wk.split('/')[0]} wanted {worst[0]} but sat in {worst[1]} for {(t - start) / 1e6:.1f} s", start, t, f"{wk} vs {ck}", "State machine could not reach the wanted state: look at that subsystem's readiness gates and setpoint findings.", "states"))
                        break
                    start = None
        # Only the inputs to Composite can block a shot. The other ShotReady/* keys (Override,
        # StartReady, KeepReady, GateOpen, FeedAllowed, HoldingFeed) are outputs or the operator's
        # override, and are false most of a match by design: Chezy Q11 (2026-09-19) ranked
        # "Override 130s, StartReady 130s" as the blockers of a match where every real gate was open.
        gates = {n: s for n, s in self.series.items() if n.startswith("SuperStructure/ShotReady/") and n.rsplit("/", 1)[-1] in SHOT_GATE_INPUTS and s.vs and isinstance(s.vs[0], bool)}
        comp = self.series.get("SuperStructure/ShotReady/Composite")
        cmd = self.series.get("Launcher/CommandedRPM")
        # "Spun up" means a launch state, not the 700 RPM IDLE_PREP idle: isAtSpeed() is false by
        # definition outside LAUNCH/SET_SHOT, so counting idle time blamed LauncherAtSpeed for 110 s
        # of a practice session in which every set shot reached speed within a second.
        lstate = self.series.get("Launcher/SystemState")
        if comp and gates and cmd:
            # while the launcher is spun up and composite is false, which gate is false?
            blame: dict[str, float] = defaultdict(float)
            total = 0.0
            for a, b in true_segments(comp, self.log.last_us, predicate=lambda v: v is False):
                # step through in 50 ms slices
                t = a
                while t < b:
                    c = cmd.value_at(t)
                    if lstate:
                        spun = str(lstate.value_at(t)) in LAUNCH_STATES
                    else:
                        spun = isinstance(c, (int, float)) and c > 1000
                    if spun and in_windows(t, self.windows):
                        total += 0.05
                        for n, s in gates.items():
                            if s.value_at(t) is False:
                                blame[n.rsplit("/", 1)[-1]] += 0.05
                    t += 50_000
            if total >= 3.0:
                top = sorted(blame.items(), key=lambda kv: -kv[1])[:3]
                self.findings.append(
                    Finding(
                        "HIGH",
                        f"Shot NOT ready for {total:.0f} s while in a launch state; blocking gates: " + ", ".join(f"{k} {v:.0f}s" for k, v in top),
                        None,
                        None,
                        "SuperStructure/ShotReady/Composite false while Launcher/SystemState is LAUNCH or SET_SHOT",
                        "The first gate named is why fuel did not feed. PoseTrusted → vision; TurretOnTarget → turret; LauncherAtSpeed/HoodAtAngle → those mechanisms; RangeOk → driver position. A few seconds here is normal: every burst re-earns the gate.",
                        "states",
                    )
                )
            elif comp:
                self.clean.append("shot-ready gates opened whenever the launcher was in a launch state")

    def _vision(self):
        tr = self.series.get("Vision/PoseTrustedForAiming")
        enabled_s = sum(w.dur_s for w in self.windows)
        if tr and enabled_s > 0:
            untrusted = sum(overlap_s(a, b, self.windows) for a, b in true_segments(tr, self.log.last_us, predicate=lambda v: v is False))
            frac = untrusted / enabled_s
            if frac > 0.3:
                self.findings.append(Finding("HIGH", f"Vision pose NOT trusted for {frac * 100:.0f}% of enabled time ({untrusted:.0f} s)", None, None, "Vision/PoseTrustedForAiming false", "Camera offline/misaligned, no tags in view, or the pose never seeded. Check the camera feed and the Alerts/console for camera messages.", "vision"))
            elif frac > 0.1:
                self.findings.append(Finding("MEDIUM", f"Vision pose untrusted {frac * 100:.0f}% of enabled time", None, None, "Vision/PoseTrustedForAiming", "Brief dropouts; ok if they line up with driving away from tags.", "vision"))
            else:
                self.clean.append(f"vision pose trusted {100 - frac * 100:.0f}% of enabled time")
        seed = self.series.get("Vision/PoseSeedConfirmed")
        auto = [w for w in self.windows if w.kind == "auto"]
        if seed and auto:
            v = seed.value_at(auto[0].start_us)
            if v is False:
                self.findings.append(Finding("HIGH", "Pose was NOT seed-confirmed at auto start", auto[0].start_us, None, "Vision/PoseSeedConfirmed false at DS enable", "Robot placed without the camera seeing a tag, or camera down. Auto paths start from a wrong pose.", "vision"))
        since = self.series.get("Vision/SecondsSinceAcceptedEstimate")
        if since and since.vs:
            en = [(t, x) for t, x in zip(since.ts, since.vs) if isinstance(x, (int, float)) and in_windows(t, self.windows)]
            if en:
                t, mx = max(en, key=lambda p: p[1])
                if mx >= 5:
                    self.findings.append(Finding("MEDIUM", f"No accepted vision estimate for {mx:.0f} s while enabled", t - int(mx * 1e6), t, "Vision/SecondsSinceAcceptedEstimate", "Camera dropout or no tags in view.", "vision"))
        rej = self.series.get("Vision/PoseReset/Rejection")
        if rej:
            msgs = [(t, str(v)) for t, v in zip(rej.ts, rej.vs) if str(v).strip() and str(v).lower() not in ("none", "")]
            if msgs:
                self.findings.append(Finding("MEDIUM", f"Pose resets rejected {len(msgs)}x: {msgs[-1][1][:80]}", msgs[0][0], msgs[-1][0], "Vision/PoseReset/Rejection", "The auto-start pose reset was refused; see the reason string.", "vision"))
        slip = self.series.get("Vision/TurretZero/SlipDegPerMinute")
        if slip and slip.vs:
            mx = max((abs(x) for x in slip.vs if isinstance(x, (int, float))), default=0)
            if mx >= 2.0:
                self.findings.append(Finding("MEDIUM", f"Turret zero slipping {mx:.1f}°/min", None, None, "Vision/TurretZero/SlipDegPerMinute", "Turret encoder/belt slipping; re-zero and inspect the turret drive.", "vision"))
        pose = self.series.get("Swerve/State/Pose")
        if pose and len(pose) > 2:
            jumps = []
            for (t0, p0), (t1, p1) in zip(zip(pose.ts, pose.vs), zip(pose.ts[1:], pose.vs[1:])):
                if not (isinstance(p0, tuple) and isinstance(p1, tuple)):
                    continue
                dt = (t1 - t0) / 1e6
                if 0 < dt <= POSE_JUMP_WINDOW_S:
                    d = ((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2) ** 0.5
                    if d >= POSE_JUMP_M and in_windows(t1, self.windows):
                        jumps.append((t1, d))
            if jumps:
                t, d = max(jumps, key=lambda j: j[1])
                self.findings.append(Finding("MEDIUM", f"Odometry pose jumped {len(jumps)}x while enabled (largest {d:.1f} m)", jumps[0][0], t, "Swerve/State/Pose", "Bad vision measurement accepted or a pose reset mid-match; turret aim and auto paths go wrong right after.", "vision"))

    def _auton(self):
        auto = [w for w in self.windows if w.kind == "auto"]
        if not auto:
            return
        a0 = auto[0]
        pe = self.series.get("Auton/StartPoseErrorMeters")
        he = self.series.get("Auton/StartHeadingErrorDeg")
        if pe and pe.vs:
            v = pe.value_at(a0.start_us + 500_000)
            if isinstance(v, (int, float)) and v >= 0.3:
                self.findings.append(Finding("HIGH", f"Robot placed {v:.2f} m from the auto start pose", a0.start_us, None, "Auton/StartPoseErrorMeters", "Placement or a wrong auto selected; the path started from the wrong spot.", "auton"))
        if he and he.vs:
            v = he.value_at(a0.start_us + 500_000)
            if isinstance(v, (int, float)) and abs(v) >= 5:
                self.findings.append(Finding("HIGH", f"Robot heading {v:.1f}° off the auto start heading", a0.start_us, None, "Auton/StartHeadingErrorDeg", "Placement or gyro/vision seeding; check PoseSeedConfirmed.", "auton"))
        wu = self.series.get("Auton Warmed Up")
        if wu and wu.value_at(a0.start_us) is False:
            self.findings.append(Finding("MEDIUM", "Auto path was not warmed up at auto start", a0.start_us, None, "Auton Warmed Up false", "First path load happened live; expect a late start. Was the auto changed right before the match?", "auton"))
        pose = self.series.get("Swerve/State/Pose")
        if pose and len(pose) > 2:
            pts = [(t, p) for t, p in zip(pose.ts, pose.vs) if isinstance(p, tuple) and a0.start_us <= t <= a0.end_us]
            if len(pts) >= 2:
                # Furthest the pose got from where auto began, not end minus start. In the
                # 2026-09-19 Chezy P8 log the robot drove the whole auto and the pose was then
                # snapped back to the start pose on the disable, so end minus start read 0.00 m
                # and this fired CRITICAL on a working auto.
                x0, y0 = pts[0][1][0], pts[0][1][1]
                d = max(((p[0] - x0) ** 2 + (p[1] - y0) ** 2) ** 0.5 for _, p in pts)
                if d < 0.2 and a0.dur_s > 3:
                    self.findings.append(Finding("CRITICAL", "Robot did NOT move during auto", a0.start_us, a0.end_us, f"Swerve/State/Pose moved {d:.2f} m over {a0.dur_s:.0f} s", "No auto selected, path failed to load (console 'Could not load path planner paths'), or drive motors disconnected.", "auton"))
                else:
                    self.clean.append(f"robot drove {d:.1f} m in auto")

    def dur_s(self) -> float:
        return ((self.log.last_us or 0) - (self.log.first_us or 0)) / 1e6

    # ── output ──

    def report(self, show_all=False, tail=0) -> str:
        L = []
        m = self.meta
        dur = ((self.log.last_us or 0) - (self.log.first_us or 0)) / 1e6
        L.append(f"== {self.path.name}  ({self.log.size_bytes / 1e6:.1f} MB, {dur:.0f} s, {len(self.log.entries)} topics" + (", TRUNCATED" if self.log.truncated else "") + ")")
        build = " ".join(filter(None, [m.get("GitBranch"), m.get("GitSHA", "")[:8], m.get("BuildDate")]))
        if build:
            L.append(f"   build: {build}")
        match = " ".join(f"{k}={m[k]}" for k in ("eventName", "matchType", "matchNumber", "alliance", "station") if k in m and m[k] not in ("", "0"))
        if match:
            L.append(f"   match: {match}")
        if self.windows:
            L.append("   enabled: " + "; ".join(f"{w.kind} {w.start_us / 1e6:.1f}s→{w.end_us / 1e6:.1f}s ({w.dur_s:.1f} s)" for w in self.windows))
        elif self.no_ds:
            L.append("   enabled: unknown (no DS:enabled topic; whole log treated as one window)")
        else:
            L.append("   enabled: never (no DS:enabled true) — nothing below is match data")
        L.append("")
        shown = self.findings if show_all else [f for f in self.findings if SEVERITY_ORDER[f.severity] <= 2][:8]
        if not self.findings:
            L.append("NO SUSPECTS FOUND by the automatic checks. If the robot still misbehaved, describe the symptom and dig with the wpilog-decode skill.")
        else:
            L.append("TOP SUSPECTS (ranked)")
            for i, f in enumerate(shown, 1):
                when = fmt_t(f.t_start_us, self.windows)
                if f.t_end_us and f.t_start_us:
                    when += f" → {f.t_end_us / 1e6:.1f}s"
                L.append(f"{i:2d}. [{f.severity}] {f.title}")
                if f.t_start_us is not None:
                    L.append(f"      when: {when}")
                if f.evidence:
                    L.append(f"      evidence: {f.evidence}")
                L.append(f"      check: {f.check}")
            hidden = len(self.findings) - len(shown)
            if hidden > 0:
                L.append(f"    (+{hidden} lower-severity findings; rerun with --all)")
        if self.clean:
            L.append("")
            L.append("LOOKED CLEAN: " + "; ".join(self.clean))
        if tail:
            L.append("")
            L.append(f"LAST {tail} CONSOLE / PRINT LINES")
            lines = sorted([(t, s, msg) for s in ("messages", "Prints") for t, msg in self.text.get(s, [])])[-tail:]
            for t, s, msg in lines:
                L.append(f"   {fmt_t(t, self.windows)} [{s}] {msg.strip()[:160]}")
        return "\n".join(L)

    def to_json(self):
        return {
            "file": str(self.path),
            "meta": self.meta,
            "duration_s": ((self.log.last_us or 0) - (self.log.first_us or 0)) / 1e6,
            "truncated": self.log.truncated,
            "enabled_windows": [{"kind": w.kind, "start_s": w.start_us / 1e6, "end_s": w.end_us / 1e6} for w in self.windows],
            "findings": [
                {"severity": f.severity, "title": f.title, "start_s": None if f.t_start_us is None else f.t_start_us / 1e6, "end_s": None if f.t_end_us is None else f.t_end_us / 1e6, "evidence": f.evidence, "check": f.check, "detector": f.detector}
                for f in self.findings
            ],
            "clean": self.clean,
        }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help=".wpilog files or directories (newest N per directory, see --latest)")
    ap.add_argument("--latest", type=int, default=1, help="how many newest logs to take from each directory (default 1)")
    ap.add_argument("--all", action="store_true", help="show every finding, including LOW")
    ap.add_argument("--tail", type=int, default=0, metavar="N", help="also print the last N console/Print lines")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args()

    paths = find_logs(args.logs, args.latest)
    results = []
    for p in paths:
        t = Triage(p)
        t.run()
        results.append(t)
    if args.json:
        print(json.dumps([t.to_json() for t in results], indent=1))
        return
    if len(results) > 1:
        ordered = sorted(results, key=lambda t: t.path.stat().st_mtime)
        for a, b in zip(ordered, ordered[1:]):
            gap_min = (b.path.stat().st_mtime - b.dur_s() - a.path.stat().st_mtime) / 60
            if -1 < gap_min < 10:
                print(f"NOTE: {b.path.name} starts about {max(gap_min, 0):.1f} min after {a.path.name} ends: robot code restarted or the RIO rebooted between them.")
        print()
    for t in results:
        print(t.report(show_all=args.all, tail=args.tail))
        print()


if __name__ == "__main__":
    main()
