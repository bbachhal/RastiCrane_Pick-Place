#!/usr/bin/env python3
"""RASTICrane dashboard: local web UI <-> USB serial bridge.

A browser cannot open COM3, so this tiny server owns the port and the page talks
to it over HTTP on localhost. It sends exactly the lines the UI asks for and
never invents motion commands of its own.

    python server.py            # then open http://127.0.0.1:8787

The firmware remains the safety authority: it clamps every pulse to its own
bounds, ramps all motion, and rejects anything outside its calibration.
"""

import base64
import json
import math
import os
import re
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import serial
import serial.tools.list_ports

try:
    import plan
    import vision
except Exception as _exc:      # OpenCV missing or broken
    vision = None
    VISION_IMPORT_ERROR = str(_exc)

HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.join(HERE, "web")
POINTS_FILE = os.path.join(HERE, "points.json")
SEQ_FILE = os.path.join(HERE, "sequences.json")
VISION_FILE = os.path.join(HERE, "vision.json")
HOME_FILE = os.path.join(HERE, "home.json")
UPLOAD_DIR = os.path.join(HERE, "uploads")
LAST_PHOTO = os.path.join(UPLOAD_DIR, "last_photo.jpg")
CAD_FILE = os.path.join(HERE, "cad_geometry.json")
CONFIG_FILE = os.path.join(HERE, "config.json")
PORT = 8787

PULSE_RE = re.compile(r"Commanded M1/M2/base:\s*(\d+)\s*/\s*(\d+)\s*/\s*(\d+)")
XYZ_RE = re.compile(r"MODEL magnet XYZ:\s*(-?[\d.]+)\s*/\s*(-?[\d.]+)\s*/\s*(-?[\d.]+)")


class Link:
    """Owns the serial port and a rolling log of everything seen or sent."""

    def __init__(self):
        self.lock = threading.Lock()
        self.port = None
        self.name = ""
        self.log = deque(maxlen=600)
        self.counter = 0
        self.pulses = [None, None, None]
        self.xyz = [None, None, None]
        self.reader = None
        self.stop_reader = threading.Event()
        self._checked = 0.0

    def add(self, text, kind="rx"):
        with self.lock:
            self.counter += 1
            self.log.append({"n": self.counter, "t": time.time(), "kind": kind, "text": text})

    def open(self, name, baud=115200):
        self.close()
        # dsrdtr/rts False keeps us from strobing the reset line harder than needed.
        port = serial.Serial()
        port.port = name
        port.baudrate = baud
        port.timeout = 0.2
        port.dtr = False
        port.rts = False
        port.open()
        self.port = port
        self.name = name
        self.stop_reader.clear()
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()
        self.add(f"--- opened {name} at {baud} ---", "sys")

    def close(self):
        self.stop_reader.set()
        if self.reader and self.reader.is_alive():
            self.reader.join(timeout=1.0)
        self.reader = None
        if self.port:
            try:
                self.port.close()
            except Exception:
                pass
            self.add(f"--- closed {self.name} ---", "sys")
        self.port = None
        self.name = ""

    def _fail(self, message):
        """Device vanished or errored: drop the handle without joining our own thread."""
        self.add(f"--- {message} ---", "sys")
        self.stop_reader.set()
        port, self.port, self.name = self.port, None, ""
        self.pulses = [None, None, None]
        if port:
            try:
                port.close()
            except Exception:
                pass

    def _read_loop(self):
        buffer = b""
        while not self.stop_reader.is_set() and self.port:
            try:
                chunk = self.port.read(256)
            except Exception as exc:
                self._fail(f"lost the port: {exc}. Replug the board and press Connect.")
                return
            if not chunk:
                continue
            buffer += chunk
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                text = raw.decode("utf-8", "replace").strip("\r\n ")
                if not text:
                    continue
                self.add(text, "rx")
                match = PULSE_RE.search(text)
                if match:
                    self.pulses = [int(match.group(i)) for i in (1, 2, 3)]
                match = XYZ_RE.search(text)
                if match:
                    self.xyz = [float(match.group(i)) for i in (1, 2, 3)]

    def send(self, line):
        if not self.port:
            raise RuntimeError("Serial port is not open.")
        line = line.strip()
        if not line:
            return
        self.port.write((line + "\n").encode("ascii", "ignore"))
        self.port.flush()
        self.add(line, "tx")

    def check_alive(self):
        """Notice an unplugged board even when nothing is being sent."""
        if not self.port:
            return
        now = time.time()
        if now - getattr(self, "_checked", 0) < 2.0:
            return
        self._checked = now
        devices = [p.device for p in serial.tools.list_ports.comports()]
        if self.name and self.name not in devices:
            self._fail(f"{self.name} disappeared (board unplugged or reset).")

    def snapshot(self, since):
        self.check_alive()
        with self.lock:
            lines = [entry for entry in self.log if entry["n"] > since]
        return {
            "connected": self.port is not None,
            "port": self.name,
            "pulses": self.pulses,
            "xyz": self.xyz,
            "counter": self.counter,
            "lines": lines,
        }


LINK = Link()


def read_json_file(path, default):
    if not os.path.isfile(path):
        return default
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json_file(path, data):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # keep the console for serial traffic only

    # ---------- helpers ----------
    def reply(self, obj, code=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def body_json(self):
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        chunks, remaining = [], length
        while remaining > 0:
            chunk = self.rfile.read(min(remaining, 1 << 20))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        return json.loads(b"".join(chunks).decode("utf-8"))

    def serve_file(self, relative):
        path = os.path.join(WEB, relative)
        if not os.path.isfile(path):
            self.send_error(404)
            return
        kind = "text/html" if path.endswith(".html") else "text/plain"
        with open(path, "rb") as handle:
            body = handle.read()
        self.send_response(200)
        self.send_header("Content-Type", f"{kind}; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    # ---------- routes ----------
    def do_GET(self):
        route = urlparse(self.path)
        query = parse_qs(route.query)
        if route.path in ("/", "/index.html"):
            return self.serve_file("index.html")
        if route.path == "/api/config":
            return self.reply(read_json_file(CONFIG_FILE, {}))
        if route.path == "/api/points":
            return self.reply(read_json_file(POINTS_FILE, {"points": []}))
        if route.path == "/api/sequences":
            return self.reply(read_json_file(SEQ_FILE, {"sequences": []}))
        if route.path == "/api/cad":
            return self.reply(read_json_file(CAD_FILE, {}))
        if route.path == "/api/vision":
            return self.reply(read_json_file(VISION_FILE, {"objects": {}}))
        if route.path == "/api/home":
            return self.reply(read_json_file(HOME_FILE, {}))
        if route.path == "/api/ports":
            ports = [{"device": p.device, "description": p.description}
                     for p in serial.tools.list_ports.comports()]
            return self.reply({"ports": ports})
        if route.path == "/api/state":
            since = int((query.get("since") or ["0"])[0])
            return self.reply(LINK.snapshot(since))
        self.send_error(404)

    def do_POST(self):
        route = urlparse(self.path)
        try:
            data = self.body_json()
            if route.path == "/api/connect":
                LINK.open(data.get("port", "COM3"), int(data.get("baud", 115200)))
                return self.reply({"ok": True, "port": LINK.name})
            if route.path == "/api/disconnect":
                LINK.close()
                return self.reply({"ok": True})
            if route.path == "/api/send":
                for line in data.get("lines", [data.get("line", "")]):
                    LINK.send(line)
                return self.reply({"ok": True})
            if route.path == "/api/points":
                store = read_json_file(POINTS_FILE, {"points": []})
                point = data.get("point")
                if data.get("action") == "delete":
                    store["points"] = [p for p in store["points"] if p["name"] != data.get("name")]
                else:
                    store["points"] = [p for p in store["points"] if p["name"] != point["name"]]
                    store["points"].append(point)
                write_json_file(POINTS_FILE, store)
                return self.reply(store)
            if route.path == "/api/arm/fit":
                cfg = read_json_file(CONFIG_FILE, {})
                fit = plan.fit_arm(cfg, data.get("touches", []))
                cfg["calibration"]["M1"] = fit["M1"]
                cfg["calibration"]["M2"] = fit["M2"]
                write_json_file(CONFIG_FILE, cfg)
                return self.reply({"ok": True, "fit": fit})
            if route.path == "/api/program":
                # Build a program from TAUGHT arm poses plus computed base pulses.
                # No M1/M2 calibration needed: the arm pose is reused and only the base moves.
                store = read_json_file(POINTS_FILE, {"points": []})
                by_name = {p["name"]: p for p in store["points"]}
                seen = read_json_file(VISION_FILE, {})
                home = read_json_file(HOME_FILE, {})
                cfg = read_json_file(CONFIG_FILE, {})
                base_cal = cfg["calibration"]["BASE"]
                us_per_deg = ((base_cal["pulses"][-1] - base_cal["pulses"][0]) /
                              (base_cal["angles_deg"][-1] - base_cal["angles_deg"][0]))
                lo, hi = cfg["limits"]["manualMin"][2], cfg["limits"]["manualMax"][2]

                rack = data["rack"]          # {firstPulse, lastPulse, slotCount}
                poses = data["poses"]        # {rackHover: name, rackGrip: name, ...}
                letters = data["letters"]    # [{letter, slot}]

                def slot_pulse(index):
                    # firstPulse belongs to slot firstIndex (default 1) and lastPulse to the
                    # last slot, so a rack measured at any two slots interpolates correctly.
                    first_index = rack.get("firstIndex", 1)
                    last_index = rack.get("lastIndex", rack["slotCount"])
                    span = last_index - first_index
                    if span == 0:
                        return rack["firstPulse"]
                    pitch = (rack["lastPulse"] - rack["firstPulse"]) / span
                    # baseTrim shifts every slot together, for dialling in the whole rack.
                    return rack["firstPulse"] + pitch * (index - first_index) + rack.get("baseTrim", 0)

                def azimuth_pulse(name):
                    obj = (seen.get("objects") or {}).get(name)
                    if not obj or not home.get("homePulse_us"):
                        return None
                    return home["homePulse_us"] + obj["azimuth_deg"] * us_per_deg

                steps, problems = [], []
                mode = data.get("mode", "letters")

                def emit(label, pose_key, base_pulse, note):
                    pose = by_name.get(poses.get(pose_key, ""))
                    if pose is None:
                        problems.append(f"{label}: no taught pose for {pose_key}")
                        return
                    if base_pulse is None:
                        problems.append(f"{label}: no position known")
                        return
                    pulse = int(round(base_pulse))
                    if not (lo <= pulse <= hi):
                        problems.append(f"{label}: base {pulse} us outside {lo}-{hi}")
                        return
                    steps.append({"name": label, "pulses": [pose["pulses"][0], pose["pulses"][1], pulse],
                                  "xyz": pose.get("xyz", [0, 0, 0]), "note": note,
                                  "fromPose": pose["name"]})

                if mode == "approach":
                    # Two taught poses -> a safe vertical-ish approach.
                    # All sideways motion happens at the hover pose; the descent interpolates
                    # between hover and contact with the base held at the contact value.
                    hover = by_name.get(poses.get("hover", ""))
                    contact = by_name.get(poses.get("contact", ""))
                    start = by_name.get(poses.get("start", ""))
                    if hover is None or contact is None:
                        return self.reply({"ok": False,
                                           "error": "Teach and pick both a hover pose and a contact pose."}, 400)
                    count = max(1, int(data.get("steps", 4)))
                    label = data.get("name", "approach")
                    rungs = []
                    for i in range(1, count + 1):
                        fraction = i / (count + 1)
                        rungs.append({
                            "name": f"{label}_down_{i:02d}",
                            "pulses": [int(round(hover["pulses"][k] + fraction * (contact["pulses"][k] - hover["pulses"][k])))
                                       for k in (0, 1)] + [contact["pulses"][2]],
                            "note": f"{int(fraction*100)}% of the way down",
                        })
                    top = {"name": f"{label}_hover", "note": "over the target, at height",
                           "pulses": [hover["pulses"][0], hover["pulses"][1], contact["pulses"][2]]}
                    bottom = {"name": f"{label}_contact", "note": "contact", "pulses": list(contact["pulses"])}
                    made = [top] + rungs + [bottom]
                    for item in made:
                        item["xyz"] = [0, 0, 0]
                        item["kind"] = "approach"
                        store["points"] = [p for p in store["points"] if p["name"] != item["name"]]
                        store["points"].append(item)
                    write_json_file(POINTS_FILE, store)

                    order = []
                    if start is not None:
                        order.append(start["name"])
                    order += [item["name"] for item in made]
                    order += [item["name"] for item in reversed(made[:-1])]
                    if start is not None:
                        order.append(start["name"])
                    sequences = read_json_file(SEQ_FILE, {"sequences": []})
                    name = data.get("sequenceName") or label
                    sequences["sequences"] = [q for q in sequences["sequences"] if q["name"] != name]
                    sequences["sequences"].append({"name": name, "steps": order, "dwell": 2500})
                    write_json_file(SEQ_FILE, sequences)
                    return self.reply({"ok": True, "saved": name,
                                       "steps": [{"name": n, "pulses": by_name.get(n, {}).get("pulses")
                                                  or next(i["pulses"] for i in made if i["name"] == n),
                                                  "note": "", "fromPose": ""} for n in order]})

                if mode == "hoverTour":
                    # Hover over a list of slots at one taught arm pose, starting and ending
                    # at the home anchor. Nothing descends; this is for checking azimuths.
                    home_pulse = home.get("homePulse_us")
                    if home_pulse:
                        emit("tour_home_start", "rackHover", home_pulse, "home, arm at hover height")
                    for index in data.get("slots", []):
                        emit(f"tour_slot{index}", "rackHover", slot_pulse(index), f"hover over slot {index}")
                    if home_pulse:
                        emit("tour_home_end", "rackHover", home_pulse, "back to home")
                    result = {"steps": steps, "problems": problems, "ok": not problems}
                    if data.get("save") and not problems:
                        names = []
                        for step in steps:
                            point = {"name": step["name"], "kind": "hover", "pulses": step["pulses"],
                                     "xyz": step["xyz"], "note": step["note"]}
                            store["points"] = [p for p in store["points"] if p["name"] != step["name"]]
                            store["points"].append(point)
                            names.append(step["name"])
                        write_json_file(POINTS_FILE, store)
                        sequences = read_json_file(SEQ_FILE, {"sequences": []})
                        label = data.get("sequenceName") or "hover tour"
                        sequences["sequences"] = [q for q in sequences["sequences"] if q["name"] != label]
                        sequences["sequences"].append({"name": label, "steps": names, "dwell": 2500})
                        write_json_file(SEQ_FILE, sequences)
                        result["saved"] = label
                    return self.reply(result)

                if mode == "stampMove":
                    # Rack only: lift a stamp out of one slot and seat it in another.
                    # Needs no vision and no ink/paper poses.
                    for move in data.get("moves", []):
                        a, b = move["from"], move["to"]
                        tag = f"slot{a}to{b}"
                        emit(f"{tag}_hover_a", "rackHover", slot_pulse(a), f"above slot {a}")
                        emit(f"{tag}_grip", "rackGrip", slot_pulse(a), "magnet on the disc")
                        emit(f"{tag}_lift_a", "rackHover", slot_pulse(a), "lift the stamp clear")
                        emit(f"{tag}_hover_b", "rackHover", slot_pulse(b), f"across to slot {b}")
                        emit(f"{tag}_seat", "rackGrip", slot_pulse(b), f"lower into slot {b}")
                        emit(f"{tag}_release", "rackRelease", slot_pulse(b), "slide out, magnet lets go")
                        emit(f"{tag}_clear", "rackHover", slot_pulse(b), "back to hover")
                    result = {"steps": steps, "problems": problems, "ok": not problems}
                    if data.get("save") and not problems:
                        names = []
                        for step in steps:
                            point = {"name": step["name"], "kind": "program", "pulses": step["pulses"],
                                     "xyz": step["xyz"], "note": step["note"]}
                            store["points"] = [p for p in store["points"] if p["name"] != step["name"]]
                            store["points"].append(point)
                            names.append(step["name"])
                        write_json_file(POINTS_FILE, store)
                        sequences = read_json_file(SEQ_FILE, {"sequences": []})
                        label = data.get("sequenceName") or "stamp move"
                        sequences["sequences"] = [q for q in sequences["sequences"] if q["name"] != label]
                        sequences["sequences"].append({"name": label, "steps": names, "dwell": 2200})
                        write_json_file(SEQ_FILE, sequences)
                        result["saved"] = label
                    return self.reply(result)

                for item in letters:
                    letter, index = item["letter"], item["slot"]
                    sp = slot_pulse(index)
                    emit(f"{letter}_slot_hover", "rackHover", sp, f"above slot {index}")
                    emit(f"{letter}_slot_grip", "rackGrip", sp, "magnet on the disc")
                    emit(f"{letter}_slot_lift", "rackHover", sp, "lift clear")
                    emit(f"{letter}_ink_hover", "inkHover", azimuth_pulse("ink"), "over the ink")
                    emit(f"{letter}_ink_press", "inkPress", azimuth_pulse("ink"), "press the ink")
                    emit(f"{letter}_ink_lift", "inkHover", azimuth_pulse("ink"), "")
                    paper = azimuth_pulse("paper")
                    if paper is not None:
                        paper += item.get("paperOffsetDeg", 0.0) * us_per_deg
                    emit(f"{letter}_paper_hover", "paperHover", paper, "over the paper mark")
                    emit(f"{letter}_paper_press", "paperPress", paper, "press on paper")
                    emit(f"{letter}_paper_lift", "paperHover", paper, "")
                    emit(f"{letter}_return_hover", "rackHover", sp, "back over its slot")
                    emit(f"{letter}_return_seat", "rackGrip", sp, "seat the stamp")
                    emit(f"{letter}_release", "rackRelease", sp, "slide out, magnet lets go")

                result = {"steps": steps, "problems": problems, "ok": not problems}
                if data.get("save") and not problems:
                    names = []
                    for step in steps:
                        point = {"name": step["name"], "kind": "program", "pulses": step["pulses"],
                                 "xyz": step["xyz"], "note": step["note"]}
                        store["points"] = [p for p in store["points"] if p["name"] != step["name"]]
                        store["points"].append(point)
                        names.append(step["name"])
                    write_json_file(POINTS_FILE, store)
                    sequences = read_json_file(SEQ_FILE, {"sequences": []})
                    label = data.get("sequenceName") or "program"
                    sequences["sequences"] = [q for q in sequences["sequences"] if q["name"] != label]
                    sequences["sequences"].append({"name": label, "steps": names, "dwell": 2200})
                    write_json_file(SEQ_FILE, sequences)
                    result["saved"] = label
                return self.reply(result)
            if route.path == "/api/plan":
                cfg = read_json_file(CONFIG_FILE, {})
                seen = read_json_file(VISION_FILE, {})
                home = read_json_file(HOME_FILE, {})
                if not home.get("homePulse_us"):
                    return self.reply({"ok": False, "error": "Set the home anchor first."}, 400)
                result = plan.plan_letters(cfg, seen, home, data["letters"], data["slots"], data["heights"])
                if data.get("save") and result["allReachable"]:
                    store = read_json_file(POINTS_FILE, {"points": []})
                    names = []
                    for step in result["steps"]:
                        point = {"name": step["name"], "kind": "planned", "pulses": step["pulses"],
                                 "xyz": step["xyz"], "planned": True, "note": step.get("note", "")}
                        store["points"] = [p for p in store["points"] if p["name"] != step["name"]]
                        store["points"].append(point)
                        names.append(step["name"])
                    write_json_file(POINTS_FILE, store)
                    sequences = read_json_file(SEQ_FILE, {"sequences": []})
                    label = data.get("sequenceName") or "planned program"
                    sequences["sequences"] = [q for q in sequences["sequences"] if q["name"] != label]
                    sequences["sequences"].append({"name": label, "steps": names, "dwell": 2200})
                    write_json_file(SEQ_FILE, sequences)
                    result["saved"] = label
                return self.reply(result)
            if route.path == "/api/vision/point":
                stored = read_json_file(VISION_FILE, {})
                if "homography" not in stored:
                    return self.reply({"ok": False, "error": "Read a photo first."}, 400)
                name = data.get("name", "point")
                xy = vision.pixel_to_robot(stored, float(data["x"]), float(data["y"]))
                entry = {"xy_mm": [round(xy[0], 2), round(xy[1], 2)],
                         "radius_mm": round((xy[0] ** 2 + xy[1] ** 2) ** 0.5, 2),
                         "azimuth_deg": round(math.degrees(math.atan2(xy[1], xy[0])), 2),
                         "clicked": True}
                cfg = read_json_file(CONFIG_FILE, {})
                target = (cfg.get("vision", {}).get("taggedTargets", {}) or {}).get(name)
                if target:
                    offset = vision.offset_in_tag_frame(stored, target["tagId"], xy)
                    if offset:
                        height = target.get("offsetTag_mm", [0, 0, 0])[2]
                        target["offsetTag_mm"] = [round(offset[0], 3), round(offset[1], 3), height]
                        target["source"] = "clicked on the dashboard photo"
                        write_json_file(CONFIG_FILE, cfg)
                        entry["fromTag"] = target["tagId"]
                        entry["offsetTag_mm"] = target["offsetTag_mm"]
                stored.setdefault("objects", {})[name] = entry
                write_json_file(VISION_FILE, stored)
                return self.reply({"ok": True, "point": entry})
            if route.path == "/api/home":
                write_json_file(HOME_FILE, data)
                return self.reply({"ok": True, "home": data})
            if route.path == "/api/vision":
                if vision is None:
                    return self.reply({"ok": False,
                                       "error": "OpenCV is not available: " + VISION_IMPORT_ERROR}, 400)
                raw = data.get("image", "")
                if raw:
                    if "," in raw:
                        raw = raw.split(",", 1)[1]
                    image_bytes = base64.b64decode(raw)
                    os.makedirs(UPLOAD_DIR, exist_ok=True)
                    with open(LAST_PHOTO, "wb") as handle:   # keep it so we can re-read later
                        handle.write(image_bytes)
                elif os.path.isfile(LAST_PHOTO):
                    image_bytes = open(LAST_PHOTO, "rb").read()
                else:
                    return self.reply({"ok": False, "error": "No photo sent and none stored."}, 400)
                result = vision.analyse(image_bytes,
                                        read_json_file(CONFIG_FILE, {}),
                                        read_json_file(CAD_FILE, {}))
                stored = dict(result)
                stored.pop("preview", None)      # keep vision.json small and readable
                write_json_file(VISION_FILE, stored)
                return self.reply(result)
            if route.path == "/api/sequences":
                store = read_json_file(SEQ_FILE, {"sequences": []})
                name = data.get("name") or (data.get("sequence") or {}).get("name")
                store["sequences"] = [q for q in store["sequences"] if q["name"] != name]
                if data.get("action") != "delete":
                    store["sequences"].append(data["sequence"])
                write_json_file(SEQ_FILE, store)
                return self.reply(store)
        except Exception as exc:
            return self.reply({"ok": False, "error": str(exc)}, 400)
        self.send_error(404)


def main():
    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    url = f"http://127.0.0.1:{PORT}"
    print(f"RASTICrane dashboard on {url}")
    print("Ctrl+C here stops the server. It never sends a command on its own.")
    try:
        webbrowser.open(url)
    except Exception:
        pass
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down")
    finally:
        LINK.close()


if __name__ == "__main__":
    main()
