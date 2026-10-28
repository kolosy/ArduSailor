#!/usr/bin/env python3
"""Builds the ArduSailor firmware as a native program for the simulator.

The firmware in ../firmware is copied into build/fw and compiled unchanged, apart from three things:
  - config.h is switched to the sailing build (PILOT_DEBUG and NO_SAIL off),
  - the #defines listed in TUNABLES become variables, so the simulator UI can change them while it runs,
  - the .ino files are joined into one sketch.cpp with prototypes, the way the Arduino IDE does it.
It then links that against the fake Arduino layer in native/arduino and the simulation engine in native/.

Usage: python3 sim/build.py [--force] [--cxx clang++]
"""

import argparse
import concurrent.futures
import hashlib
import os
import re
import shutil
import subprocess
import sys

SIM = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SIM)
FIRMWARE = os.path.join(ROOT, "firmware")
NATIVE = os.path.join(SIM, "native")
THIRD_PARTY = os.path.join(SIM, "third_party")
BUILD = os.path.join(SIM, "build")
BINARY = os.path.join(BUILD, "ardusailor-sim")

# Firmware #defines the simulator can change at run time: (name, min, max, takes effect only at restart).
# Array sizes become ARRAY_TUNABLE_MAX, and the value stays the number of entries used.
TUNABLES = [
    ("IRONS", 10, 90, False),
    ("TACK_EVERY", 1000, 600000, False),
    ("GET_WITHIN", 1, 50, False),
    ("SAIL_ADJUST_ON", 0, 60, False),
    ("START_HEEL_COMP", 0, 90, False),
    ("MAX_HEEL_COMP", 0, 64, False),
    ("MIN_HEEL_COMP", 0, 64, False),
    ("HEEL_COMP_DECAY", 0, 1, False),
    ("RUDDER_MIN", 0, 180, False),
    ("RUDDER_MAX", 0, 180, False),
    ("WINCH_MAX", 0, 180, False),
    ("WINCH_MIN", 0, 180, False),
    ("RUDDER_SPEED", 10, 2000, False),
    ("WINCH_SPEED", 1, 500, False),
    ("AHRS_TRAIL", 1, 64, False),
    ("WIND_TRAIL", 1, 64, False),
    ("DATA_FREQ", 100, 60000, False),
    ("MPU_LOOPS", 1, 20, False),
    ("TOTAL_CALIBRATION_STEPS", 10, 3000, False),
    ("MAG_SAMPLE_DELAY", 0, 31, True),
    ("MAG_INCLINATION", -90, 90, False),
    ("MAG_DECLINATION", -30, 30, True),
    ("SENSOR_OFFSET", 0, 360, False),
    ("WIND_ITERATIONS", 1, 20, False),
]
ARRAY_TUNABLE_MAX = 64

FIRMWARE_EXTENSIONS = (".ino", ".cpp", ".c", ".h")
DEFINE_RE = r"^[ \t]*#define[ \t]+{name}[ \t]+([-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?)[ \t]*(//.*)?$"


def log(msg):
    print(f"[build] {msg}", flush=True)


def inputs_digest():
    h = hashlib.sha256()
    for base in (FIRMWARE, NATIVE, THIRD_PARTY):
        for dirpath, _, files in sorted(os.walk(base)):
            for f in sorted(files):
                path = os.path.join(dirpath, f)
                h.update(os.path.relpath(path, ROOT).encode())
                with open(path, "rb") as fh:
                    h.update(fh.read())
    with open(__file__, "rb") as fh:
        h.update(fh.read())
    return h.hexdigest()


def copy_firmware(fw):
    shutil.rmtree(fw, ignore_errors=True)
    os.makedirs(fw)
    for f in sorted(os.listdir(FIRMWARE)):
        if f.endswith(FIRMWARE_EXTENSIONS):
            shutil.copy(os.path.join(FIRMWARE, f), fw)


def configure(fw):
    path = os.path.join(fw, "config.h")
    with open(path) as fh:
        text = fh.read()
    for switch in ("PILOT_DEBUG", "NO_SAIL"):
        text, n = re.subn(rf"^[ \t]*#define[ \t]+{switch}\b.*$",
                          f"// {switch} switched off by the simulator build (sailing configuration)", text, flags=re.M)
        if n:
            log(f"config.h: {switch} off")
    with open(path, "w") as fh:
        fh.write(text)


def comment_above(lines, index):
    """The // comment block directly above a line, as one sentence."""
    out = []
    i = index - 1
    while i >= 0 and lines[i].strip().startswith("//"):
        out.insert(0, lines[i].strip().lstrip("/").strip())
        i -= 1
    return " ".join(out)


def make_tunables(fw):
    """Turn the TUNABLES #defines into variables; returns their descriptions."""
    found = []
    files = sorted(f for f in os.listdir(fw) if f.endswith(FIRMWARE_EXTENSIONS))
    for name, lo, hi, restart in TUNABLES:
        pattern = re.compile(DEFINE_RE.format(name=name), re.M)
        hits = []
        for f in files:
            path = os.path.join(fw, f)
            with open(path) as fh:
                text = fh.read()
            m = pattern.search(text)
            if not m:
                continue
            literal = m.group(1)
            is_int = re.fullmatch(r"[-+]?\d+", literal) is not None
            ctype = "int" if is_int else "double"
            lines = text[: m.start()].split("\n")
            comment = comment_above(text.split("\n"), len(lines) - 1) or (m.group(2) or "").lstrip("/ ").strip()
            replacement = f"extern {ctype} sim_{name};\n#define {name} (sim_{name})"
            text = text[: m.start()] + replacement + text[m.end():]
            # a #define used as an array size can't become a variable: size the array for the maximum
            text, arrays = re.subn(rf"\[\s*{name}\s*\]", f"[{ARRAY_TUNABLE_MAX}]", text)
            if arrays:
                hi = min(hi, ARRAY_TUNABLE_MAX)
            with open(path, "w") as fh:
                fh.write(text)
            hits.append(dict(name=name, file=f, comment=comment, ctype=ctype, value=literal,
                             min=lo, max=hi, restart=restart))
        if len(hits) != 1:
            sys.exit(f"[build] expected one #define {name} with a numeric value in the firmware, found {len(hits)}")
        found.extend(hits)
    return found


def write_tunables_cpp(tunables, path):
    def c_string(s):
        return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'

    out = ['// generated by build.py: the firmware #defines the simulator can change', '#include "sim_api.h"', ""]
    for t in tunables:
        out.append(f"{t['ctype']} sim_{t['name']} = {t['value']};")
    out += ["", "extern \"C\" const sim_tunable sim_tunables[] = {"]
    for t in tunables:
        out.append(f"    {{{c_string(t['name'])}, {c_string(t['file'])}, {c_string(t['comment'])}, "
                   f"{1 if t['ctype'] == 'int' else 0}, {1 if t['restart'] else 0}, (void *)&sim_{t['name']}, "
                   f"{t['value']}, {t['min']}, {t['max']}}},")
    out += ["};", f"extern \"C\" const int sim_tunable_count = {len(tunables)};", ""]
    with open(path, "w") as fh:
        fh.write("\n".join(out))


FUNC_RE = re.compile(
    r"^((?:static\s+|inline\s+)*)([A-Za-z_][\w\s\*&]*?[\s\*&])([A-Za-z_]\w*)\s*\(([^;{}()]*)\)\s*\{", re.M)


def strip_comments(text):
    text = re.sub(r"/\*.*?\*/", lambda m: re.sub(r"[^\n]", " ", m.group(0)), text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def prototypes(text):
    protos = []
    for m in FUNC_RE.finditer(strip_comments(text)):
        qualifiers, ret, name, args = m.group(1), m.group(2).strip(), m.group(3), m.group(4).strip()
        if name in ("if", "while", "for", "switch") or ret in ("else", "return", "new", "delete"):
            continue
        protos.append(f"{qualifiers}{ret} {name}({args});")
    return protos


def insertion_line(lines):
    """After the main sketch file's #include block (and any #if around an include), like the Arduino IDE."""
    depth, where = 0, 0
    for i, line in enumerate(lines):
        s = line.strip()
        if re.match(r"#\s*if", s):
            depth += 1
        elif re.match(r"#\s*endif", s):
            depth -= 1
            if depth == 0 and where == -1:
                where = i + 1
        elif re.match(r"#\s*include", s):
            where = i + 1 if depth == 0 else -1
    return max(where, 0)


def make_sketch(fw, path):
    main = "firmware.ino"
    inos = [main] + sorted(f for f in os.listdir(fw) if f.endswith(".ino") and f != main)
    texts = {}
    for f in inos:
        with open(os.path.join(fw, f)) as fh:
            texts[f] = fh.read()
    protos = []
    for f in inos:
        protos += prototypes(texts[f])

    main_lines = texts[main].split("\n")
    at = insertion_line(main_lines)
    out = ['#include "Arduino.h"', f'#line 1 "{main}"']
    out += main_lines[:at]
    out += ["// prototypes, as generated by the Arduino IDE"] + protos
    out += [f'#line {at + 1} "{main}"'] + main_lines[at:]
    for f in inos[1:]:
        out += [f'#line 1 "{f}"', texts[f]]
    out += ['#line 1 "sketch_bridge.inc"', '#include "sketch_bridge.inc"', ""]
    with open(path, "w") as fh:
        fh.write("\n".join(out))
    log(f"sketch.cpp from {', '.join(inos)} ({len(protos)} prototypes)")


def compile_all(cxx, cc, fw):
    obj = os.path.join(BUILD, "obj")
    shutil.rmtree(obj, ignore_errors=True)
    os.makedirs(obj)
    arduino = os.path.join(NATIVE, "arduino")
    pid = os.path.join(THIRD_PARTY, "PID")
    autotune = os.path.join(THIRD_PARTY, "PID_AutoTune")
    common = ["-O2", "-g", "-pthread"]
    firmware_flags = common + ["-std=gnu++11", "-w", "-DARDUINO=10819", "-DARDUINO_SIM",
                               f"-I{fw}", f"-I{arduino}", f"-I{NATIVE}", f"-I{pid}", f"-I{autotune}"]
    engine_flags = common + ["-std=c++11", "-Wall", "-Wextra", f"-I{NATIVE}"]

    jobs = []
    for src in [os.path.join(BUILD, "sketch.cpp"), os.path.join(BUILD, "tunables.cpp"),
                os.path.join(arduino, "arduino_core.cpp"),
                os.path.join(pid, "PID_v1.cpp"), os.path.join(autotune, "PID_AutoTune_v0.cpp")] + \
            [os.path.join(fw, f) for f in sorted(os.listdir(fw)) if f.endswith(".cpp")]:
        jobs.append([cxx] + firmware_flags + ["-c", src])
    for src in sorted(os.listdir(fw)):
        if src.endswith(".c"):
            jobs.append([cc, "-O2", "-g", "-w", "-std=gnu11", f"-I{fw}", "-c", os.path.join(fw, src)])
    for src in sorted(os.listdir(NATIVE)):
        if src.endswith(".cpp"):
            jobs.append([cxx] + engine_flags + ["-c", os.path.join(NATIVE, src)])

    objects = []

    def run(cmd):
        src = cmd[-1]
        out = os.path.join(obj, os.path.basename(os.path.dirname(src)) + "_" + os.path.basename(src) + ".o")
        objects.append(out)
        res = subprocess.run(cmd + ["-o", out], capture_output=True, text=True)
        return src, res

    failed = False
    with concurrent.futures.ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as pool:
        for src, res in pool.map(run, jobs):
            if res.returncode != 0 or res.stderr.strip():
                sys.stderr.write(res.stderr)
            if res.returncode != 0:
                log(f"FAILED: {os.path.relpath(src, ROOT)}")
                failed = True
    if failed:
        sys.exit(1)

    res = subprocess.run([cxx] + common + sorted(objects) + ["-o", BINARY], capture_output=True, text=True)
    if res.returncode != 0:
        sys.stderr.write(res.stderr)
        sys.exit("[build] link failed")


def build(force=False, cxx=None, cc=None):
    cxx = cxx or os.environ.get("CXX", "c++")
    cc = cc or os.environ.get("CC", "cc")
    digest = inputs_digest() + cxx + cc
    stamp = os.path.join(BUILD, ".inputs")
    if not force and os.path.exists(BINARY) and os.path.exists(stamp):
        with open(stamp) as fh:
            if fh.read() == digest:
                log("up to date")
                return BINARY

    os.makedirs(BUILD, exist_ok=True)
    fw = os.path.join(BUILD, "fw")
    copy_firmware(fw)
    configure(fw)
    tunables = make_tunables(fw)
    write_tunables_cpp(tunables, os.path.join(BUILD, "tunables.cpp"))
    log(f"{len(tunables)} firmware settings made adjustable")
    make_sketch(fw, os.path.join(BUILD, "sketch.cpp"))
    log(f"compiling with {cxx}")
    compile_all(cxx, cc, fw)
    with open(stamp, "w") as fh:
        fh.write(digest)
    log(f"built {os.path.relpath(BINARY, ROOT)}")
    return BINARY


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--force", action="store_true", help="rebuild even if nothing changed")
    ap.add_argument("--cxx", help="C++ compiler (default: $CXX or c++)")
    ap.add_argument("--cc", help="C compiler (default: $CC or cc)")
    args = ap.parse_args()
    build(args.force, args.cxx, args.cc)
