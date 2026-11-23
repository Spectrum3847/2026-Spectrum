#!/usr/bin/env python3
"""Check the generated phoenix6 tables against CTRE's Phoenix 6 Java sources.

Compares every config group (SPN, serializer type, default, order) and every
SPN constant. Downloads the Java sources jar for PHOENIX into a temp dir.
"""
import io, pathlib, re, sys, tempfile, urllib.request, zipfile

PHOENIX = "26.3.0"
URL = ("https://maven.ctr-electronics.com/release/com/ctre/phoenix6/wpiapi-java/"
       f"{PHOENIX}/wpiapi-java-{PHOENIX}-sources.jar")
src = pathlib.Path(__file__).resolve().parent.parent / "crates/phoenix6/src"

with tempfile.TemporaryDirectory() as tmp:
    zipfile.ZipFile(io.BytesIO(urllib.request.urlopen(URL).read())).extractall(tmp)
    java = pathlib.Path(tmp) / "com/ctre/phoenix6"
    problems = 0

    spn_java = dict(re.findall(r"^\s*(\w+)\((\d+)\)", (java / "spns/SpnValue.java").read_text(), re.M))
    spn_rust = dict(re.findall(r"pub const (\w+): i32 = (\d+);", (src / "spn.rs").read_text()))
    for name, value in spn_rust.items():
        if spn_java.get(name) != value:
            print(f"SPN {name}: java {spn_java.get(name)} rust {value}")
            problems += 1

    rust = (src / "configs/groups.rs").read_text()
    fields_checked = 0
    for m in re.finditer(r"pub struct (\w+) \{(.*?)\n    \}", rust, re.S):
        name = m.group(1)
        fields = re.findall(r"^\s*(\w+) / with_\w+: ([\w:]+) = ([^=]+?) => (\w+),", m.group(2), re.M)
        text = (java / f"configs/{name}.java").read_text()
        ser = re.search(r"public final String serialize\(\) \{(.*?)return", text, re.S).group(1)
        jser = re.findall(r"ConfigJNI\.Serialize(\w+)\(SpnValue\.(\w+)\.value, (\w+)(\.value)?\)", ser)
        jdef = dict(re.findall(r"public (?:[\w.]+) (\w+) = ([^;]+);", text))
        if len(jser) != len(fields):
            print(f"COUNT {name}: java {len(jser)} rust {len(fields)}")
            problems += 1
        for (jtype, spn, jfield, is_enum), (_, rtype, rdef, rspn) in zip(jser, fields):
            fields_checked += 1
            want = {"double": "f64", "boolean": "bool"}.get(jtype)
            jd, rd = jdef.get(jfield, "?").strip(), rdef.strip()
            if want == "f64":
                same = float(jd.rstrip("dD")) == float(rd)
            elif is_enum:
                same = jd.split(".")[-1] == rd.split("::")[-1]
            else:
                same = jd == rd
            if spn != rspn or (want and want != rtype) or not same:
                print(f"FIELD {name}.{jfield}: java {spn} {jtype} {jd} rust {rspn} {rtype} {rd}")
                problems += 1

print(f"spn constants {len(spn_rust)}, config fields {fields_checked}, problems {problems}")
sys.exit(1 if problems else 0)
