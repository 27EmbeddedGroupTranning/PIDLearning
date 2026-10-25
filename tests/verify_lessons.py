"""Compile independent lesson presets without changing students' src/tuning.cpp."""
import csv
import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import lab

PRESETS = [
    ("P", "0.8, 0, 0, 0.65, 0.025, true", 45, 20, 0.08, False),
    ("PI", "0.8, 0.45, 0, 0.65, 0.025, true", 45, 20, 0.08, False),
    ("PID", "0.8, 0.45, 0.22, 0.65, 0.025, true", 45, 20, 0.08, False),
    ("windup_OFF", "0.8, 0.45, 0.22, 0.08, 0.025, false", 120, 20, 0, True),
    ("windup_ON", "0.8, 0.45, 0.22, 0.08, 0.025, true", 120, 20, 0, True),
]


def main():
    lab.BUILD.mkdir(exist_ok=True)
    metrics = []
    for name, gains, yaw, pitch, load, reverse in PRESETS:
        source = lab.BUILD / f"preset_{name}.cpp"
        source.write_text('#include "tuning.hpp"\n' +
                          f'Gains yaw_gains() {{ return {{{gains}}}; }}\n' +
                          f'Gains pitch_gains() {{ return {{{gains}}}; }}\n', encoding="utf-8")
        exe = lab.BUILD / (f"lesson_{name}" + (".exe" if lab.os.name == "nt" else ""))
        lab.command([lab.compiler(), "-std=c++11", "-O2", "-I", str(lab.ROOT / "include"),
                     str(lab.ROOT / "src/main.cpp"), str(source), "-o", str(exe)])
        output = lab.command([str(exe), str(yaw), str(pitch), str(load), str(int(reverse))])
        (lab.BUILD / f"lesson_{name}.csv").write_text(output, encoding="utf-8")
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(io.StringIO(output))]
        step = [r for r in rows if 1 <= r["time"] < 6]
        tail = [r for r in rows if r["time"] >= 14]
        overshoot = max(0, max(r["yaw"] - yaw for r in step)) / yaw * 100
        mae = sum(abs(r["yaw_target"] - r["yaw"]) for r in tail) / len(tail)
        at_reverse = next(r for r in rows if r["time"] == 8)
        metrics.append((name, overshoot, mae, at_reverse["yaw_u"]))
        print(f"{name:12s} overshoot={overshoot:6.2f}%  final_MAE={mae:6.2f}deg  u_at_8s={at_reverse['yaw_u']:+.4f}")
    assert metrics[0][2] > 4, "P should retain a load error"
    assert metrics[2][2] < 1, "PID should reject the load"
    assert metrics[2][1] < metrics[1][1], "D should reduce overshoot for these presets"
    assert metrics[3][2] > 50 and metrics[4][2] < 2, "anti-windup should improve reversal recovery"
    print("Lesson comparisons passed. CSV files saved under build/.")


if __name__ == "__main__":
    main()
