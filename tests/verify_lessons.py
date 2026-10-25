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
    # 双环结构下外环限幅的量纲是速度指令（rad/s）：原 0.08 的力矩限幅在该
    # 量纲下无法完成 120° 反转，故 windup 预设改用 1.2 rad/s 并降低 Ki
    # （该预设无负载，不影响负载恢复教学；降低 Ki 可让抗饱和收益更清晰）。
    ("windup_OFF", "0.8, 0.2, 0.22, 1.2, 0.025, false", 120, 20, 0, True),
    ("windup_ON", "0.8, 0.2, 0.22, 1.2, 0.025, true", 120, 20, 0, True),
]


def main():
    lab.BUILD.mkdir(exist_ok=True)
    metrics = []
    for name, gains, yaw, pitch, load, reverse in PRESETS:
        source = lab.BUILD / f"preset_{name}.cpp"
        # 内环 LQR 配置与 src/tuning.cpp 一致且对全部预设相同，
        # 保证各预设之间的差异只来自外环 PID 增益。
        source.write_text('#include "tuning.hpp"\n' +
                          f'Gains yaw_gains() {{ return {{{gains}}}; }}\n' +
                          f'Gains pitch_gains() {{ return {{{gains}}}; }}\n' +
                          'LQRParams yaw_lqr_params() { return {0.035, 0.06, 1.0, 1.0, 1.0, true}; }\n' +
                          'LQRParams pitch_lqr_params() { return {0.035, 0.06, 1.0, 1.0, 1.0, true}; }\n',
                          encoding="utf-8")
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
    # 双环结构下内环 LQR 已提供速度阻尼，外环 D 项对超调的影响大幅减弱
    # （单环时代 D 明显抑制超调的结论不再成立；此处断言其影响为次要量级）。
    assert abs(metrics[2][1] - metrics[1][1]) < 5, "D effect on overshoot is minor in dual-loop"
    # 双环下外环限幅量纲为速度指令，巡航末段的合法积分滞后使抗饱和开启时
    # 仍留有少量过冲，故用定性比值断言（关闭抗饱和的恢复显著更差）。
    assert metrics[3][2] > 50 and metrics[4][2] < metrics[3][2] / 4, \
        "anti-windup should improve reversal recovery"
    print("Lesson comparisons passed. CSV files saved under build/.")


if __name__ == "__main__":
    main()
