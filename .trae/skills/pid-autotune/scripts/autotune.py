#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
autotune.py — PID 自动整定工具：阶跃辨识 + 继电自整定 + Z-N 表 + 闭环验证
纯 Python 标准库（3.9+），无第三方依赖。结果输出到 stdout。

用法
====
1) 传递函数模式（系数降幂排列，常数项最后；输出限幅内任意 SISO 线性对象）:
   python autotune.py --num 1 --den 0.035,0.06,0

2) 自定义对象模块模式（非线性对象 / 已有仿真代码）:
   python autotune.py --plant-script my_plant.py

   模块契约（全部可选，缺省时用内置参考值）:
       DT     = 0.002      # 控制周期 s
       UMAX   = 0.65       # 控制量限幅（绝对值）
       DTAU   = 0.025      # 微分低通时间常数 s
       AW     = True       # 条件积分抗饱和
       TARGET = 0.7854     # 验证场景设定值（输出量单位）
       LOAD   = 0.08       # 负载扰动幅值（进入对象输入通道）
       LOAD_T = 6.0        # 负载加入时刻 s
       STEP_T = 1.0        # 设定值阶跃时刻 s
       T_END  = 15.0       # 验证总时长 s
       H      = 0.25       # 继电幅值
       EPS    = 0.005      # 继电滞环（输出量单位）
       def make_state(): ...                 # 返回初始状态（保证 y=0 为工作平衡点）
       def plant_step(state, u, dist, dt):   # 就地推进一个 dt，u 为控制量，dist 为扰动
       def measure(state): return y          # 被控量
       def make_pid(kp, ki, kd, umax, dtau, aw, dt):  # 可选：用用户自己的控制器替换参考 PID
           return 控制器  # 需实现 .update(target, y) -> u 和 .reset()

3) 硬件实测 Ku / Tu 直接查表并验证:
   python autotune.py --plant-script my_plant.py --ku 7.4 --tu 0.44

常用选项
========
--mode step|relay|tables|all   分步或全流程（默认 all）
--h --eps                      继电幅值 / 滞环
--dt --umax --dtau --aw 0|1    控制器与执行器参数
--target --load --load-t --step-t --T   验证场景（输出量单位）
--current kp,ki,kd             把用户现有参数加入验证对照
--invert                       执行器正方向与被控量正方向相反时使用
--open-t --u0                  开环阶跃辨识的时长与幅值
"""
import argparse
import math
import sys


# ---------------------------------------------------------------- 对象封装
class TFPlant:
    """线性传递函数，可控标准型 + 显式欧拉积分（与常见嵌入式仿真一致）。

    den 首系数归一。状态 x=[x1..xn]：x1'=x2, ..., xn' = -(Σ a_i x)+u, y=Σ b_i x_i。
    """

    def __init__(self, num, den):
        if not num or not den or abs(den[0]) < 1e-12:
            raise ValueError("den 不能为空且最高次系数不能为 0")
        if len(num) > len(den):
            raise ValueError("目前仅支持严格真分式（num 阶次 < den 阶次）")
        d0 = den[0]
        self.a = [c / d0 for c in den[1:]]          # a[i] = d_{i+1}
        rev = [c / d0 for c in reversed(num)]        # rev[i] = s^i 系数
        self.b = rev + [0.0] * (len(self.a) - len(rev))
        self.n = len(self.a)

    def make_state(self):
        return [0.0] * self.n

    def plant_step(self, x, u, dist, dt):
        a, n = self.a, self.n
        acc = u + dist
        for j in range(n):
            acc -= a[n - 1 - j] * x[j]
        for j in range(n - 1):
            x[j] += x[j + 1] * dt
        x[n - 1] += acc * dt

    def measure(self, x):
        return sum(bi * xi for bi, xi in zip(self.b, x))


class ScriptPlant:
    """用户 plant 模块封装。"""

    def __init__(self, mod):
        for fn in ("make_state", "plant_step", "measure"):
            if not hasattr(mod, fn):
                raise ValueError(f"plant 模块缺少 {fn}()")
        self.mod = mod

    def make_state(self):
        return self.mod.make_state()

    def plant_step(self, x, u, dist, dt):
        self.mod.plant_step(x, u, dist, dt)

    def measure(self, x):
        return self.mod.measure(x)


# ---------------------------------------------------------------- 参考 PID（镜像 pid.hpp：测量微分+低通、条件积分抗饱和、限幅）
class RefPID:
    def __init__(self, kp, ki, kd, umax, dtau, aw, dt):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.umax, self.dtau, self.aw, self.dt = umax, dtau, aw, dt
        self.reset()

    def reset(self):
        self.integral = self.derivative = self.prev = 0.0
        self.first = True

    def update(self, target, y):
        dt = self.dt
        e = target - y
        raw = 0.0 if self.first else -(y - self.prev) / dt
        self.derivative += dt / (self.dtau + dt) * (raw - self.derivative)
        self.prev = y
        self.first = False
        cand = self.integral + e * dt
        trial = self.kp * e + self.ki * cand + self.kd * self.derivative
        pushes = (trial > self.umax and e > 0) or (trial < -self.umax and e < 0)
        if self.ki != 0 and (not self.aw or not pushes):
            self.integral = cand
        out = self.kp * e + self.ki * self.integral + self.kd * self.derivative
        return max(-self.umax, min(self.umax, out))


def make_pid_factory(mod, cfg, kp, ki, kd):
    if mod is not None and hasattr(mod, "make_pid"):
        return lambda: mod.make_pid(kp, ki, kd, cfg["umax"], cfg["dtau"], bool(cfg["aw"]), cfg["dt"])
    return lambda: RefPID(kp, ki, kd, cfg["umax"], cfg["dtau"], cfg["aw"], cfg["dt"])


# ---------------------------------------------------------------- 第 1 步：开环阶跃辨识
def step_identify(plant, cfg, u0, T):
    dt = cfg["dt"]
    st = plant.make_state()
    y0 = plant.measure(st)
    n = int(T / dt)
    ys = []
    for k in range(n):
        ys.append(plant.measure(st) - y0)
        plant.plant_step(st, u0, 0.0, dt)
    y_end = ys[-1]
    tail = ys[int(0.75 * n):]
    span = max(tail) - min(tail)
    if abs(y_end) > 1e-12 and span < 0.01 * abs(y_end):
        # 自平衡对象 → FOPDT 两点拟合
        K = y_end / u0
        t1 = t2 = None
        for k, yv in enumerate(ys):
            if t1 is None and yv >= 0.283 * y_end:
                t1 = k * dt
            if yv >= 0.632 * y_end:
                t2 = k * dt
                break
        if t1 is None or t2 is None or t2 <= t1:
            return {"class": "stable-unfittable"}
        Tc = 1.5 * (t2 - t1)
        L = t2 - Tc
        return {"class": "FOPDT", "K": K, "L": max(L, 0.0), "T": Tc}
    # 不收敛 → 积分类：稳态斜率 + 等效一阶滞后
    i0 = int(0.75 * n)
    xs = [k * dt for k in range(i0, n)]
    ysl = ys[i0:]
    mx = sum(xs) / len(xs)
    my = sum(ysl) / len(ysl)
    denom = sum((x - mx) ** 2 for x in xs)
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ysl)) / denom if denom else 0.0
    if abs(slope) < 1e-12:
        return {"class": "unknown"}
    b_est = u0 / slope
    tau = (slope * T - ys[-1]) * b_est / u0
    return {"class": "integrating", "K_i": 1.0 / b_est, "tau": tau}


# ---------------------------------------------------------------- 第 2 步：继电反馈极限环
def relay_test(plant, cfg, h, eps, T):
    dt = cfg["dt"]
    sign = -1.0 if cfg["invert"] else 1.0
    st = plant.make_state()
    state, u = 1, h
    switch_t, half_peaks, cur = [], [], 0.0
    for k in range(int(T / dt)):
        t = k * dt
        e = -plant.measure(st)  # 设定值 0 的调节工况
        ns = state
        if e > eps:
            ns = 1
        elif e < -eps:
            ns = -1
        if ns != state:  # 关键：只在状态翻转事件记录，不能按电平逐拍记录
            state = ns
            u = h * state
            switch_t.append(t)
            half_peaks.append(cur)
            cur = 0.0
        cur = max(cur, abs(plant.measure(st)))
        plant.plant_step(st, sign * u, 0.0, dt)
    warm = 0.4 * T
    sw = [t for t in switch_t if t > warm]
    pk = [p for p, t in zip(half_peaks, switch_t) if t > warm]
    if len(sw) < 3:
        return None
    Tus = [sw[i + 2] - sw[i] for i in range(len(sw) - 2)]
    A = sum(pk) / len(pk)
    if A <= 0:
        return None
    Tu = sum(Tus) / len(Tus)
    return {"A": A, "Tu": Tu, "Ku": 4.0 * h / (math.pi * A),
            "Tu_min": min(Tus), "Tu_max": max(Tus), "cycles": len(Tus)}


# ---------------------------------------------------------------- 第 3 步：Z-N 家族表
def zn_candidates(Ku, Tu, fopdt=None):
    out = [
        ("ZN-classic",   0.60 * Ku, 1.2 * Ku / Tu, 0.075 * Ku * Tu),
        ("Pessen",       0.70 * Ku, 1.0 * Ku / Tu, 0.0875 * Ku * Tu),
        ("ZN-some-ov",   0.33 * Ku, 0.66 * Ku / Tu, 0.110 * Ku * Tu),
        ("ZN-no-ov",     0.20 * Ku, 0.40 * Ku / Tu, 0.0667 * Ku * Tu),
        ("Tyreus-Luyben", 0.45 * Ku, 0.45 * Ku / (2.2 * Tu), 0.45 * Ku * Tu / 6.3),
    ]
    if fopdt and fopdt.get("class") == "FOPDT":
        K, L, T = fopdt["K"], fopdt["L"], fopdt["T"]
        if L > 1e-9 and abs(K) > 1e-12:
            kp = 1.2 * T / (K * L)
            out.append(("ZN-openloop", kp, kp / (2.0 * L), kp * 0.5 * L))
    return out


# ---------------------------------------------------------------- 第 4 步：闭环验证（复现用户工况）
def verify(plant, cfg, pid_factory, target, load, step_t, load_t, T):
    dt = cfg["dt"]
    sign = -1.0 if cfg["invert"] else 1.0
    pid = pid_factory()
    st = plant.make_state()
    n = int(T / dt)
    band = 0.02 * abs(target)
    peak, settle_t, dip, recover_t, iae, umax = 0.0, None, 0.0, None, 0.0, 0.0
    for k in range(n):
        t = k * dt
        tgt = target if t >= step_t else 0.0
        ext = load if t >= load_t else 0.0
        y = plant.measure(st)
        u = pid.update(tgt, y)
        umax = max(umax, abs(u))
        if step_t <= t < load_t:
            peak = max(peak, y)
            if abs(tgt - y) > band:
                settle_t = t
        if t >= step_t:
            iae += abs(tgt - y) * dt  # 全程 IAE（含负载段），方案排序统一用这个
        if t >= load_t:
            dip = max(dip, abs(tgt - y))
            if abs(tgt - y) > band:
                recover_t = t
        if abs(y) > 1e3 * (abs(target) + 1.0) or not math.isfinite(y):
            return None  # 发散
        plant.plant_step(st, sign * u, ext, dt)
    overshoot = (peak - target) / target * 100.0 if target > 0 else float("nan")
    return {
        "overshoot": overshoot,
        "settle": (settle_t - step_t) if settle_t is not None else 0.0,
        "dip": dip,
        "recover": (recover_t - load_t) if recover_t is not None else 0.0,
        "iae": iae,
        "umax": umax,
    }


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser(description="PID 自动整定：阶跃辨识 + 继电自整定 + Z-N + 闭环验证")
    ap.add_argument("--num", help="传递函数分子系数（降幂，逗号分隔）")
    ap.add_argument("--den", help="传递函数分母系数（降幂，逗号分隔）")
    ap.add_argument("--plant-script", help="自定义对象模块路径（契约见文件头）")
    ap.add_argument("--mode", default="all", choices=["step", "relay", "tables", "all"])
    ap.add_argument("--dt", type=float)
    ap.add_argument("--umax", type=float)
    ap.add_argument("--dtau", type=float)
    ap.add_argument("--aw", type=int, choices=[0, 1])
    ap.add_argument("--h", type=float)
    ap.add_argument("--eps", type=float)
    ap.add_argument("--relay-t", type=float, default=8.0)
    ap.add_argument("--target", type=float)
    ap.add_argument("--load", type=float)
    ap.add_argument("--load-t", type=float)
    ap.add_argument("--step-t", type=float)
    ap.add_argument("--T", type=float)
    ap.add_argument("--open-t", type=float, default=10.0)
    ap.add_argument("--u0", type=float)
    ap.add_argument("--current", help="用户现有参数 kp,ki,kd（加入对照）")
    ap.add_argument("--ku", type=float, help="硬件实测临界增益（跳过继电仿真）")
    ap.add_argument("--tu", type=float, help="硬件实测振荡周期")
    ap.add_argument("--invert", action="store_true")
    args = ap.parse_args()

    mod = None
    if args.plant_script:
        import importlib.util
        spec = importlib.util.spec_from_file_location("user_plant", args.plant_script)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)

    def pick(cli_val, mod_attr, default):
        if cli_val is not None:
            return cli_val
        if mod is not None and hasattr(mod, mod_attr):
            return getattr(mod, mod_attr)
        return default

    cfg = {
        "dt": pick(args.dt, "DT", 0.002),
        "umax": pick(args.umax, "UMAX", 0.65),
        "dtau": pick(args.dtau, "DTAU", 0.025),
        "aw": pick(args.aw, "AW", 1),
        "invert": args.invert,
    }
    target = pick(args.target, "TARGET", 1.0)
    load = pick(args.load, "LOAD", 0.0)
    load_t = pick(args.load_t, "LOAD_T", 6.0)
    step_t = pick(args.step_t, "STEP_T", 1.0)
    T = pick(args.T, "T_END", 15.0)
    h0 = pick(args.h, "H", 0.4 * cfg["umax"])
    eps = pick(args.eps, "EPS", 0.005)
    u0 = pick(args.u0, "U0", 0.15 * cfg["umax"])

    if args.plant_script:
        plant = ScriptPlant(mod)
    elif args.num and args.den:
        plant = TFPlant([float(v) for v in args.num.split(",")],
                        [float(v) for v in args.den.split(",")])
    else:
        ap.error("需要 --num/--den 或 --plant-script")

    fopdt = None
    if args.mode in ("step", "all"):
        print("=== 开环阶跃辨识 (u0=%g, T=%gs) ===" % (u0, args.open_t))
        fopdt = step_identify(plant, cfg, u0, args.open_t)
        if fopdt["class"] == "FOPDT":
            print("  类别: 自平衡 FOPDT   K=%g  L=%g s  T=%g s  L/T=%.2f" %
                  (fopdt["K"], fopdt["L"], fopdt["T"], fopdt["L"] / max(fopdt["T"], 1e-12)))
            print("  → L/T 明显(>0.3)可用开环 Z-N；否则仍建议继电法")
        elif fopdt["class"] == "integrating":
            print("  类别: 积分类（不收敛）   积分增益 K_i=%g  等效滞后 tau=%g s" %
                  (fopdt.get("K_i", float("nan")), fopdt.get("tau", float("nan"))))
            print("  → 开环 Z-N 退化（L≈0），必须用继电法；纯 P 永不会临界振荡")
        else:
            print("  类别: %s（无法拟合，检查对象/单位）" % fopdt["class"])

    ku = tu = None
    if args.ku and args.tu:
        ku, tu = args.ku, args.tu
        print("\n=== 使用硬件实测 Ku=%g, Tu=%g s ===" % (ku, tu))
    elif args.mode in ("relay", "all"):
        print("\n=== 继电反馈极限环 (eps=%g) ===" % eps)
        results = []
        for tag, hv in (("0.6h", 0.6 * h0), ("h", h0), ("1.2h", 1.2 * h0)):
            r = relay_test(plant, cfg, hv, eps, args.relay_t)
            if r is None:
                print("  %s=%g: 未起振（检查 h 是否大于扰动/初始偏差、对象符号、eps 是否过大）" % (tag, hv))
                continue
            results.append((hv, r))
            warn = ""
            if r["Tu"] < 12 * cfg["dt"]:
                warn = "  ⚠ Tu 逼近采样周期：对象无明显死区，Ku 不可靠，改用开环 Z-N 或增大 eps"
            print("  h=%g: A=%g  Tu=%.4f s (min %.4f / max %.4f, %d 周期)  Ku=4h/(pi*A)=%.3f%s" %
                  (hv, r["A"], r["Tu"], r["Tu_min"], r["Tu_max"], r["cycles"], r["Ku"], warn))
        if results:
            mid = results[len(results) // 2] if len(results) >= 3 else results[0]
            hv, r = mid
            ku, tu = r["Ku"], r["Tu"]
            if len(results) >= 2:
                kus = [x[1]["Ku"] for x in results]
                print("  → Ku 离散度 %.1f%%（非正弦波形的正常描述函数偏差，取中值 h=%g 的结果）" %
                      (100.0 * (max(kus) - min(kus)) / max(abs(kus[0]), 1e-12), hv))

    if args.mode in ("tables", "all"):
        if ku is None or tu is None:
            print("\n缺少 Ku/Tu（继电未起振或未提供 --ku/--tu），跳过查表")
            return
        print("\n=== Z-N 表候选 → 闭环验证 (target=%g, load=%g@%gs, band=2%%) ===" %
              (target, load, load_t))
        print("  %-14s %8s %9s %8s | %7s %8s %9s %7s %8s %8s" %
              ("方案", "Kp", "Ki", "Kd", "超调%", "调节s", "负载跌落", "恢复s", "IAE", "峰值|u|"))
        rows = zn_candidates(ku, tu, fopdt)
        extras = []
        if args.current:
            c = [float(v) for v in args.current.split(",")]
            extras.append(("用户当前", c[0], c[1], c[2]))
        all_rows = rows + extras
        best, best_iae = None, float("inf")
        for name, kp, ki, kd in all_rows:
            m = verify(plant, cfg, make_pid_factory(mod, cfg, kp, ki, kd),
                       target, load, step_t, load_t, T)
            if m is None:
                print("  %-14s %8.3f %9.3f %8.3f | %s" % (name, kp, ki, kd, "发散，淘汰"))
                continue
            stable = m["iae"] < 1e3
            print("  %-14s %8.3f %9.3f %8.3f | %7.1f %8.2f %9.3f %7.2f %8.3f %8.3f" %
                  (name, kp, ki, kd, m["overshoot"], m["settle"], m["dip"],
                   m["recover"], m["iae"], m["umax"]))
            if stable and m["iae"] < best_iae:
                best, best_iae = name, m["iae"]
        if best:
            print("  <<< IAE 最小: %s（最终选择还需结合超调容差与峰值|u|判断，见 SKILL.md 第 5 步）" % best)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001
        print("错误: %s" % exc, file=sys.stderr)
        sys.exit(1)
