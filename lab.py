"""Local teaching server. Python 3.9+, C++11 compiler; no pip dependencies."""
import argparse
import csv
import io
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
import queue
from collections import deque
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
import webbrowser

ROOT = Path(__file__).resolve().parent
BUILD = ROOT / "build"
EXE = BUILD / ("gimbal_sim.exe" if os.name == "nt" else "gimbal_sim")
LOCK = threading.Lock()


def command(args):
    result = subprocess.run(args, cwd=ROOT, capture_output=True, text=True,
                            encoding="utf-8", errors="replace", timeout=60)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result.stdout


def compiler():
    configured = os.environ.get("CXX")
    found = shutil.which(configured) if configured else (shutil.which("g++") or shutil.which("clang++"))
    if not found:
        raise RuntimeError("找不到 g++ / clang++。请安装 C++ 编译器并加入 PATH，或将 CXX 设置为编译器路径。参见 README.md。")
    return found


def build(target=EXE):
    BUILD.mkdir(exist_ok=True)
    command([compiler(), "-std=c++11", "-O2", "-Wall", "-Wextra", "-I", str(ROOT / "include"),
             str(ROOT / "src/main.cpp"), str(ROOT / "src/tuning.cpp"), "-o", str(target)])


def simulate(data):
    values = [float(data.get(k, default)) for k, default in
              [("yaw", 45), ("pitch", 20), ("load", 0.08)]]
    if not all(math.isfinite(v) for v in values):
        raise ValueError("参数必须为有限数字")
    if abs(values[0]) > 150 or abs(values[1]) > 60 or abs(values[2]) > 0.3:
        raise ValueError("参数超出范围：Yaw ±150°，Pitch ±60°，外力矩 ±0.3 N·m")
    reverse = data.get("reverse", False)
    if not isinstance(reverse, bool):
        raise ValueError("reverse 必须为布尔值")
    build()
    output = command([str(EXE), *map(str, values), "1" if reverse else "0"])
    rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(io.StringIO(output))]
    if not rows or not all(math.isfinite(v) for row in rows for v in row.values()):
        raise RuntimeError("仿真出现非有限值，请检查 PID 参数和控制器代码。")
    (BUILD / "latest.csv").write_text(output, encoding="utf-8")
    return {"rows": rows}


class LiveEngine:
    """One persistent C++ controller; HTTP threads only change commands/read telemetry."""
    def __init__(self):
        self.lock = threading.Lock()
        self.controls = dict(yaw=0.0, pitch=0.0, load=0.0, paused=False)
        self.rows = deque(maxlen=751)
        self.sequence = 0
        self.generation = 0
        self.session = str(time.time_ns())
        self.reset_pending = True
        self.error = None
        self.process = None
        self.worker = None
        self.stop_event = threading.Event()
        self.slot = 0

    def control(self, data):
        update = {}
        for key, limit in [("yaw", 150), ("pitch", 60), ("load", 0.3)]:
            if key in data:
                value = float(data[key])
                if not math.isfinite(value) or abs(value) > limit:
                    raise ValueError(f"{key} 超出范围 ±{limit}")
                update[key] = value
        if "paused" in data:
            if not isinstance(data["paused"], bool):
                raise ValueError("paused 必须为布尔值")
            update["paused"] = data["paused"]
        with self.lock:
            self.controls.update(update)
            return dict(self.controls)

    def reset(self):
        with self.lock:
            self.reset_pending = True

    def stop(self):
        self.stop_event.set()
        if self.worker:
            self.worker.join(timeout=3)
        if self.process:
            if self.process.poll() is None:
                self.process.terminate()
            self.process.wait(timeout=3)
            self.process.stdin.close()
            self.process.stdout.close()
        self.process = None
        self.worker = None

    def restart(self):
        # Compile an inactive executable first: syntax errors leave the old process running.
        next_slot = 1 - self.slot
        target = BUILD / (f"live_{next_slot}" + (".exe" if os.name == "nt" else ""))
        build(target)
        process = subprocess.Popen([str(target), "--stream"], cwd=ROOT, stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                   text=True, encoding="utf-8", bufsize=1)
        output = queue.Queue()

        def read_output():
            try:
                for line in process.stdout:
                    output.put(line)
            finally:
                output.put(None)

        threading.Thread(target=read_output, daemon=True).start()
        try:
            header = output.get(timeout=2)
            if not header or not header.startswith("time,yaw_target,"):
                raise RuntimeError("C++ 实时协议未就绪")
        except (queue.Empty, RuntimeError) as error:
            process.terminate()
            process.wait(timeout=3)
            process.stdin.close()
            process.stdout.close()
            raise RuntimeError("C++ 实时进程启动失败；旧进程保持运行") from error
        self.stop()
        self.slot = next_slot
        self.process = process
        self.stop_event = threading.Event()
        with self.lock:
            self.reset_pending = True
            self.error = None
        self.worker = threading.Thread(target=self.pump, args=(process, output, header.strip().split(",")), daemon=True)
        self.worker.start()

    def pump(self, process, output, fields):
        deadline = time.monotonic()
        try:
            while not self.stop_event.is_set():
                with self.lock:
                    settings = dict(self.controls)
                    reset = self.reset_pending
                    self.reset_pending = False
                if not settings["paused"] or reset:
                    steps = 0 if reset else 10
                    process.stdin.write(f"{settings['yaw']} {settings['pitch']} {settings['load']} {steps} {int(reset)}\n")
                    process.stdin.flush()
                    line = output.get(timeout=2)
                    if line is None:
                        raise RuntimeError("C++ 进程已退出，请检查代码并重新编译")
                    values = [float(v) for v in line.strip().split(",")]
                    if len(values) != len(fields) or not all(math.isfinite(v) for v in values):
                        raise RuntimeError("C++ 返回了无效数据，请检查 PID 参数")
                    row = dict(zip(fields, values))
                    with self.lock:
                        if reset:
                            self.rows.clear()
                            self.generation += 1
                        self.sequence += 1
                        row["seq"] = self.sequence
                        self.rows.append(row)
                deadline += 0.02
                now = time.monotonic()
                if deadline < now:
                    deadline = now  # Do not replay a backlog after an OS scheduling stall.
                self.stop_event.wait(max(0, deadline - now))
        except (OSError, ValueError, RuntimeError, queue.Empty) as error:
            with self.lock:
                self.error = str(error) or "C++ 响应超时，请重新编译"

    def state(self, after=0):
        with self.lock:
            return dict(rows=[r.copy() for r in self.rows if r["seq"] > after],
                        controls=dict(self.controls), generation=self.generation, session=self.session,
                        error=self.error, running=self.worker is not None and self.worker.is_alive())


ENGINE = LiveEngine()


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT / "web"), **kwargs)

    def log_request(self, code="-", size="-"):
        if self.path.startswith("/api/state") and code == 200:
            return
        super().log_request(code, size)

    def do_POST(self):
        if self.path not in ("/api/run", "/api/control", "/api/reset"):
            self.send_error(404)
            return
        # Only the local teaching UI may request compilation; refuse cross-origin requests.
        expected = "http://" + self.headers.get("Host", "")
        origin = self.headers.get("Origin")
        if origin and origin != expected:
            self.send_error(403)
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 4096:
                raise ValueError("请求长度无效")
            data = json.loads(self.rfile.read(size))
            if not isinstance(data, dict):
                raise ValueError("请求必须为 JSON 对象")
            if self.path == "/api/control":
                response = {"controls": ENGINE.control(data)}
            elif self.path == "/api/reset":
                ENGINE.reset()
                response = {"ok": True}
            else:
                with LOCK:
                    ENGINE.restart()
                response = {"ok": True}
            self.reply(200, response)
        except (ValueError, TypeError, RuntimeError, OSError, subprocess.TimeoutExpired) as error:
            self.reply(400, {"error": str(error)})

    def do_GET(self):
        path = urlsplit(self.path)
        if path.path == "/api/state":
            from urllib.parse import parse_qs
            try:
                after = int(parse_qs(path.query).get("after", ["0"])[0])
                self.reply(200, ENGINE.state(after))
            except ValueError:
                self.reply(400, {"error": "after 必须为整数"})
        elif path.path == "/api/csv":
            rows = ENGINE.state()["rows"]
            stream = io.StringIO(newline="")
            if rows:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            payload = stream.getvalue().encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", 'attachment; filename="pid-response.csv"')
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        else:
            super().do_GET()

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def reply(self, status, data):
        payload = json.dumps(data, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


def test():
    BUILD.mkdir(exist_ok=True)
    executable = BUILD / ("pid_tests.exe" if os.name == "nt" else "pid_tests")
    command([compiler(), "-std=c++11", "-O2", "-Wall", "-Wextra", "-I", str(ROOT / "include"),
             str(ROOT / "tests/pid_tests.cpp"), "-o", str(executable)])
    print(command([str(executable)]).strip())
    rows = simulate({})["rows"]
    assert len(rows) == 751 and rows[-1]["time"] == 15, "sampling interval"
    for axis, target in [("yaw", 45), ("pitch", 20)]:
        tail = [r for r in rows if r["time"] >= 14]
        assert max(abs(r[axis] - target) for r in tail) < 1, "default loaded settling"
        assert max(abs(r[axis + "_u"]) for r in rows) <= 0.650001, "actuator limit"
    print("Simulation tests passed: 751 samples, loaded final error < 1 degree, torque within limits")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--test", action="store_true")
    args = parser.parse_args()
    try:
        if args.test:
            test()
        else:
            ENGINE.restart()
            server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
            url = f"http://127.0.0.1:{args.port}"
            print(f"PID 云台实验室：{url}\n拖动滑块实时控制；修改 src/tuning.cpp 后点击重新编译 C++。Ctrl+C 退出。", flush=True)
            if not args.no_browser:
                webbrowser.open(url)
            try:
                server.serve_forever()
            finally:
                server.server_close()
                ENGINE.stop()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, OSError, subprocess.TimeoutExpired, AssertionError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
