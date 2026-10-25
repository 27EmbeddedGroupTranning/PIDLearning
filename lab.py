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


def build():
    BUILD.mkdir(exist_ok=True)
    command([compiler(), "-std=c++11", "-O2", "-Wall", "-Wextra", "-I", str(ROOT / "include"),
             str(ROOT / "src/main.cpp"), str(ROOT / "src/tuning.cpp"), "-o", str(EXE)])


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


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT / "web"), **kwargs)

    def do_POST(self):
        if self.path != "/api/run":
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
            with LOCK:
                response = simulate(data)
            self.reply(200, response)
        except (ValueError, TypeError, RuntimeError, OSError, subprocess.TimeoutExpired) as error:
            self.reply(400, {"error": str(error)})

    def do_GET(self):
        if urlsplit(self.path).path == "/api/csv":
            path = BUILD / "latest.csv"
            if not path.exists():
                self.send_error(404, "Run a simulation first")
                return
            payload = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", 'attachment; filename="pid-response.csv"')
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        else:
            super().do_GET()

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
            build()
            server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
            url = f"http://127.0.0.1:{args.port}"
            print(f"PID 云台实验室：{url}\n修改 src/tuning.cpp 后，在页面点击编译并运行。Ctrl+C 退出。", flush=True)
            if not args.no_browser:
                webbrowser.open(url)
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, OSError, subprocess.TimeoutExpired, AssertionError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
