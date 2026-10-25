"""Real persistent C++ process + HTTP contract checks; no browser required."""
import json
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import lab


def main():
    lab.BUILD = lab.BUILD / "test-live"
    lab.BUILD.mkdir(parents=True, exist_ok=True)
    engine = lab.ENGINE
    engine.restart()
    server = lab.ThreadingHTTPServer(("127.0.0.1", 0), lab.Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_port}"

    def request(path, data=None):
        req = urllib.request.Request(base + path, data=None if data is None else json.dumps(data).encode(),
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as response:
            return json.load(response)

    def until(predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            state = request("/api/state")
            if predicate(state):
                return state
            time.sleep(0.02)
        raise AssertionError("Timed out waiting for live state")

    try:
        first = until(lambda s: s["rows"] and s["rows"][-1]["time"] >= 0.06)
        process = engine.process
        request("/api/control", {"yaw": 90, "pitch": -30})
        state = until(lambda s: s["rows"][-1]["yaw"] > 1)
        row = state["rows"][-1]
        assert row["yaw_target"] == 90 and row["pitch_target"] == -30
        assert row["yaw"] < 90 and row["pitch"] < 0, "physical response, not direct pose assignment"
        generation = state["generation"]
        request("/api/control", {"yaw": -60, "load": 0.1})
        changed = until(lambda s: s["rows"][-1]["yaw_target"] == -60)
        assert changed["generation"] == generation and engine.process is process, "target must not restart C++"
        assert changed["rows"][-1]["time"] > row["time"], "simulation time must continue"
        assert changed["rows"][-1]["yaw"] > 0, "inertia must survive command changes"
        assert changed["rows"][-1]["load"] == 0.1
        request("/api/control", {"paused": True})
        time.sleep(0.06)
        paused = request("/api/state")
        time.sleep(0.1)
        assert request("/api/state")["rows"][-1] == paused["rows"][-1], "pause must freeze C++ simulation"
        request("/api/reset", {})
        reset = until(lambda s: s["generation"] > generation)
        assert reset["rows"][-1]["time"] == 0 and reset["rows"][-1]["yaw"] == 0
        assert reset["controls"]["yaw"] == -60, "reset must preserve target"
        request("/api/control", {"paused": False})
        until(lambda s: s["rows"][-1]["yaw"] < 0)
        cursor = request("/api/state")["rows"][-1]["seq"]
        assert all(r["seq"] > cursor for r in request(f"/api/state?after={cursor}")["rows"])
        try:
            request("/api/control", {"yaw": 151})
            raise AssertionError("invalid target accepted")
        except urllib.error.HTTPError as error:
            assert error.code == 400
        # Failure to compile must not discard the working controller.
        original_build = lab.build
        def fail_build(*args):
            raise RuntimeError("intentional test compiler failure")
        lab.build = fail_build
        try:
            try:
                request("/api/run", {})
                raise AssertionError("compile failure accepted")
            except urllib.error.HTTPError as error:
                assert error.code == 400
            assert engine.process is process and process.poll() is None
        finally:
            lab.build = original_build
        previous_generation = engine.generation
        request("/api/run", {})
        until(lambda s: s["generation"] > previous_generation)
        assert process.poll() is not None and engine.process is not process
        with urllib.request.urlopen(base + "/api/csv") as response:
            assert response.read().startswith(b"time,yaw_target")
        with urllib.request.urlopen(base + "/") as response:
            html = response.read().decode()
            assert 'type="range"' in html and 'YOUR NEXT CHANGE' not in html
        print("Live checks passed: target changes, persistent state, pause, reset, validation, rebuild recovery, CSV")
    finally:
        server.shutdown()
        server.server_close()
        engine.stop()


if __name__ == "__main__":
    main()
