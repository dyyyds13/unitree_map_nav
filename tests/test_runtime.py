import json
import time

import numpy as np
import pytest

from go2_rescue.cli import main
from go2_rescue.config import Config
from go2_rescue.controller import Autonomy
from go2_rescue.motion import UnitreeMotion
from go2_rescue.recording import Recorder, decode_packet, replay
from go2_rescue.synthetic import SensorWorld
from go2_rescue.types import Frame, Velocity


class FakeClient:
    def __init__(self):
        self.moves = []
        self.stops = 0

    def Move(self, *args):
        self.moves.append(args)
        return 0

    def StopMove(self):
        self.stops += 1
        return 0


def test_real_motion_refuses_unknown_hardware_even_with_execute():
    with pytest.raises(ValueError, match="verified"):
        UnitreeMotion(Config(), execute=True, client=FakeClient())
    with pytest.raises(ValueError, match="DryRun"):
        UnitreeMotion(Config(), client=FakeClient())


def test_motion_limits_watchdog_and_exception_shutdown():
    now = [0.]
    client = FakeClient()
    config = Config(calibrated=True, hardware_model="test-fixture", network_interface="test0")
    motion = UnitreeMotion(config, execute=True, client=client, clock=lambda: now[0])
    try:
        now[0] = .1
        motion.send(Velocity(100, 100, 100))
        assert len(client.moves) == 1
        assert abs(client.moves[-1][0]) <= config.acceleration * .1 + 1e-9
        now[0] = .5
        motion.check_timeout()
        assert client.stops == 1 and motion.last_sent == Velocity()
        with pytest.raises(ValueError, match="Non-finite"):
            motion.send(Velocity(float("nan")))
        assert client.stops == 2
    finally:
        motion.close()
    assert client.stops == 3 and not motion.thread.is_alive()


def test_transport_failure_latches_fault_and_stops():
    client = FakeClient()
    client.Move = lambda *_: 3
    motion = UnitreeMotion(Config(calibrated=True, hardware_model="fixture", network_interface="test0"),
                           execute=True, client=client)
    try:
        with pytest.raises(RuntimeError, match="failed"):
            motion.send(Velocity(.1))
        assert client.stops == 1
        with pytest.raises(RuntimeError):
            motion.send(Velocity(.1))
    finally:
        motion.close()


def test_recording_roundtrip_and_no_overwrite(tmp_path):
    c = Config()
    world = SensorWorld(c)
    recorder = Recorder(tmp_path / "recording")
    original = []
    for _ in range(3):
        frame = world.frame()
        original.append(frame)
        recorder.write(frame)
        world.advance(Velocity(.1))
    recorder.close()
    restored = list(replay(tmp_path / "recording"))
    assert len(restored) == 3
    for a, b in zip(original, restored):
        assert a.timestamp == b.timestamp and np.array_equal(a.points, b.points)
        assert np.array_equal(a.image, b.image)
    with pytest.raises(ValueError, match="empty"):
        Recorder(tmp_path / "recording")


def test_live_packet_contract_rejects_unknown_frames_and_clocks():
    message = {"schema": "go2-rescue-frame/1", "point_frame": "body_flu", "clock": "host_monotonic",
               "timestamp": time.monotonic(), "points": [[1, 0, 0]], "quaternion_wxyz": [1, 0, 0, 0],
               "velocity": {"vx": 0., "vy": 0., "wz": 0.}, "state_age": 0.}
    frame = decode_packet(json.dumps(message).encode())
    assert frame.points.shape == (1, 3)
    message["point_frame"] = "map_truth"
    with pytest.raises(ValueError, match="body_flu"):
        decode_packet(json.dumps(message).encode())
    message["point_frame"], message["clock"] = "body_flu", "remote_wall_time"
    with pytest.raises(ValueError, match="monotonic"):
        decode_packet(json.dumps(message).encode())


def test_controller_stale_state_empty_scan_and_start_debounce():
    c, world = Config(map_size=10.), SensorWorld(Config())
    autonomy = Autonomy(c)
    frame = world.frame()
    frame.state_age = 1.
    result = autonomy.step(frame)
    assert result.velocity == Velocity() and result.reason == "stale_state"
    frame.state_age = 0.
    result = autonomy.step(frame)
    assert [e.kind for e in result.events] == ["start"]
    world.advance(Velocity())
    assert not any(e.kind == "start" for e in autonomy.step(world.frame()).events)
    result = autonomy.step(Frame(.2, np.empty((0, 3))))
    assert result.velocity == Velocity() and result.reason == "localization_unreliable"


def test_missing_finish_does_not_emit_false_success():
    c = Config(map_size=10., mission_seconds=1., return_reserve=.4)
    world, autonomy = SensorWorld(c), Autonomy(c)
    events = []
    result = None
    for _ in range(14):
        result = autonomy.step(world.frame())
        events.extend(result.events)
        world.advance(Velocity())
    assert result.state == "timed_out"
    assert not any(e.kind == "finish" for e in events)
    assert sum(e.kind == "timeout" for e in events) == 1


def test_cli_demo_replay_and_live_execution_guard(tmp_path, monkeypatch):
    out = tmp_path / "run"
    monkeypatch.setattr("sys.argv", ["go2-rescue", "demo", "--output", str(out), "--steps", "3", "--record"])
    main()
    summary = json.loads((out / "summary.json").read_text())
    assert summary["frames"] == 3 and not summary["robot_commands_enabled"]
    replay_out = tmp_path / "replayed"
    monkeypatch.setattr("sys.argv", ["go2-rescue", "replay", "--input", str(out / "recording"),
                                     "--output", str(replay_out), "--steps", "3"])
    main()
    original = [json.loads(x) for x in (out / "trajectory.jsonl").read_text().splitlines()]
    replayed = [json.loads(x) for x in (replay_out / "trajectory.jsonl").read_text().splitlines()]
    assert [x["velocity"] for x in original] == [x["velocity"] for x in replayed]
    monkeypatch.setattr("sys.argv", ["go2-rescue", "live", "--output", str(tmp_path / "robot"), "--execute"])
    with pytest.raises(SystemExit) as exc:
        main()
    assert exc.value.code == 1
    assert not (tmp_path / "robot").exists()


@pytest.mark.parametrize("seed", [0, 1])
def test_sensor_only_mission_detects_reports_returns_and_finishes(seed):
    c = Config(map_size=10., mission_seconds=8., return_reserve=6.)
    world = SensorWorld(c, seed=seed, mission_fixture=True)
    autonomy = Autonomy(c)
    events = []
    for _ in range(100):
        result = autonomy.step(world.frame())
        events.extend(result.events)
        world.advance(result.velocity)
        if result.state in {"finished", "timed_out"}:
            break
    kinds = [e.kind for e in events]
    assert kinds == ["start", "target", "finish"]
    assert result.velocity == Velocity()
    assert events[-1].timestamp > events[1].timestamp
