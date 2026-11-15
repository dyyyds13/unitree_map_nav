"""Native Unitree velocity transport with a command freshness watchdog.

Importing this module cannot connect to or move a robot. Only constructing an
executing UnitreeMotion opens SDK transport. No stand-up/mode-switch calls occur.
"""
from __future__ import annotations

import threading
import time
import numpy as np

from .config import Config
from .types import Velocity


class DryRunMotion:
    def __init__(self):
        self.commands: list[Velocity] = []

    def send(self, command: Velocity):
        self.commands.append(command)

    def close(self):
        self.send(Velocity())


class UnitreeMotion:
    def __init__(self, config: Config, *, execute: bool = False, client=None, clock=time.monotonic):
        if not execute:
            raise ValueError("Use DryRunMotion unless execution is explicitly enabled")
        if not config.calibrated or not config.network_interface or config.hardware_model == "unknown":
            raise ValueError("Real motion requires verified model, network interface and calibrated configuration")
        config.validate()
        self.c, self.clock = config, clock
        if client is None:
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            from .sdk import initialize
            initialize(config.network_interface)
            client = SportClient()
            client.SetTimeout(.2)
            client.Init()
        self.client = client
        self.lock = threading.Lock()
        self.closed = threading.Event()
        self.last_time, self.last_sent = clock(), Velocity()
        self.fault: str | None = None
        self.stopped = True
        self.thread = threading.Thread(target=self._watchdog, daemon=True, name="go2-command-watchdog")
        self.thread.start()

    def send(self, command: Velocity):
        if not np.isfinite(command.array()).all():
            self.stop()
            raise ValueError("Non-finite robot command")
        with self.lock:
            if self.closed.is_set() or self.fault:
                raise RuntimeError(self.fault or "Motion transport closed")
            now = self.clock()
            dt = min(.2, max(.02, now - self.last_time))
            limits = np.array([self.c.max_vx, self.c.max_vy, self.c.max_wz])
            requested = np.clip(command.array(), -limits, limits)
            if np.linalg.norm(requested) < 1e-6:
                code = self.client.StopMove()
                sent = np.zeros(3)
                self.stopped = True
            else:
                increments = np.array([self.c.acceleration, self.c.acceleration, self.c.angular_acceleration]) * dt
                sent = self.last_sent.array() + np.clip(requested - self.last_sent.array(), -increments, increments)
                code = self.client.Move(*map(float, sent))
                self.stopped = False
            if code not in (None, 0):
                self.fault = f"Unitree command failed: {code}"
                self.client.StopMove()
                raise RuntimeError(self.fault)
            self.last_sent, self.last_time = Velocity(*sent), now

    def stop(self):
        with self.lock:
            self.client.StopMove()
            self.last_sent, self.stopped = Velocity(), True

    def check_timeout(self):
        with self.lock:
            if not self.stopped and self.clock() - self.last_time > self.c.command_timeout:
                try:
                    code = self.client.StopMove()
                    if code not in (None, 0):
                        self.fault = f"Watchdog StopMove failed: {code}"
                except Exception as error:
                    self.fault = f"Watchdog transport error: {error}"
                self.stopped, self.last_sent = True, Velocity()

    def _watchdog(self):
        while not self.closed.wait(.025):
            self.check_timeout()

    def close(self):
        self.closed.set()
        self.thread.join(timeout=.5)
        self.stop()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
