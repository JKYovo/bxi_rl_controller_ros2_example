"""On-demand lifecycle for the robot-local HoloRetarget source service."""

from __future__ import annotations

import os
import subprocess
import time


SERVICE_ENV = "HOLOMOTION_RETARGET_SERVICE"
SYSTEMCTL_MODE_ENV = "HOLOMOTION_SYSTEMCTL_MODE"
DEFAULT_SERVICE = "holomotion-retarget.service"
START_TIMEOUT_S = 8.0
STOP_TIMEOUT_S = 12.0
STARTUP_STABILITY_S = 0.5


def _service_name(value: str) -> str:
    name = value.strip()
    if not name or any(
        character
        not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.@-"
        for character in name
    ):
        raise ValueError(
            f"invalid HoloRetarget systemd service name: {value!r}"
        )
    return name


def _run_systemctl(
    *arguments: str, timeout: float
) -> subprocess.CompletedProcess[str]:
    command: tuple[str, ...] = ("systemctl", *arguments)
    if (
        arguments
        and arguments[0] in {"start", "stop"}
        and os.environ.get(SYSTEMCTL_MODE_ENV, "system") == "sudo"
    ):
        # Ubuntu 22.04's LocalAuthority backend cannot authorize one specific
        # systemd unit.  The local simulator therefore uses an exact-command
        # sudoers rule; -n guarantees mode switching can never block on a
        # password prompt.  Robot deployments keep the default system mode.
        command = ("sudo", "-n", "/usr/bin/systemctl", *arguments)
    return subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def _detail(result: subprocess.CompletedProcess[str]) -> str:
    return (
        result.stderr or result.stdout or f"exit status {result.returncode}"
    ).strip()


class HoloRetargetSourceService:
    """Start one dedicated source service and stop only an instance we started."""

    def __init__(self, service_name: str | None = None) -> None:
        self.service_name = _service_name(
            service_name or os.environ.get(SERVICE_ENV, DEFAULT_SERVICE)
        )
        self.started_by_mod = False
        loaded = _run_systemctl(
            "show",
            "--property=LoadState",
            "--value",
            self.service_name,
            timeout=START_TIMEOUT_S,
        )
        if loaded.returncode != 0 or loaded.stdout.strip() != "loaded":
            raise RuntimeError(
                f"HoloRetarget service {self.service_name!r} is not loaded: "
                f"{_detail(loaded)}"
            )
        active = _run_systemctl(
            "is-active",
            "--quiet",
            self.service_name,
            timeout=START_TIMEOUT_S,
        )
        if active.returncode == 0:
            return
        started = _run_systemctl(
            "start",
            self.service_name,
            timeout=START_TIMEOUT_S,
        )
        if started.returncode != 0:
            raise RuntimeError(
                f"cannot start HoloRetarget service {self.service_name!r}: "
                f"{_detail(started)}"
            )
        self.started_by_mod = True
        time.sleep(STARTUP_STABILITY_S)
        stable = _run_systemctl(
            "is-active",
            "--quiet",
            self.service_name,
            timeout=START_TIMEOUT_S,
        )
        if stable.returncode != 0:
            self.started_by_mod = False
            raise RuntimeError(
                f"HoloRetarget service {self.service_name!r} exited during startup"
            )

    def close(self) -> None:
        if not self.started_by_mod:
            return
        self.started_by_mod = False
        stopped = _run_systemctl(
            "stop",
            self.service_name,
            timeout=STOP_TIMEOUT_S,
        )
        if stopped.returncode != 0:
            raise RuntimeError(
                f"cannot stop HoloRetarget service {self.service_name!r}: "
                f"{_detail(stopped)}"
            )


__all__ = ["HoloRetargetSourceService"]
