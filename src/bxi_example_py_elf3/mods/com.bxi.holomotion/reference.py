"""Ordered HoloMotion reference_qpos receiver.

The ZMQ thread only validates and buffers packets. The BXI control thread owns
the playout cursor and advances it exactly once after a successful policy step.
"""

from __future__ import annotations

import json
import math
import time
from collections import OrderedDict
from dataclasses import dataclass
from threading import Event, Lock, Thread

import numpy as np
import zmq

HEADER_SIZE = 1280
TOPIC = b"reference_qpos"
QPOS_DIM = 36
FUTURE_FRAME_COUNT = 10
OBSERVATION_FRAME_COUNT = FUTURE_FRAME_COUNT + 1
CURRENT_FRAME_OFFSET = 0
# Legacy online contract: current + 10 policy future frames.
WINDOW_SIZE = OBSERVATION_FRAME_COUNT
_DTYPES = {
    "f32": np.dtype("<f4"),
    "f64": np.dtype("<f8"),
    "i32": np.dtype("<i4"),
    "i64": np.dtype("<i8"),
    "u8": np.dtype("u1"),
    "bool": np.dtype("?"),
}


def _scalar(payload: dict[str, np.ndarray], name: str, default=None):
    if name not in payload:
        return default
    values = np.asarray(payload[name]).reshape(-1)
    if values.size != 1:
        raise ValueError(f"{name} must contain exactly one value")
    return values[0].item()


def unpack_reference_packet(packet: bytes) -> dict[str, np.ndarray]:
    if not packet.startswith(TOPIC):
        raise ValueError("reference packet topic mismatch")
    payload = memoryview(packet)[len(TOPIC) :]
    if len(payload) < HEADER_SIZE:
        raise ValueError("reference packet is shorter than its header")
    raw_header = bytes(payload[:HEADER_SIZE]).rstrip(b"\x00")
    if not raw_header:
        raise ValueError("reference packet has an empty header")
    header = json.loads(raw_header.decode("utf-8"))
    data = payload[HEADER_SIZE:]
    result: dict[str, np.ndarray] = {}
    offset = 0
    for field in header.get("fields", []):
        name = str(field["name"])
        dtype_name = str(field["dtype"])
        if dtype_name not in _DTYPES:
            raise ValueError(f"unsupported dtype for {name}: {dtype_name}")
        dtype = _DTYPES[dtype_name]
        shape = tuple(int(value) for value in field.get("shape", []))
        count = int(np.prod(shape, dtype=np.int64)) if shape else 1
        end = offset + count * dtype.itemsize
        if end > len(data):
            raise ValueError(f"reference field {name} exceeds packet bounds")
        value = np.frombuffer(data[offset:end], dtype=dtype, count=count)
        result[name] = np.array(
            value.reshape(shape if shape else ()), copy=True
        )
        offset = end
    return result


@dataclass(frozen=True, slots=True)
class ReferenceWindow:
    qpos: np.ndarray
    frame_index: int
    latest_frame_index: int
    source_latest_frame_index: int
    receive_age_s: float
    source_latest_receive_age_s: float
    stream_epoch: int


class ReferenceReceiver:
    """One producer thread plus one ordered BXI-cycle consumer."""

    def __init__(
        self,
        *,
        capacity: int = 256,
    ) -> None:
        if capacity < WINDOW_SIZE:
            raise ValueError(
                "reference capacity must fit one kinematics window"
            )
        self.capacity = int(capacity)
        self.uri = "tcp://127.0.0.1:6001"
        self.max_source_age_s = 0.6
        self.max_inter_frame_gap_s = 0.06
        self._lock = Lock()
        self._frames: OrderedDict[int, np.ndarray] = OrderedDict()
        self._receive_times: OrderedDict[int, float] = OrderedDict()
        self._last_frame_index: int | None = None
        self._last_sample_time: float | None = None
        self._playout_frame: int | None = None
        self._stream_epoch = 0
        self._reset_count = 0
        self._last_error: BaseException | None = None
        self._stop = Event()
        self._ready = Event()
        self._thread: Thread | None = None

    def configure(
        self,
        *,
        uri: str,
        max_source_age_s: float,
        max_inter_frame_gap_s: float,
    ) -> None:
        if self._thread is not None:
            raise RuntimeError("cannot reconfigure a running receiver")
        if not uri:
            raise ValueError("reference URI must not be empty")
        for name, value in (
            ("max_source_age_s", max_source_age_s),
            ("max_inter_frame_gap_s", max_inter_frame_gap_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        self.uri = str(uri)
        self.max_source_age_s = float(max_source_age_s)
        self.max_inter_frame_gap_s = float(max_inter_frame_gap_s)

    @property
    def last_error(self) -> BaseException | None:
        return self._last_error

    @property
    def reset_count(self) -> int:
        with self._lock:
            return self._reset_count

    def start(self) -> None:
        thread = self._thread
        if thread is not None and thread.is_alive():
            return
        self._stop.clear()
        self._ready.clear()
        self._last_error = None
        thread = Thread(
            target=self._run,
            name="holomotion-reference",
            daemon=False,
        )
        self._thread = thread
        thread.start()
        self._ready.wait(timeout=2.0)
        if not self._ready.is_set():
            raise RuntimeError("reference receiver startup timed out")
        if self._last_error is not None:
            raise RuntimeError(
                f"reference receiver failed to start: {self._last_error}"
            ) from self._last_error

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)
            if thread.is_alive():
                raise RuntimeError("reference receiver did not stop")
        self._thread = None

    def _run(self) -> None:
        context = None
        socket = None
        try:
            context = zmq.Context()
            socket = context.socket(zmq.SUB)
            socket.setsockopt(zmq.SUBSCRIBE, TOPIC)
            socket.setsockopt(zmq.RCVHWM, self.capacity)
            socket.setsockopt(zmq.LINGER, 0)
            socket.connect(self.uri)
            poller = zmq.Poller()
            poller.register(socket, zmq.POLLIN)
            self._ready.set()
            while not self._stop.is_set():
                if socket not in dict(poller.poll(timeout=50)):
                    continue
                while True:
                    try:
                        packet = socket.recv(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    try:
                        self.push_payload(
                            unpack_reference_packet(packet),
                            receive_monotonic=time.monotonic(),
                        )
                    except Exception as exc:  # noqa: BLE001 - reject a bad packet
                        self._last_error = exc
                        self.clear(discontinuity=True)
        except BaseException as exc:  # noqa: BLE001 - publish thread startup failure
            self._last_error = exc
            self._ready.set()
        finally:
            if socket is not None:
                socket.close(linger=0)
            if context is not None:
                context.term()

    def clear(self, *, discontinuity: bool = False) -> None:
        with self._lock:
            self._frames.clear()
            self._receive_times.clear()
            self._last_frame_index = None
            self._last_sample_time = None
            self._playout_frame = None
            if discontinuity:
                self._stream_epoch += 1
                self._reset_count += 1

    def push_payload(
        self,
        payload: dict[str, np.ndarray],
        *,
        receive_monotonic: float,
    ) -> bool:
        qpos = np.asarray(payload.get("reference_qpos"), dtype=np.float32)
        if qpos.shape != (QPOS_DIM,):
            raise ValueError(
                f"reference_qpos must have shape ({QPOS_DIM},), got {qpos.shape}"
            )
        if not np.all(np.isfinite(qpos)):
            raise ValueError("reference_qpos contains NaN or Inf")
        frame_index = int(_scalar(payload, "frame_index"))
        sample_time = float(
            _scalar(payload, "timestamp_monotonic", receive_monotonic)
        )
        if not math.isfinite(sample_time):
            raise ValueError("timestamp_monotonic must be finite")
        with self._lock:
            last_frame = self._last_frame_index
            last_sample = self._last_sample_time
            if last_frame is not None and frame_index <= last_frame:
                if frame_index != 0 or last_frame == 0:
                    return False
                self._frames.clear()
                self._receive_times.clear()
                self._playout_frame = None
                self._stream_epoch += 1
                self._reset_count += 1
                last_frame = None
                last_sample = None
            discontinuity = False
            if last_frame is not None:
                discontinuity = (
                    frame_index != last_frame + 1
                    or sample_time <= float(last_sample)
                    or sample_time - float(last_sample)
                    > self.max_inter_frame_gap_s
                )
            if discontinuity:
                self._frames.clear()
                self._receive_times.clear()
                self._playout_frame = None
                self._stream_epoch += 1
                self._reset_count += 1
            self._frames[frame_index] = qpos.copy()
            self._receive_times[frame_index] = float(receive_monotonic)
            self._last_frame_index = frame_index
            self._last_sample_time = sample_time
            while len(self._frames) > self.capacity:
                oldest, _ = self._frames.popitem(last=False)
                self._receive_times.pop(oldest, None)
            return True

    def reset_playout_to_latest(self) -> None:
        with self._lock:
            self._playout_frame = None
            self._select_latest_start_locked()

    @property
    def playout_description(self) -> str:
        return "11-frame current + 10 future window (~200 ms)"

    def _raw_window_indices(self, current: int) -> tuple[int, ...]:
        start = current - CURRENT_FRAME_OFFSET
        return tuple(range(start, start + WINDOW_SIZE))

    def _select_latest_start_locked(self) -> bool:
        required = WINDOW_SIZE
        current_lag = WINDOW_SIZE - CURRENT_FRAME_OFFSET - 1
        if len(self._frames) < required:
            return False
        latest = next(reversed(self._frames))
        current = latest - current_lag
        indices = self._raw_window_indices(current)
        if all(index in self._frames for index in indices):
            self._playout_frame = current
            return True
        return False

    def _build_qpos_window(self, indices: tuple[int, ...]) -> np.ndarray:
        return np.stack(
            tuple(self._frames[index] for index in indices), axis=0
        )

    def peek_window(self, now: float | None = None) -> ReferenceWindow | None:
        timestamp = time.monotonic() if now is None else float(now)
        with self._lock:
            if (
                self._playout_frame is None
                and not self._select_latest_start_locked()
            ):
                return None
            assert self._playout_frame is not None
            current = self._playout_frame
            indices = self._raw_window_indices(current)
            if not all(index in self._frames for index in indices):
                return None
            latest_receive = self._receive_times[indices[-1]]
            source_latest = next(reversed(self._frames))
            source_latest_receive = self._receive_times[source_latest]
            return ReferenceWindow(
                qpos=self._build_qpos_window(indices),
                frame_index=current,
                latest_frame_index=indices[-1],
                source_latest_frame_index=source_latest,
                receive_age_s=max(0.0, timestamp - latest_receive),
                source_latest_receive_age_s=max(
                    0.0, timestamp - source_latest_receive
                ),
                stream_epoch=self._stream_epoch,
            )

    def advance_after_successful_step(self, frame_index: int) -> None:
        with self._lock:
            if self._playout_frame != int(frame_index):
                raise RuntimeError(
                    "reference playout cursor changed during inference"
                )
            self._playout_frame += 1
            keep_from = self._playout_frame - CURRENT_FRAME_OFFSET
            while self._frames:
                oldest = next(iter(self._frames))
                if oldest >= keep_from:
                    break
                self._frames.pop(oldest)
                self._receive_times.pop(oldest, None)

    def latest_receive_age(self, now: float | None = None) -> float:
        timestamp = time.monotonic() if now is None else float(now)
        with self._lock:
            if not self._receive_times:
                return float("inf")
            latest_key = next(reversed(self._receive_times))
            latest = self._receive_times[latest_key]
            return max(0.0, timestamp - latest)


__all__ = [
    "CURRENT_FRAME_OFFSET",
    "FUTURE_FRAME_COUNT",
    "OBSERVATION_FRAME_COUNT",
    "QPOS_DIM",
    "WINDOW_SIZE",
    "ReferenceReceiver",
    "ReferenceWindow",
    "unpack_reference_packet",
]
