"""Centered 21-frame reference receiver for live RGMT inference.

The packet format and ordered buffering are shared with HoloMotion. RGMT uses
ten past frames, the current frame, and ten future frames, so only its window
selection differs.
"""

from __future__ import annotations

from .reference import (
    QPOS_DIM,
    ReferenceReceiver as _BaseReferenceReceiver,
    ReferenceWindow,
    unpack_reference_packet,
)

PAST_FRAME_COUNT = 10
FUTURE_FRAME_COUNT = 10
CURRENT_FRAME_OFFSET = PAST_FRAME_COUNT
WINDOW_SIZE = PAST_FRAME_COUNT + 1 + FUTURE_FRAME_COUNT


class ReferenceReceiver(_BaseReferenceReceiver):
    """Use the existing packet/stream checks with an RGMT-centered window."""

    def __init__(self, *, capacity: int = 256) -> None:
        if capacity < WINDOW_SIZE:
            raise ValueError("reference capacity must fit one RGMT window")
        super().__init__(capacity=capacity)

    @property
    def playout_description(self) -> str:
        return "21-frame past 10 + current + future 10 window (~200 ms)"

    def _raw_window_indices(self, current: int) -> tuple[int, ...]:
        start = current - CURRENT_FRAME_OFFSET
        return tuple(range(start, start + WINDOW_SIZE))

    def _select_latest_start_locked(self) -> bool:
        if len(self._frames) < WINDOW_SIZE:
            return False
        latest = next(reversed(self._frames))
        current = latest - FUTURE_FRAME_COUNT
        indices = self._raw_window_indices(current)
        if all(index in self._frames for index in indices):
            self._playout_frame = current
            return True
        return False


__all__ = [
    "CURRENT_FRAME_OFFSET",
    "FUTURE_FRAME_COUNT",
    "PAST_FRAME_COUNT",
    "QPOS_DIM",
    "WINDOW_SIZE",
    "ReferenceReceiver",
    "ReferenceWindow",
    "unpack_reference_packet",
]
