"""
Quadrature encoder reader (CircuitPython/Blinka only) — minimal and private use.
Small API: get_count(), read_pins(), stop().
"""

import threading
import time
import board  # type: ignore
import digitalio  # type: ignore


class QuadratureEncoder:
    def __init__(self, channel_a, channel_b, cpr, invert=False):
        # Public config
        self.channel_a = int(channel_a)
        self.channel_b = int(channel_b)
        self.cpr = max(1, int(cpr))
        self.invert = bool(invert)

        # Internal state
        self._count = 0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None

        # Setup Blinka pins directly in __init__ (minimal methods)
        pin_a = getattr(board, f'D{self.channel_a}')
        pin_b = getattr(board, f'D{self.channel_b}')
        self._pin_a = digitalio.DigitalInOut(pin_a)
        self._pin_b = digitalio.DigitalInOut(pin_b)
        self._pin_a.switch_to_input(pull=digitalio.Pull.UP)
        self._pin_b.switch_to_input(pull=digitalio.Pull.UP)
        self._prev_a = int(self._pin_a.value)
        self._prev_b = int(self._pin_b.value)

        # Start polling thread immediately
        def _worker():
            while not self._stop_event.is_set():
                a = int(self._pin_a.value)
                b = int(self._pin_b.value)
                if a != self._prev_a:
                    # A-edge: +1 when A != B, -1 when A == B
                    direction = 1 if (a ^ b) == 1 else -1
                    if self.invert:
                        direction = -direction
                    with self._lock:
                        self._count += direction
                    self._prev_a = a
                if b != self._prev_b:
                    # B-edge: +1 when A == B, -1 when A != B
                    direction = 1 if (a ^ b) == 0 else -1
                    if self.invert:
                        direction = -direction
                    with self._lock:
                        self._count += direction
                    self._prev_b = b
                time.sleep(0.0005)  # ~2 kHz loop

        self._thread = threading.Thread(target=_worker, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if getattr(self, '_pin_a', None) is not None:
            self._pin_a.deinit()
        if getattr(self, '_pin_b', None) is not None:
            self._pin_b.deinit()

    def get_count(self):
        with self._lock:
            return self._count

    def read_pins(self):
        return int(self._pin_a.value), int(self._pin_b.value)
