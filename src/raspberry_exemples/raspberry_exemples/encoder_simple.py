"""Encoder quadratura simples usando libgpiod.

Mantém API mínima: get_count() e reset().
Polling rápido (0.5 ms). Ideal para código claro semelhante ao exemplo ESP32.
"""
import threading
import time

try:
    import gpiod
except ImportError as e:
    raise RuntimeError("Instale python3-libgpiod: sudo apt install python3-libgpiod") from e


class SimpleEncoder:
    _TRANS = {
        0b0001: +1, 0b0010: -1, 0b0100: -1, 0b0111: +1,
        0b1000: -1, 0b1011: +1, 0b1101: +1, 0b1110: -1,
    }

    def __init__(self, pin_a: int, pin_b: int, chip: str = 'gpiochip0', poll_s: float = 0.0005):
        self._chip = gpiod.Chip(chip)
        self._a = self._chip.get_line(pin_a)
        self._b = self._chip.get_line(pin_b)
        self._a.request(consumer=f"enc{pin_a}A", type=gpiod.LINE_REQ_DIR_IN)
        self._b.request(consumer=f"enc{pin_b}B", type=gpiod.LINE_REQ_DIR_IN)
        self._count = 0
        self._lock = threading.Lock()
        self._last = (self._a.get_value() << 1) | self._b.get_value()
        self._poll = poll_s
        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        sleep = time.sleep
        while not self._stop:
            a = self._a.get_value()
            b = self._b.get_value()
            cur = (a << 1) | b
            if cur != self._last:
                trans = (self._last << 2) | cur
                delta = self._TRANS.get(trans, 0)
                if delta:
                    with self._lock:
                        self._count += delta
                self._last = cur
            sleep(self._poll)

    def get_count(self) -> int:
        with self._lock:
            return self._count

    def reset(self):
        with self._lock:
            self._count = 0

    def close(self):
        self._stop = True
        if self._thread.is_alive():
            self._thread.join(timeout=0.2)
        for line in (self._a, self._b):
            try:
                line.release()
            except Exception:
                pass
        try:
            self._chip.close()
        except Exception:
            pass
