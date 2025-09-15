import threading
import time
import gpiod

class Encoder:
    """Quadrature encoder usando libgpiod v1.6 API clássica.

    Implementação por polling rápido (0.5ms) lendo linhas; suficiente para encoders de baixa/média velocidade.
    API compatível: read() -> posição acumulada.
    """
    _TRANSITIONS = {
        0b0001: +1, 0b0010: -1, 0b0100: -1, 0b0111: +1,
        0b1000: -1, 0b1011: +1, 0b1101: +1, 0b1110: -1,
    }

    def __init__(self, A: int, B: int, chip_name: str = 'gpiochip0', poll_interval: float = 0.0005):
        self._chip = gpiod.Chip(chip_name)
        self._lineA = self._chip.get_line(A)
        self._lineB = self._chip.get_line(B)
        self._lineA.request(consumer='encA', type=gpiod.LINE_REQ_DIR_IN)
        self._lineB.request(consumer='encB', type=gpiod.LINE_REQ_DIR_IN)
        self._pos = 0
        self._lock = threading.Lock()
        self._last_state = (self._lineA.get_value() << 1) | self._lineB.get_value()
        self._stop = False
        self._poll_interval = poll_interval
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop:
            a = self._lineA.get_value()
            b = self._lineB.get_value()
            state = (a << 1) | b
            if state != self._last_state:
                transition = (self._last_state << 2) | state
                delta = self._TRANSITIONS.get(transition, 0)
                if delta:
                    with self._lock:
                        self._pos += delta
                self._last_state = state
            time.sleep(self._poll_interval)

    def read(self) -> int:
        with self._lock:
            return self._pos

    def close(self):
        self._stop = True
        if self._thread.is_alive():
            self._thread.join(timeout=0.2)
        for line in (self._lineA, self._lineB):
            try:
                line.release()
            except Exception:
                pass
        try:
            self._chip.close()
        except Exception:
            pass
