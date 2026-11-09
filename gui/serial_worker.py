"""Background thread worker for non-blocking serial communication."""

from __future__ import annotations

from PyQt6.QtCore import QThread, pyqtSignal
import serial


class SerialReaderWorker(QThread):
    """Background worker for reading serial telemetry lines."""

    telemetry_received = pyqtSignal(str)
    connection_error = pyqtSignal(str)

    def __init__(self, connection: serial.Serial) -> None:
        super().__init__()
        self.connection = connection
        self._running = True

    def run(self) -> None:
        while self._running:
            try:
                line = self.connection.readline()
                if line:
                    decoded = line.decode("ascii", errors="replace").strip()
                    if decoded:
                        self.telemetry_received.emit(decoded)
            except (serial.SerialException, OSError) as exc:
                if self._running:
                    self.connection_error.emit(str(exc))
                break

    def stop(self) -> None:
        """Stops the reader worker loop."""
        self._running = False
