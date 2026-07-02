"""QTcpSocket wrapper for talking to the haptic-device IPC command server.

Protocol (see ipcServer.cpp): newline-delimited plain-text commands, one
newline-terminated response per command, sent strictly in request/response
order over a single connection.
"""

from collections import deque

from PySide6.QtCore import QObject, Signal
from PySide6.QtNetwork import QAbstractSocket, QTcpSocket


def parse_status(line: str) -> dict:
    """Parses 'OK STATUS mode=x freeze=y potential=z atoms=n anchored=m energy=e
    timestep=t render_atoms=b render_forces=b render_bonds=b settling_err=e
    k_return=k k_dampen=k return_delay=t'."""
    fields = {}
    for token in line.split()[2:]:
        if "=" in token:
            key, value = token.split("=", 1)
            fields[key] = value
    return fields


class IpcClient(QObject):
    connected = Signal()
    disconnected = Signal()
    statusReceived = Signal(dict)
    commandFailed = Signal(str, str)  # command, error message

    def __init__(self, parent=None):
        super().__init__(parent)
        self._socket = QTcpSocket(self)
        self._socket.connected.connect(self.connected)
        self._socket.disconnected.connect(self._on_disconnected)
        self._socket.readyRead.connect(self._on_ready_read)
        self._buffer = b""
        self._pending = deque()

    def connect_to(self, host: str, port: int):
        if self._socket.state() != QAbstractSocket.SocketState.UnconnectedState:
            self._socket.abort()
        self._buffer = b""
        self._pending.clear()
        self._socket.connectToHost(host, port)

    def disconnect_from_host(self):
        self._socket.abort()
        self._buffer = b""
        self._pending.clear()

    def is_connected(self) -> bool:
        return self._socket.state() == QAbstractSocket.SocketState.ConnectedState

    def send(self, command: str):
        if not self.is_connected():
            return
        self._pending.append(command)
        self._socket.write((command + "\n").encode("utf-8"))

    def _on_disconnected(self):
        self._buffer = b""
        self._pending.clear()
        self.disconnected.emit()

    def _on_ready_read(self):
        self._buffer += bytes(self._socket.readAll())
        while b"\n" in self._buffer:
            raw_line, self._buffer = self._buffer.split(b"\n", 1)
            self._handle_line(raw_line.decode("utf-8", errors="replace").strip())

    def _handle_line(self, line: str):
        command = self._pending.popleft() if self._pending else ""
        if not line:
            return
        if line.startswith("OK STATUS"):
            self.statusReceived.emit(parse_status(line))
        elif line.startswith("ERR"):
            self.commandFailed.emit(command, line)
        # plain "OK" acknowledgements need no further handling
