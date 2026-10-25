#!/usr/bin/env python3
"""Bridge the PIC's Modbus RTU registers to a Modbus TCP server."""

from __future__ import annotations

import argparse
import logging
import socket
import socketserver
import struct
import threading
import time
from collections import deque

import numpy as np
import serial

RTU_REGISTER_COUNT = 4
TCP_REGISTER_COUNT = 12
BRIDGE_STATUS_REGISTER = 4
FFT_FREQUENCY_REGISTERS = (5, 6, 7)  # centi-Hz
FFT_AMPLITUDE_REGISTERS = (8, 9, 10)  # milli-g
FFT_STATUS_REGISTER = 11
ACCEL_LSB_PER_G = 16384.0


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def append_crc(frame: bytes) -> bytes:
    crc = crc16_modbus(frame)
    return frame + bytes((crc & 0xFF, crc >> 8))


def read_exact(port: serial.Serial, count: int, deadline: float) -> bytes:
    data = bytearray()
    while len(data) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        port.timeout = min(0.05, remaining)
        chunk = port.read(count - len(data))
        if chunk:
            data.extend(chunk)
    return bytes(data)


def read_rtu_registers(
    port: serial.Serial, slave_id: int, start: int, count: int, timeout: float
) -> list[int]:
    request = append_crc(
        bytes((slave_id, 0x03, start >> 8, start & 0xFF, count >> 8, count & 0xFF))
    )
    port.reset_input_buffer()
    port.write(request)
    port.flush()

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        first = read_exact(port, 1, deadline)
        if not first:
            break
        if first[0] != slave_id:
            continue

        header = read_exact(port, 2, deadline)
        if len(header) != 2:
            break
        function, byte_count = header

        if function == (0x03 | 0x80):
            tail = read_exact(port, 3, deadline)
            frame = first + header + tail
            if len(tail) == 3 and crc16_modbus(frame[:-2]) == int.from_bytes(frame[-2:], "little"):
                raise RuntimeError(f"PIC Modbus exception 0x{tail[0]:02X}")
            continue

        if function != 0x03 or byte_count != count * 2:
            continue

        tail = read_exact(port, byte_count + 2, deadline)
        frame = first + header + tail
        if len(tail) != byte_count + 2:
            break
        if crc16_modbus(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
            continue

        return [int.from_bytes(frame[i : i + 2], "big") for i in range(3, 3 + byte_count, 2)]

    raise TimeoutError("sem resposta RTU válida do PIC (timeout ou CRC inválido)")


class RegisterState:
    def __init__(self, fft_window: int, expected_period: float) -> None:
        # 0..3 mirror the PIC, 4 is bridge status, 5..10 hold FFT peaks, 11 is FFT status.
        self._registers = [0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1]
        self._lock = threading.Lock()
        self.last_update = 0.0
        self._fft_window = fft_window
        self._max_gap = max(expected_period * 3.0, 0.25)
        self._samples: deque[tuple[float, tuple[int, int, int]]] = deque(maxlen=fft_window)

    def update_from_pic(self, values: list[int]) -> None:
        now = time.monotonic()
        with self._lock:
            self._registers[:RTU_REGISTER_COUNT] = values
            self._registers[BRIDGE_STATUS_REGISTER] = 0
            self.last_update = now

            if values[3] != 0:
                self._samples.clear()
                self._registers[FFT_STATUS_REGISTER] = 1
                return

            if self._samples and now - self._samples[-1][0] > self._max_gap:
                self._samples.clear()
            signed_axes = tuple(value - 0x10000 if value & 0x8000 else value for value in values[:3])
            self._samples.append((now, signed_axes))

            if len(self._samples) < self._fft_window:
                self._registers[FFT_STATUS_REGISTER] = 1
                return

            frequencies, amplitudes = self._calculate_fft()
            for reg, frequency in zip(FFT_FREQUENCY_REGISTERS, frequencies):
                self._registers[reg] = min(0xFFFF, max(0, round(frequency * 100.0)))
            for reg, amplitude in zip(FFT_AMPLITUDE_REGISTERS, amplitudes):
                self._registers[reg] = min(0xFFFF, max(0, round(amplitude * 1000.0)))
            self._registers[FFT_STATUS_REGISTER] = 0

    def _calculate_fft(self) -> tuple[list[float], list[float]]:
        times = np.asarray([item[0] for item in self._samples], dtype=np.float64)
        samples = np.asarray([item[1] for item in self._samples], dtype=np.float64)
        duration = times[-1] - times[0]
        if duration <= 0:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

        sample_rate = (len(times) - 1) / duration
        uniform_times = np.linspace(times[0], times[-1], len(times))
        values_g = samples / ACCEL_LSB_PER_G
        window = np.hanning(len(values_g))
        frequencies = np.fft.rfftfreq(len(values_g), d=1.0 / sample_rate)
        peak_frequencies: list[float] = []
        peak_amplitudes: list[float] = []

        for axis in range(3):
            uniform_values = np.interp(uniform_times, times, values_g[:, axis])
            centered = uniform_values - np.mean(uniform_values)
            spectrum = np.abs(np.fft.rfft(centered * window)) * (2.0 / np.sum(window))
            spectrum[0] = 0.0
            peak_index = int(np.argmax(spectrum[1:]) + 1)
            peak_frequencies.append(float(frequencies[peak_index]))
            peak_amplitudes.append(float(spectrum[peak_index]))

        return peak_frequencies, peak_amplitudes

    def mark_communication_error(self) -> None:
        with self._lock:
            self._registers[BRIDGE_STATUS_REGISTER] = 1
            self._registers[FFT_STATUS_REGISTER] = 1
            self._samples.clear()

    def read(self, start: int, count: int) -> list[int] | None:
        if count < 1 or start < 0 or start + count > TCP_REGISTER_COUNT:
            return None
        with self._lock:
            return self._registers[start : start + count]


def poll_pic(
    port: serial.Serial,
    state: RegisterState,
    slave_id: int,
    poll_interval: float,
    timeout: float,
    retries: int,
) -> None:
    last_error_log = 0.0
    previous_failed = False
    while True:
        started = time.monotonic()
        success = False
        error = ""
        for _ in range(retries + 1):
            try:
                values = read_rtu_registers(port, slave_id, 0, RTU_REGISTER_COUNT, timeout)
                state.update_from_pic(values)
                success = True
                break
            except (OSError, RuntimeError, TimeoutError, serial.SerialException) as exc:
                error = str(exc)

        if success:
            if previous_failed:
                logging.info("comunicação Modbus RTU com PIC restabelecida")
            previous_failed = False
        else:
            state.mark_communication_error()
            now = time.monotonic()
            if not previous_failed or now - last_error_log >= 10:
                logging.warning("falha ao consultar PIC: %s", error)
                last_error_log = now
            previous_failed = True

        time.sleep(max(0.0, poll_interval - (time.monotonic() - started)))


def recv_exact(sock: socket.socket, count: int) -> bytes:
    data = bytearray()
    while len(data) < count:
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise ConnectionError("cliente Modbus TCP desconectou")
        data.extend(chunk)
    return bytes(data)


def make_tcp_response(transaction_id: int, unit_id: int, pdu: bytes) -> bytes:
    mbap = struct.pack(">HHHB", transaction_id, 0, len(pdu) + 1, unit_id)
    return mbap + pdu


class ModbusTCPHandler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        state: RegisterState = self.server.register_state  # type: ignore[attr-defined]
        logging.info("cliente TCP conectado: %s", self.client_address[0])
        try:
            while True:
                mbap = recv_exact(self.request, 7)
                transaction_id, protocol_id, length, unit_id = struct.unpack(">HHHB", mbap)
                if length < 2 or length > 254:
                    break
                pdu = recv_exact(self.request, length - 1)
                response_pdu = self.process_pdu(state, pdu)
                if protocol_id != 0:
                    response_pdu = bytes((pdu[0] | 0x80, 0x01))
                self.request.sendall(make_tcp_response(transaction_id, unit_id, response_pdu))
        except (ConnectionError, OSError):
            pass
        finally:
            logging.info("cliente TCP desconectado: %s", self.client_address[0])

    @staticmethod
    def process_pdu(state: RegisterState, pdu: bytes) -> bytes:
        if not pdu:
            return bytes((0x80, 0x03))
        function = pdu[0]
        if function != 0x03:
            return bytes((function | 0x80, 0x01))
        if len(pdu) != 5:
            return bytes((function | 0x80, 0x03))

        start, count = struct.unpack(">HH", pdu[1:5])
        if count == 0 or count > 125:
            return bytes((function | 0x80, 0x03))
        values = state.read(start, count)
        if values is None:
            return bytes((function | 0x80, 0x02))

        data = b"".join(struct.pack(">H", value) for value in values)
        return bytes((function, len(data))) + data


class ModbusTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], state: RegisterState) -> None:
        super().__init__(address, ModbusTCPHandler)
        self.register_state = state


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="porta serial RS-485, por exemplo COM4 ou /dev/ttyUSB0")
    parser.add_argument("--host", default="0.0.0.0", help="IP de escuta TCP (padrão: todas as interfaces)")
    parser.add_argument("--tcp-port", type=int, default=5020, help="porta Modbus TCP (padrão: 5020)")
    parser.add_argument("--slave-id", type=int, default=1, help="endereço RTU do PIC (padrão: 1)")
    parser.add_argument("--baud", type=int, default=9600, help="baud RTU (padrão: 9600)")
    parser.add_argument("--poll-ms", type=int, default=40, help="intervalo RTU e amostragem da FFT (padrão: 40 ms)")
    parser.add_argument("--fft-window", type=int, default=128, help="amostras por janela FFT (padrão: 128)")
    parser.add_argument("--timeout", type=float, default=0.35, help="timeout por consulta RTU em segundos")
    parser.add_argument("--retries", type=int, default=1, help="repetições antes de marcar falha")
    args = parser.parse_args()

    if not 1 <= args.slave_id <= 247:
        parser.error("--slave-id deve estar entre 1 e 247")
    if not 1 <= args.tcp_port <= 65535:
        parser.error("--tcp-port deve estar entre 1 e 65535")
    if args.poll_ms < 25 or args.fft_window < 16 or args.retries < 0 or args.timeout <= 0:
        parser.error("--poll-ms deve ser >= 25, --fft-window >= 16, --retries >= 0 e --timeout > 0")

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        with serial.Serial(
            args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=0.5,
        ) as serial_port:
            poll_interval = args.poll_ms / 1000.0
            state = RegisterState(args.fft_window, poll_interval)
            poller = threading.Thread(
                target=poll_pic,
                args=(serial_port, state, args.slave_id, poll_interval, args.timeout, args.retries),
                daemon=True,
            )
            poller.start()

            with ModbusTCPServer((args.host, args.tcp_port), state) as server:
                logging.info(
                    "ponte ativa: RTU %s @ %d baud, escravo %d -> Modbus TCP %s:%d",
                    args.port,
                    args.baud,
                    args.slave_id,
                    args.host,
                    args.tcp_port,
                )
                logging.info("TCP: holding registers 0..11; FFT em janela de %d amostras", args.fft_window)
                logging.info("Nyquist nominal: %.2f Hz", 500.0 / args.poll_ms)
                server.serve_forever()
    except KeyboardInterrupt:
        logging.info("ponte encerrada")
        return 0
    except (OSError, serial.SerialException) as exc:
        logging.error("não foi possível iniciar a ponte: %s", exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
