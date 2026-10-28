#!/usr/bin/env python3
"""Bridge the PIC's Modbus RTU registers to a Modbus TCP server."""

from __future__ import annotations

import argparse
import csv
import logging
import socket
import socketserver
import struct
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import serial

RTU_REGISTER_COUNT = 12
TCP_REGISTER_COUNT = 22
BRIDGE_STATUS_REGISTER = 4
FFT_FREQUENCY_REGISTERS = (5, 6, 7)  # centi-Hz
FFT_AMPLITUDE_REGISTERS = (8, 9, 10)  # milli-g
FFT_STATUS_REGISTER = 11
PIC_SEQUENCE_REGISTER = 12
PIC_OVERRUN_REGISTER = 13
HOST_MISSED_REGISTER = 14
SAMPLE_RATE_REGISTER = 15  # centi-Hz
RMS_REGISTERS = (16, 17, 18)  # mg
PEAK_TO_PEAK_REGISTERS = (19, 20, 21)  # mg
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
            tail = read_exact(port, 2, deadline)
            frame = first + header + tail
            if len(tail) == 2 and crc16_modbus(frame[:-2]) == int.from_bytes(frame[-2:], "little"):
                raise RuntimeError(f"PIC Modbus exception 0x{header[1]:02X}")
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


class SpectrumCSVWriter:
    """Append non-overlapping complete FFT spectra to a CSV file."""

    def __init__(self, path: str) -> None:
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = output_path.open("a", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        if self._file.tell() == 0:
            self._writer.writerow(
                ("timestamp_utc", "sample_rate_hz", "frequency_hz", "x_g", "y_g", "z_g")
            )
            self._file.flush()

    def write_window(
        self, sample_rate: float, frequencies: list[float], spectra: list[list[float]]
    ) -> None:
        timestamp = datetime.now(timezone.utc).isoformat()
        for index in range(1, len(frequencies)):
            self._writer.writerow(
                (
                    timestamp,
                    f"{sample_rate:.6f}",
                    f"{frequencies[index]:.6f}",
                    *(f"{axis[index]:.9f}" for axis in spectra),
                )
            )
        self._file.flush()

    def close(self) -> None:
        self._file.close()


class SamplesCSVWriter:
    """Append received samples/events; host_received_utc is not a PIC acquisition time."""

    def __init__(self, path: str) -> None:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        self._file = output.open("a", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        try:
            if self._file.tell() == 0:
                self._writer.writerow((
                    "host_received_utc", "row_type", "sequence", "x_raw", "y_raw", "z_raw",
                    "x_g", "y_g", "z_g", "sensor_status", "communication_status",
                    "pic_overrun", "host_missed",
                ))
                self._file.flush()
        except OSError:
            self.close()
            raise

    def write_event(self, row_type, sequence, axes, sensor_status, communication_status,
                    pic_overrun, host_missed) -> None:
        raw = axes if axes is not None else ("", "", "")
        scaled = tuple(value / ACCEL_LSB_PER_G for value in axes) if axes is not None else raw
        self._writer.writerow((datetime.now(timezone.utc).isoformat(), row_type, sequence,
                               *raw, *scaled, sensor_status, communication_status,
                               pic_overrun, host_missed))
        self._file.flush()

    def close(self) -> None:
        try:
            self._file.close()
        except OSError as exc:
            logging.error("falha ao fechar CSV de amostras: %s", exc)


class RegisterState:
    def __init__(
        self,
        fft_window: int,
        expected_period: float,
        spectrum_writer: SpectrumCSVWriter | None = None,
        samples_writer: SamplesCSVWriter | None = None,
    ) -> None:
        # 0..3 espelham o PIC, 4 indica o estado da ponte, 5..11 armazenam a FFT, 12..15 contêm diagnósticos.
        self._registers = [0] * TCP_REGISTER_COUNT
        self._registers[3] = 1
        self._registers[4] = 1
        self._registers[FFT_STATUS_REGISTER] = 1
        self._lock = threading.Lock()
        self.last_update = 0.0
        self._fft_window = fft_window
        self._max_gap = max(expected_period * 3.0, 0.05)
        self._samples: deque[tuple[float, tuple[int, int, int]]] = deque(maxlen=fft_window)
        self._last_sequence: int | None = None
        self._last_sample_at: float | None = None
        self._missed_samples = 0
        self._spectrum_writer = spectrum_writer
        self._samples_since_export = 0
        self._samples_writer = samples_writer
        self._csv_last_sequence: int | None = None
        self._csv_error = None

    def _write_sample_event(self, row_type, sequence="", axes=None) -> None:
        if self._samples_writer is None:
            return
        try:
            self._samples_writer.write_event(
                row_type, sequence, axes,
                "" if row_type == "communication_error" else self._registers[3],
                self._registers[BRIDGE_STATUS_REGISTER],
                "" if row_type == "communication_error" else self._registers[PIC_OVERRUN_REGISTER],
                self._missed_samples,
            )
        except OSError as exc:
            logging.error("exportação de amostras desativada: %s", exc)
            self._samples_writer.close()
            self._samples_writer = None

    def update_from_pic(self, values: list[int]) -> None:
        now = time.monotonic()
        with self._lock:
            self._registers[:4] = values[:4]
            self._registers[BRIDGE_STATUS_REGISTER] = 0
            sequence = values[4]
            self._registers[PIC_SEQUENCE_REGISTER] = sequence
            self._registers[PIC_OVERRUN_REGISTER] = values[5]
            self._registers[
                RMS_REGISTERS[0] : PEAK_TO_PEAK_REGISTERS[-1] + 1
            ] = values[6:12]
            self.last_update = now

            if values[3] != 0:
                error = ("sensor_error", values[3])
                if self._csv_error != error:
                    self._write_sample_event("sensor_error", sequence)
                self._csv_error = error
                self._samples.clear()
                self._registers[FFT_STATUS_REGISTER] = 1
                self._last_sequence = None
                self._last_sample_at = None
                self._samples_since_export = 0
                return

            self._csv_error = None
            sequence_reset = False
            if self._last_sequence is not None:
                delta = (sequence - self._last_sequence) & 0xFFFF
                if delta == 0:
                    if self._last_sample_at is not None and now - self._last_sample_at > self._max_gap:
                        self._samples.clear()
                        self._samples_since_export = 0
                        self._registers[FFT_STATUS_REGISTER] = 1
                    return  # Consulta mais rápida que a aquisição: não duplica amostras.
                sequence_reset = sequence < self._last_sequence and not (
                    self._last_sequence >= 0xF000 and sequence < 0x1000
                )
                if sequence_reset or delta >= 0x8000:
                    self._samples.clear()
                    sequence_reset = True
                elif delta > 1:
                    self._missed_samples = min(0xFFFF, self._missed_samples + delta - 1)
            self._last_sequence = sequence
            self._last_sample_at = now
            self._samples_since_export += 1
            self._registers[HOST_MISSED_REGISTER] = self._missed_samples

            # Keep samples across brief serial glitches or skipped sequence numbers;
            # _calculate_fft interpolates them using their measured arrival times.
            # A long time gap still invalidates the window below.
            if self._samples and now - self._samples[-1][0] > self._max_gap:
                self._samples.clear()
                self._samples_since_export = 1
            signed_axes = tuple(value - 0x10000 if value & 0x8000 else value for value in values[:3])
            if sequence != self._csv_last_sequence:
                self._write_sample_event("sample", sequence, signed_axes)
                self._csv_last_sequence = sequence
            self._samples.append((now, signed_axes))

            if len(self._samples) >= 2:
                elapsed = self._samples[-1][0] - self._samples[0][0]
                if elapsed > 0:
                    self._registers[SAMPLE_RATE_REGISTER] = min(
                        0xFFFF, round((len(self._samples) - 1) / elapsed * 100.0)
                    )

            if len(self._samples) < self._fft_window:
                self._registers[FFT_STATUS_REGISTER] = 1
                return

            frequencies, spectra = self._calculate_fft()
            if not frequencies:
                self._registers[FFT_STATUS_REGISTER] = 1
                return
            times = [item[0] for item in self._samples]
            measured_rate = (len(times) - 1) / (times[-1] - times[0])
            self._registers[SAMPLE_RATE_REGISTER] = min(0xFFFF, round(measured_rate * 100.0))
            peak_frequencies: list[float] = []
            peak_amplitudes: list[float] = []
            for spectrum in spectra:
                peak_index = int(np.argmax(spectrum[1:]) + 1)
                peak_amplitudes.append(float(spectrum[peak_index]))
                peak_frequencies.append(
                    float(frequencies[peak_index]) if spectrum[peak_index] > 0.0 else 0.0
                )
            for reg, frequency in zip(FFT_FREQUENCY_REGISTERS, peak_frequencies):
                self._registers[reg] = min(0xFFFF, max(0, round(frequency * 100.0)))
            for reg, amplitude in zip(FFT_AMPLITUDE_REGISTERS, peak_amplitudes):
                self._registers[reg] = min(0xFFFF, max(0, round(amplitude * 1000.0)))
            self._registers[FFT_STATUS_REGISTER] = 0
            if self._spectrum_writer and self._samples_since_export >= self._fft_window:
                try:
                    self._spectrum_writer.write_window(measured_rate, frequencies, spectra)
                except OSError as exc:
                    logging.error("exportação do espectro desativada: %s", exc)
                    self._spectrum_writer.close()
                    self._spectrum_writer = None
                self._samples_since_export = 0

    def _calculate_fft(self) -> tuple[list[float], list[list[float]]]:
        times = np.asarray([item[0] for item in self._samples], dtype=np.float64)
        samples = np.asarray([item[1] for item in self._samples], dtype=np.float64)
        duration = times[-1] - times[0]
        if duration <= 0:
            return [], [[], [], []]

        sample_rate = (len(times) - 1) / duration
        uniform_times = np.linspace(times[0], times[-1], len(times))
        values_g = samples / ACCEL_LSB_PER_G
        window = np.hanning(len(values_g))
        frequencies = np.fft.rfftfreq(len(values_g), d=1.0 / sample_rate)
        spectra: list[list[float]] = []

        for axis in range(3):
            uniform_values = np.interp(uniform_times, times, values_g[:, axis])
            centered = uniform_values - np.mean(uniform_values)
            spectrum = np.abs(np.fft.rfft(centered * window)) * (2.0 / np.sum(window))
            spectrum[0] = 0.0
            if len(values_g) % 2 == 0:
                spectrum[-1] *= 0.5  # Nyquist bin is not doubled in a one-sided spectrum.
            spectra.append(spectrum.tolist())

        return frequencies.tolist(), spectra

    def mark_communication_error(self) -> None:
        with self._lock:
            self._registers[BRIDGE_STATUS_REGISTER] = 1
            self._registers[FFT_STATUS_REGISTER] = 1
            if self._csv_error != ("communication_error",):
                self._write_sample_event("communication_error")
            self._csv_error = ("communication_error",)

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
    stop_event: threading.Event,
) -> None:
    last_error_log = 0.0
    previous_failed = False
    while not stop_event.is_set():
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

        stop_event.wait(max(0.0, poll_interval - (time.monotonic() - started)))


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
    parser.add_argument("port", help="porta serial do CH340/USB-UART, por exemplo COM4 ou /dev/ttyUSB0")
    parser.add_argument("--host", default="0.0.0.0", help="IP de escuta TCP (padrão: todas as interfaces)")
    parser.add_argument("--tcp-port", type=int, default=5020, help="porta Modbus TCP (padrão: 5020)")
    parser.add_argument("--slave-id", type=int, default=1, help="endereço RTU do PIC (padrão: 1)")
    parser.add_argument("--baud", type=int, default=115200, help="baud RTU (padrão: 115200)")
    parser.add_argument("--poll-ms", type=float, default=4.0, help="intervalo de consulta (padrão: 4 ms)")
    parser.add_argument("--fft-window", type=int, default=512, help="amostras por janela FFT (padrão: 512)")
    parser.add_argument(
        "--spectrum-csv",
        metavar="ARQUIVO",
        help="salva espectros completos em CSV, uma linha por frequência e janela",
    )
    parser.add_argument("--timeout", type=float, default=0.05, help="timeout por consulta RTU em segundos")
    parser.add_argument("--samples-csv", metavar="ARQUIVO",
                        help="salva amostras e erros em CSV; host_received_utc é recepção UTC no PC, não aquisição no PIC")
    parser.add_argument("--retries", type=int, default=1, help="repetições antes de marcar falha")
    args = parser.parse_args()
    if args.samples_csv and args.spectrum_csv:
        samples_path = Path(args.samples_csv).resolve()
        spectrum_path = Path(args.spectrum_csv).resolve()
        if samples_path == spectrum_path or (
            samples_path.exists() and spectrum_path.exists() and samples_path.samefile(spectrum_path)
        ):
            parser.error("--samples-csv e --spectrum-csv devem usar arquivos diferentes")

    if not 1 <= args.slave_id <= 247:
        parser.error("--slave-id deve estar entre 1 e 247")
    if not 1 <= args.tcp_port <= 65535:
        parser.error("--tcp-port deve estar entre 1 e 65535")
    if args.poll_ms < 3 or args.fft_window < 16 or args.retries < 0 or args.timeout <= 0:
        parser.error("--poll-ms deve ser >= 3, --fft-window >= 16, --retries >= 0 e --timeout > 0")

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    spectrum_writer = None
    samples_writer = None
    poll_stop = threading.Event()
    poller = None
    try:
        if args.spectrum_csv:
            spectrum_writer = SpectrumCSVWriter(args.spectrum_csv)
        if args.samples_csv:
            try:
                samples_writer = SamplesCSVWriter(args.samples_csv)
            except OSError as exc:
                logging.error("exportação de amostras desativada: %s", exc)
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
            state = RegisterState(args.fft_window, poll_interval, spectrum_writer, samples_writer)
            poller = threading.Thread(
                target=poll_pic,
                args=(
                    serial_port,
                    state,
                    args.slave_id,
                    poll_interval,
                    args.timeout,
                    args.retries,
                    poll_stop,
                ),
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
                logging.info("TCP: holding registers 0..21; FFT em janela de %d amostras", args.fft_window)
                logging.info("amostragem do PIC: 250 Hz nominal; frequência medida publicada no registrador 15")
                server.serve_forever()
    except KeyboardInterrupt:
        logging.info("ponte encerrada")
        return 0
    except (OSError, serial.SerialException) as exc:
        logging.error("não foi possível iniciar a ponte: %s", exc)
        return 1
    finally:
        poll_stop.set()
        if poller is not None:
            poller.join()
        if spectrum_writer is not None:
            spectrum_writer.close()
        if samples_writer is not None:
            samples_writer.close()


if __name__ == "__main__":
    raise SystemExit(main())
