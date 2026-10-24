#!/usr/bin/env python3
"""Receive MPU6050 packets from the PIC and print dominant FFT peaks."""

import argparse
import struct
import sys

import numpy as np
import serial

PACKET_SIZE = 13
SYNC = b"\xAA\x55"
SAMPLE_RATE_HZ = 250.0
ACCEL_LSB_PER_G = 16384.0


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def packets_from_serial(port: serial.Serial):
    buffer = bytearray()
    while True:
        buffer.extend(port.read(256))
        while True:
            start = buffer.find(SYNC)
            if start < 0:
                keep = buffer[-1:] if buffer.endswith(b"\xAA") else b""
                buffer.clear()
                buffer.extend(keep)
                break
            if start:
                del buffer[:start]
            if len(buffer) < PACKET_SIZE:
                break

            packet = bytes(buffer[:PACKET_SIZE])
            expected_crc = packet[11] | (packet[12] << 8)
            if crc16_modbus(packet[:11]) != expected_crc:
                del buffer[0]
                continue
            del buffer[:PACKET_SIZE]

            sequence, status, ax, ay, az = struct.unpack(">HBhhh", packet[2:11])
            yield sequence, status, (ax, ay, az)


def report_fft(samples: list[tuple[int, int, int]], window_name: str) -> None:
    values = np.asarray(samples, dtype=np.float64) / ACCEL_LSB_PER_G
    window = np.hanning(len(values))
    frequencies = np.fft.rfftfreq(len(values), d=1.0 / SAMPLE_RATE_HZ)
    print(f"\nFFT {window_name}: {len(values)} amostras, {len(values) / SAMPLE_RATE_HZ:.2f} s")

    for column, axis in enumerate(("X", "Y", "Z")):
        centered = values[:, column] - np.mean(values[:, column])
        spectrum = np.abs(np.fft.rfft(centered * window)) * (2.0 / np.sum(window))
        spectrum[0] = 0.0
        available = np.arange(1, len(frequencies))
        count = min(5, len(available))
        peaks = available[np.argpartition(spectrum[available], -count)[-count:]]
        peaks = sorted(peaks, key=lambda index: spectrum[index], reverse=True)
        formatted = ", ".join(f"{frequencies[i]:.2f} Hz ({spectrum[i]:.4f} g)" for i in peaks)
        print(f"  {axis}: {formatted}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="porta serial, por exemplo COM4 ou /dev/ttyUSB0")
    parser.add_argument("--window", type=int, default=2048, help="amostras por FFT (padrão: 2048)")
    args = parser.parse_args()
    if args.window < 16:
        parser.error("--window precisa ser pelo menos 16")

    samples: list[tuple[int, int, int]] = []
    last_sequence = None
    rejected = 0
    print(f"Conectando a {args.port} em 115200 bps; Ctrl+C para encerrar.")

    try:
        with serial.Serial(args.port, 115200, timeout=1) as port:
            for sequence, status, axes in packets_from_serial(port):
                if last_sequence is not None and sequence != ((last_sequence + 1) & 0xFFFF):
                    missing = (sequence - last_sequence - 1) & 0xFFFF
                    print(f"Perda de {missing} quadro(s); bloco FFT reiniciado.")
                    samples.clear()
                last_sequence = sequence

                if status != 0:
                    rejected += 1
                    samples.clear()
                    if rejected == 1 or rejected % 10 == 0:
                        print(f"Erro de leitura do MPU6050 ({rejected} no total).")
                    continue

                samples.append(axes)
                if len(samples) == args.window:
                    report_fft(samples, args.port)
                    samples.clear()
    except KeyboardInterrupt:
        print("\nColeta encerrada.")
    except serial.SerialException as exc:
        print(f"Erro na porta serial: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

