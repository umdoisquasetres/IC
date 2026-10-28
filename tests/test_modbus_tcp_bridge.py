import csv
import math
import tempfile
import unittest
import time
from collections import deque
from pathlib import Path
from unittest.mock import patch

from pc.modbus_tcp_bridge import (
    FFT_STATUS_REGISTER,
    RegisterState,
    SpectrumCSVWriter,
    append_crc,
    read_rtu_registers,
)


class FakeSerial:
    def __init__(self, response: bytes) -> None:
        self.response = bytearray(response)
        self.timeout = None
        self.written = b""

    def reset_input_buffer(self) -> None:
        pass

    def write(self, data: bytes) -> int:
        self.written = data
        return len(data)

    def flush(self) -> None:
        pass

    def read(self, count: int) -> bytes:
        result = bytes(self.response[:count])
        del self.response[:count]
        return result


class ReadRTURegistersTests(unittest.TestCase):
    def test_reads_register_values(self) -> None:
        response = append_crc(bytes((1, 3, 4, 0x12, 0x34, 0xAB, 0xCD)))
        port = FakeSerial(response)

        values = read_rtu_registers(port, 1, 0, 2, 0.1)

        self.assertEqual(values, [0x1234, 0xABCD])
        self.assertEqual(port.written, append_crc(bytes((1, 3, 0, 0, 0, 2))))

    def test_decodes_five_byte_exception_frame(self) -> None:
        response = append_crc(bytes((1, 0x83, 0x02)))
        port = FakeSerial(response)

        with self.assertRaisesRegex(RuntimeError, "exception 0x02"):
            read_rtu_registers(port, 1, 0, 12, 0.1)


class RegisterStateTests(unittest.TestCase):
    @staticmethod
    def pic_values(sequence: int) -> list[int]:
        return [100, 200, 0xFF00, 0, sequence, 0, 0, 0, 0, 0, 0, 0]

    def test_pic_restart_clears_fft_samples_without_counting_huge_loss(self) -> None:
        state = RegisterState(fft_window=16, expected_period=0.004)
        state._last_sequence = 50000
        state._samples.append((time.monotonic(), (1, 2, 3)))

        state.update_from_pic(self.pic_values(0))

        self.assertEqual(len(state._samples), 1)
        self.assertEqual(state._samples[-1][1], (100, 200, -256))
        self.assertEqual(state._missed_samples, 0)
        self.assertEqual(state._registers[FFT_STATUS_REGISTER], 1)

    def test_repeated_sequence_invalidates_stale_fft(self) -> None:
        state = RegisterState(fft_window=16, expected_period=0.004)
        state._last_sequence = 10
        state._last_sample_at = time.monotonic() - 1.0
        state._samples = deque([(state._last_sample_at, (1, 2, 3))], maxlen=16)
        state._registers[FFT_STATUS_REGISTER] = 0

        state.update_from_pic(self.pic_values(10))

        self.assertEqual(len(state._samples), 0)
        self.assertEqual(state._registers[FFT_STATUS_REGISTER], 1)

    def test_normal_sequence_wrap_is_not_treated_as_restart(self) -> None:
        state = RegisterState(fft_window=16, expected_period=0.004)
        state._last_sequence = 0xFFFF
        state._samples.append((time.monotonic(), (1, 2, 3)))

        state.update_from_pic(self.pic_values(0))

        self.assertEqual(len(state._samples), 2)
        self.assertEqual(state._missed_samples, 0)

    def test_fft_peak_and_spectrum_csv_for_known_tone(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "spectrum.csv"
            writer = SpectrumCSVWriter(str(output))
            state = RegisterState(fft_window=16, expected_period=0.004, spectrum_writer=writer)
            times = [index * 0.004 for index in range(16)]
            with patch("pc.modbus_tcp_bridge.time.monotonic", side_effect=times):
                for index in range(16):
                    axis_x = round(0.1 * 16384 * math.sin(2 * math.pi * 2 * index / 16)) & 0xFFFF
                    state.update_from_pic(
                        [axis_x, 0, 0, 0, index, 0, 0, 0, 0, 0, 0, 0]
                    )
            writer.close()

            with output.open(newline="", encoding="utf-8") as spectrum_file:
                rows = list(csv.DictReader(spectrum_file))

            self.assertEqual(len(rows), 8)  # Positive-frequency bins; DC is omitted.
            self.assertEqual(state.read(5, 1), [3125])  # 31.25 Hz in centi-Hz.
            self.assertAlmostEqual(float(rows[1]["frequency_hz"]), 31.25)
            self.assertAlmostEqual(float(rows[1]["x_g"]), 0.1, delta=0.002)

    def test_spectrum_csv_is_written_across_brief_rtu_gaps(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "spectrum.csv"
            writer = SpectrumCSVWriter(str(output))
            state = RegisterState(fft_window=16, expected_period=0.004, spectrum_writer=writer)
            times = [index * 0.004 for index in range(18)]
            with patch("pc.modbus_tcp_bridge.time.monotonic", side_effect=times):
                for sequence in range(18):
                    if sequence == 5:
                        continue  # A missed PIC sample must not discard the whole FFT window.
                    if sequence == 8:
                        state.mark_communication_error()
                    axis_x = round(0.1 * 16384 * math.sin(2 * math.pi * sequence / 16)) & 0xFFFF
                    state.update_from_pic(
                        [axis_x, 0, 0, 0, sequence, 0, 0, 0, 0, 0, 0, 0]
                    )
            writer.close()

            with output.open(newline="", encoding="utf-8") as spectrum_file:
                rows = list(csv.DictReader(spectrum_file))

            self.assertEqual(len(rows), 8)
            self.assertEqual(state.read(11, 1), [0])
            self.assertEqual(state.read(14, 1), [1])

    def test_zero_signal_does_not_report_a_fake_peak(self) -> None:
        state = RegisterState(fft_window=16, expected_period=0.004)
        times = [index * 0.004 for index in range(16)]
        with patch("pc.modbus_tcp_bridge.time.monotonic", side_effect=times):
            for index in range(16):
                state.update_from_pic([0, 0, 0, 0, index, 0, 0, 0, 0, 0, 0, 0])

        self.assertEqual(state.read(5, 6), [0, 0, 0, 0, 0, 0])


if __name__ == "__main__":
    unittest.main()
