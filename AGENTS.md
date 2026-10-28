# Repository notes

- Firmware is the MPLAB X project for PIC18LF4685; build configuration is `default` with XC8 3.10 and PIC18Fxxxx_DFP 1.6.159. Command-line build: `make CONF=default build` from the project root, with MPLAB X/XC8 installed and the local generated toolchain paths valid.
- `main.c` owns the 4 ms Timer2 acquisition schedule; `mpu6050.c` implements I²C on RC3/RC4; `uart.c` implements UART and optional RS-485 DE control; `modbus.c` exposes the PIC's 12-register RTU map. The Python bridge in `pc/modbus_tcp_bridge.py` polls RTU and serves a separate 22-register TCP map.
- Keep the RTU PIC map (0–11) distinct from the TCP bridge map (0–21); the bridge-only FFT and diagnostics do not exist in PIC registers. `README.md` documents both maps and wiring.
- Python dependencies are in `pc/requirements.txt`. Run bridge tests with `python -m unittest discover -s tests -v`; syntax-check scripts with `python -m py_compile pc/modbus_tcp_bridge.py pc/receiver_fft.py`.
- The active serial link is Modbus RTU only; do not add startup/debug text to the PIC UART. The documented CH340 hookup is USB-UART TTL with crossed TX/RX and common GND, not RS-485 A/B.
- `pc/modbus_tcp_bridge.py --spectrum-csv <path>` optionally appends complete FFT windows to CSV; TCP registers expose only per-axis dominant peaks, not all FFT bins.
- `pc/receiver_fft.py` expects a binary `AA 55` packet stream not emitted by current firmware. `rtc_eeprom.c` is compiled but not connected to the active acquisition flow.
- MPLAB-generated makefiles under `nbproject/Makefile-*.mk`, `build/`, and `dist/` are generated or machine-specific. Do not edit them as source; builds may rewrite tracked artifacts. Preserve existing local changes to these files.
