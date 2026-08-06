# Roki — Motherboard Firmware (STM32H7)

Bare-metal firmware for the STM32H743VI MCU on the motherboard of the **Roki**
humanoid robot (Starkit RoboCup team). Handles the robot's IMU and body/servo
bus, and talks to the host (Raspberry Pi) over a typed binary RPC protocol.

This repository is the **MCU side** of a two-part system:
- [**roki-mb-interface**](https://github.com/tndrd/roki-mb-interface) — host library (Raspberry Pi), C++ core + Python bindings
- **roki-mb-firmware** (this repo) — STM32H7 firmware (MCU side)

## What it does

- **Typed binary RPC** over serial with the host: one shared procedure descriptor
  drives request/response handling on both sides, so protocol mismatches are caught
  at compile time (see `RequestHandler`).
- **IMU integration** — Bosch BHI260 (BHI2 sensor hub), orientation streaming with
  sequence numbers so the host can detect dropped frames.
- **Body/servo communication** — command queue with synchronous and asynchronous
  (queued) modes, plus a **strobe filter** that timestamps and aligns incoming
  frames (`StrobeFilter`, `FrameQueue`).
- **Error reporting across the device boundary** — bus-level failures are encoded
  and returned to the host as named error codes rather than failing silently.

## Stack

C++ · bare-metal STM32H743VI (Cortex-M7) · custom linker scripts (FLASH/RAM) ·
HAL for peripherals · gtest-tested protocol layer (host side)

## Layout

- `roki-mb-firmware/` — application logic (protocol handler, IMU, body queue, strobe filter)
- `IMUHelpers/` — Bosch BHI260 driver integration
- `Core/`, `*.ld`, `startup_*.s` — startup, linker scripts, MCU bring-up
