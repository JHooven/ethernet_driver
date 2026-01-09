# STM32F429I-DISC1 Ethernet Driver (Rust)

This workspace contains a minimal Ethernet MAC/DMA driver for the STM32F429 (DISC1 board) with an optional smoltcp ICMP echo responder.

## Board & Pins

- RMII (AF11, Very High speed):
  - REF_CLK: PA1
  - CRS_DV:  PA7
  - TX_EN:   PG11
  - TXD0:    PG13
  - TXD1:    PG14
  - RXD0:    PC4
  - RXD1:    PC5
- MDIO/MDC (AF11):
  - MDIO:    PA2
  - MDC:     PC1
- RMII is selected via `SYSCFG.PMC.MII_RMII_SEL`.

Ensure the external PHY (e.g., LAN8720) is wired for RMII and provides a 50 MHz REF_CLK to PA1.

## Defaults

- MAC: 02:00:00:00:00:01
- IPv4: 192.168.1.100/24

## Features

- `eth-driver`: enables `stm32-eth` DMA and smoltcp ICMP echo scaffold.

## Build

This project targets embedded (`thumbv7em-none-eabihf`). You will need:
- Rust target installed: `rustup target add thumbv7em-none-eabihf`
- A linker script `memory.x` for STM32F429 placed at workspace root.

Example commands:

```bash
# Check (host)
cargo check

# Build for embedded target with driver feature
cargo build --features eth-driver --target thumbv7em-none-eabihf
```

If linking fails with "memory.x not found", add a suitable linker script matching STM32F429 memory layout.

## Notes

- DMA RX/TX rings are placed in normal SRAM (not CCM). This is required for peripheral access per stm32-eth docs.
- MDIO/MDC pin setup is performed by low-level PAC helpers. For full MIIM-driven PHY initialization, consider using `stm32_eth::new_with_mii` and the `BarePhy` helper from `stm32_eth::mac::phy`.
- The ICMP echo loop is minimal; extend with UDP/TCP sockets as needed.
# STM32F429I Ethernet Driver (Rust)

This project scaffolds a no_std Rust firmware for STM32F429I (e.g. STM32F429ZI) with GPIO + RCC setup for RMII Ethernet. The Discovery board (STM32F429I-DISC1) does NOT include an onboard PHY, so an external RMII PHY (e.g. LAN8720A/LAN8742A/DP83848) must be wired.

## Hardware assumptions

- MCU: STM32F429 (tested targets: `STM32F429ZITx` package)
- External PHY: RMII (LAN8720 module). MDIO address assumed `0`.
- RMII pin mapping (typical, AF11):
  - REF_CLK: PA1
  - MDIO:    PA2
  - CRS_DV:  PA7
  - MDC:     PC1
  - RXD0:    PC4
  - RXD1:    PC5
  - TX_EN:   PG11
  - TXD0:    PG13
  - TXD1:    PG14
- SYSCFG configured for RMII and Ethernet clocks enabled.

Adjust pins to match your wiring if different.

## Status

- Compiles: yes (`thumbv7em-none-eabihf`)
- GPIO + SYSCFG + RCC setup for RMII: ready
- Ethernet MAC/DMA driver: placeholder (to be wired using `stm32-eth` once PHY is confirmed)
- Main loop: minimal scaffold

## Build and flash

Install the target and probe-rs runner if needed:

```bash
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools # provides `probe-rs`
```

Build/check:

```bash
cargo check
```

Flash (adjust chip if not STM32F429ZITx):

```bash
cargo run
```

The runner is configured in `.cargo/config.toml`.

## LAN8720 ↔ STM32F429I-DISC1 wiring (AF11)

- LAN8720 `REF_CLK` → `PA1`
- LAN8720 `MDIO`    → `PA2`
- LAN8720 `RX_DV`   → `PA7` (CRS_DV)
- LAN8720 `MDC`     → `PC1`
- LAN8720 `RXD0`    → `PC4`
- LAN8720 `RXD1`    → `PC5`
- LAN8720 `TX_EN`   → `PG11`
- LAN8720 `TXD0`    → `PG13`
- LAN8720 `TXD1`    → `PG14`
- LAN8720 `RESET`   → tie to 3.3V or a GPIO (optional)
- LAN8720 `3V3/GND` → board 3.3V/GND

Ensure a stable 50 MHz REF_CLK from LAN8720 to `PA1`.

## Next steps (enable real Ethernet)

1. Confirm your external PHY model and its MDIO address (often `0` or `1`).
2. Re-introduce the `stm32-eth` crate and wire the MAC/DMA + PHY:
   - Create RX/TX descriptor rings with static buffers
   - Initialize `EthernetDMA`/`EthernetMAC`
   - Create the PHY instance (e.g. `LAN8742A`) using the MAC's MII/MIIM interface
   - Reset and auto-negotiate link
   - Start DMA RX/TX and implement `send`/`recv_next`
3. Optionally integrate `smoltcp` for an IP stack (ARP, ICMP, UDP, etc.).

With the PHY (LAN8720) and MDIO address confirmed (`0`), I can complete the `eth.rs` driver using `stm32-eth` and provide a simple L2 echo or ICMP ping responder.

### Enable the driver feature (once wired)

The real driver is behind a feature so the project compiles on PC without it:

```bash
cargo check --features eth-driver
```

This pulls in `stm32-eth`, `heapless`, and `smoltcp`.

## Network configuration (default)

When the `eth-driver` feature is enabled, the example ICMP responder uses:

- MAC: `02:00:00:00:00:01`
- IPv4: `192.168.1.100/24`

You can change these in `src/main.rs` where the `NetStack` is initialized:

- `mac = smoltcp::wire::EthernetAddress([0x02, 0x00, 0x00, 0x00, 0x00, 0x01]);`
- `ip = smoltcp::wire::Ipv4Address::new(192, 168, 1, 100);`

Make sure your host machine is in the same subnet (e.g., `192.168.1.x/24`). Once the DMA is wired and the board is connected to your LAN, you should be able to ping the device:

```bash
ping 192.168.1.100
```

## Project layout

- `.cargo/config.toml`: target + runner
- `memory.x`: linker script for STM32F429ZI (2MB Flash, 256KB RAM)
- `src/main.rs`: no_std entry, RCC/SYSCFG, calls pin setup and driver
- `src/eth.rs`: RMII pin setup, MDIO helpers, and driver scaffold (feature-gated DMA)

## Quick test checklist

- LAN8720 powered at 3.3V, GND connected
- 50 MHz REF_CLK present on `PA1`
- RMII signals wired per table above
- MDIO address strapped to `0`
- Flash firmware: `cargo run`
- Link LED on RJ45 turns on after autonegotiation

## Logging

This firmware supports two optional logging paths:

- defmt RTT (recommended): enable richer, structured logs over RTT.
- Semihosting (fallback): simple messages printed via the debugger console.

### Enable defmt logging

Build with the `defmt-logging` feature alongside the driver:

```bash
cargo build --features "eth-driver defmt-logging" --target thumbv7em-none-eabihf
```

To view logs, use a runner that supports defmt decoding (e.g., `probe-run`). If you have `probe-run` configured as the runner in `.cargo/config.toml`, you can run:

```bash
cargo run --features "eth-driver defmt-logging"
```

Alternatively, use `probe-rs` with a separate defmt print tool:

```bash
# Flash using probe-rs
probe-rs run --chip STM32F429ZI target/thumbv7em-none-eabihf/debug/ethernet_driver

# In another terminal, attach a defmt printer to RTT (tooling varies)
```

### Semihosting fallback

If defmt is not enabled, the firmware logs via semihosting.
Ensure your debug setup supports semihosting; messages will appear in the debugger console.

### Adjust log level

You can change the defmt verbosity via the `DEFMT_LOG` environment variable. Supported levels: `error`, `warn`, `info`, `debug`, `trace`.

Examples:

```bash
# Highest verbosity
DEFMT_LOG=trace cargo run --features "eth-driver defmt-logging"

# Minimal verbosity
DEFMT_LOG=error cargo run --features "eth-driver defmt-logging"
```

A default of `info` is set in `.cargo/config.toml`. Override per-invocation using the env var as shown above.
