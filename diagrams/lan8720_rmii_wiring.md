# LAN8720 ↔ STM32F429I-DISC1 (RMII) Wiring

Use RMII mode (SYSCFG.PMC.MII_RMII_SEL = 1). Set all listed GPIOs to AF11, Very High speed, push-pull, no pull.

## Pin Mapping

LAN8720 (module) → STM32F429I-DISC1 (MCU pin)

- REF_CLK → PA1  (RMII_REF_CLK, 50 MHz into MCU)
- MDIO    → PA2  (RMII_MDIO)
- MDC     → PC1  (RMII_MDC)
- CRS_DV  → PA7  (RMII_CRS_DV, aka RX_DV)
- RXD0    → PC4  (RMII_RXD0)
- RXD1    → PC5  (RMII_RXD1)
- TXEN    → PG11 (RMII_TX_EN)
- TXD0    → PG13 (RMII_TXD0)
- TXD1    → PG14 (RMII_TXD1)
- RESET   → 3V3 via pull-up or MCU GPIO (optional)
- 3V3     → 3.3V
- GND     → GND

> Note: REF_CLK must be a stable 50 MHz clock provided to PA1. Most LAN8720 breakout boards include a 50 MHz oscillator and drive REF_CLK out to the MAC.

## Quick ASCII Diagram

```
 LAN8720 Module                    STM32F429I-DISC1
 ┌───────────────┐                 ┌──────────────────────┐
 │ REF_CLK  ──────────►  PA1       │  RMII_REF_CLK (AF11) │
 │ MDIO     ──────────►  PA2       │  RMII_MDIO    (AF11) │
 │ MDC      ──────────►  PC1       │  RMII_MDC     (AF11) │
 │ CRS_DV   ──────────►  PA7       │  RMII_CRS_DV  (AF11) │
 │ RXD0     ──────────►  PC4       │  RMII_RXD0    (AF11) │
 │ RXD1     ──────────►  PC5       │  RMII_RXD1    (AF11) │
 │ TXEN     ◄──────────  PG11      │  RMII_TX_EN   (AF11) │
 │ TXD0     ◄──────────  PG13      │  RMII_TXD0    (AF11) │
 │ TXD1     ◄──────────  PG14      │  RMII_TXD1    (AF11) │
 │ RESET    ──┬─► 3V3 (PU)         │  or MCU GPIO         │
 │ 3V3       ─┘                    │  3.3V               │
 │ GND      ───────────────────────►│  GND                │
 └───────────────┘                 └──────────────────────┘
```

Arrows indicate signal direction relative to the MCU pins: RX signals are inputs to MCU; TX signals are outputs from MCU.

## Software Checklist

- Enable clocks: ETH MAC/TX/RX and the used GPIO ports.
- Select RMII: set `SYSCFG.PMC.MII_RMII_SEL = 1`.
- Configure pins: Alternate Function 11 (AF11), Very High speed.
- MDIO address: many LAN8720 modules strap PHY address to `0`; verify on your board.
- In code, ensure RMII pin mapping matches the above (see `src/eth.rs`).

## Troubleshooting

- No link: verify REF_CLK at PA1 is 50 MHz; check RJ45 magnetics and cable.
- MDIO/MDC: confirm pull-ups on MDIO line (some modules include them) and that PHY responds at the expected address.
- Swapped pairs: double-check RX/TX pin order and that CRS_DV maps to PA7.
