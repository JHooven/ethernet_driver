#![allow(static_mut_refs)]
use stm32f4::stm32f429 as pac;

#[cfg(feature = "eth-driver")]
use fugit::RateExtU32;
#[cfg(feature = "eth-driver")]
use stm32_eth::{self, dma::EthernetDMA, dma::RxRingEntry, dma::TxRingEntry, EthPins};
#[cfg(feature = "eth-driver")]
// PHY integration can be added via stm32_eth::mac::phy if desired
#[cfg(feature = "eth-driver")]
use stm32_eth::hal::{gpio::GpioExt, rcc::RccExt};
#[cfg(feature = "eth-driver")]
use stm32_eth::hal::gpio::Speed as GpioSpeed;
#[cfg(feature = "eth-driver")]
use stm32_eth::mac::phy::BarePhy;
#[cfg(feature = "eth-driver")]
use stm32_eth::mac::Pause;
#[cfg(feature = "eth-driver")]
use stm32_eth::mac::Phy;
#[cfg(feature = "eth-driver")]
use stm32_eth::mac::Speed;
#[cfg(feature = "eth-driver")]
use stm32_eth::hal::gpio::{Alternate, Pin};
#[cfg(feature = "eth-driver")]
use stm32_eth::mac::EthernetMACWithMii;

// Size constants
const MTU: usize = 1536; // includes Ethernet frame + some headroom

#[cfg(feature = "eth-driver")]
pub struct EthernetDriver {
    eth_dma: EthernetDMA<'static, 'static>,
    mac_mii: Option<EthernetMACWithMii<Pin<'A', 2, Alternate<11>>, Pin<'C', 1, Alternate<11>>>>,
    phy_addr: u8,
}

#[cfg(not(feature = "eth-driver"))]
pub struct EthernetDriver {
    mac: pac::ETHERNET_MAC,
    hclk_hz: u32,
}

impl EthernetDriver {
    #[cfg(feature = "eth-driver")]
    pub fn new(dp: pac::Peripherals) -> Self {
        // Configure clocks: HCLK must be >= 25 MHz for Ethernet
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(32.MHz()).hclk(32.MHz()).freeze();

        // Split GPIOs via HAL
        let gpioa = dp.GPIOA.split();
        let _gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        let gpiog = dp.GPIOG.split();

        // Build RMII pin mapping for STM32F429I-DISC1 (stm32-eth configures AF/speed)
        let eth_pins = EthPins {
            ref_clk: gpioa.pa1,
            crs: gpioa.pa7,
            tx_en: gpiog.pg11,
            tx_d0: gpiog.pg13,
            tx_d1: gpiog.pg14,
            rx_d0: gpioc.pc4,
            rx_d1: gpioc.pc5,
        };

        // MDIO/MDC pins (PA2/PC1) must be AF11 VeryHigh for MIIM
        let mut mdio = gpioa.pa2.into_alternate::<11>();
        mdio.set_speed(GpioSpeed::VeryHigh);
        let mut mdc = gpioc.pc1.into_alternate::<11>();
        mdc.set_speed(GpioSpeed::VeryHigh);

        // DMA rings must be in RAM accessible by peripheral (not CCM)
        use core::mem::MaybeUninit;
        static mut RX_RING: MaybeUninit<[RxRingEntry; 16]> = MaybeUninit::uninit();
        static mut TX_RING: MaybeUninit<[TxRingEntry; 8]> = MaybeUninit::uninit();

        let parts = stm32_eth::PartsIn {
            mac: dp.ETHERNET_MAC,
            mmc: dp.ETHERNET_MMC,
            dma: dp.ETHERNET_DMA,
            ptp: dp.ETHERNET_PTP,
        };

        // Safety: RX_RING/TX_RING are static and live for 'static
        let rx_entries = unsafe { RX_RING.write(core::array::from_fn(|_| RxRingEntry::new())) };
        let tx_entries = unsafe { TX_RING.write(core::array::from_fn(|_| TxRingEntry::new())) };

        let stm32_eth::Parts { dma: eth_dma, mac: mac_mii, .. } =
            stm32_eth::new_with_mii(parts, &mut rx_entries[..], &mut tx_entries[..], clocks, eth_pins, mdio, mdc)
                .expect("stm32-eth init");

        Self { eth_dma, mac_mii: Some(mac_mii), phy_addr: 0 }
    }

    #[cfg(not(feature = "eth-driver"))]
    pub fn new(dp: pac::Peripherals) -> Self {
        let hclk_hz = 16_000_000u32; // if you configure clocks later, update this
        Self { mac: dp.ETHERNET_MAC, hclk_hz }
    }

    // Initialize the MAC, DMA and the PHY at a given MDIO address
    #[cfg(feature = "eth-driver")]
    pub fn init(&mut self, phy_addr: u8) {
        // Initialize external PHY via MIIM and wait for link
        self.phy_addr = phy_addr;
        let mac_mii = self.mac_mii.take().expect("MAC with MII unavailable");
        let mut phy = BarePhy::new(mac_mii, self.phy_addr, Pause::Symmetric);
        // Reset and autoneg
        phy.blocking_reset();
        let ad = phy.best_supported_advertisement();
        phy.set_autonegotiation_advertisement(ad);
        // Wait until autonegotiation completes and link is up (bounded spins)
        const MAX_SPINS: u32 = 5_000_000;
        let mut spins: u32 = 0;
        while !phy.autoneg_completed() {
            spins = spins.wrapping_add(1);
            if spins >= MAX_SPINS { break; }
        }
        if spins >= MAX_SPINS { crate::log::warn("PHY autonegotiation timeout"); }
        spins = 0;
        while !phy.phy_link_up() {
            spins = spins.wrapping_add(1);
            if spins >= MAX_SPINS { break; }
        }
        if spins >= MAX_SPINS { crate::log::warn("PHY link-up timeout"); }

        // Choose MAC speed/duplex based on PHY status
        let st = phy.status();
        let mac_speed = if st.fd_100base_x {
            Speed::FullDuplexBase100Tx
        } else if st.hd_100base_x {
            Speed::HalfDuplexBase100Tx
        } else if st.fd_10mbps {
            Speed::FullDuplexBase10T
        } else {
            Speed::HalfDuplexBase10T
        };

        // Release MIIM back to driver and set MAC speed
        let mut mac_mii = phy.release();
        mac_mii.set_speed(mac_speed);
        self.mac_mii = Some(mac_mii);
    }

    #[cfg(not(feature = "eth-driver"))]
    pub fn init(&mut self, phy_addr: u8) {
        unsafe {
            // Reset PHY
            mdio_write(&self.mac, self.hclk_hz, phy_addr, 0x00, 1 << 15);
            // Wait for reset clear
            for _ in 0..100_000 {
                let bmcr = mdio_read(&self.mac, self.hclk_hz, phy_addr, 0x00);
                if (bmcr & (1 << 15)) == 0 { break; }
            }
            // Enable auto-negotiation
            let mut bmcr = mdio_read(&self.mac, self.hclk_hz, phy_addr, 0x00);
            bmcr |= 1 << 12; // AN enable
            mdio_write(&self.mac, self.hclk_hz, phy_addr, 0x00, bmcr);
        }
    }

    pub fn link_up(&mut self) -> bool {
        #[cfg(feature = "eth-driver")]
        {
            // Query PHY quickly via MIIM
            let mac_mii = self.mac_mii.take().expect("MAC with MII unavailable");
            let mut phy = BarePhy::new(mac_mii, self.phy_addr, Pause::Symmetric);
            let up = phy.phy_link_up();
            self.mac_mii = Some(phy.release());
            up
        }
        #[cfg(not(feature = "eth-driver"))]
        {
            unsafe {
                let bmsr = mdio_read(&self.mac, self.hclk_hz, 0, 0x01);
                (bmsr & (1 << 2)) != 0
            }
        }
    }

    #[cfg(feature = "eth-driver")]
    pub fn transmit_frame(&mut self, _frame: &[u8]) -> Result<(), ()> {
        if _frame.len() > MTU { return Err(()); }
        self.eth_dma
            .send(_frame.len(), None, |buf| {
                buf[.._frame.len()].copy_from_slice(_frame);
            })
            .map_err(|_| ())
    }

    pub fn receive_frame(&mut self) -> Option<(usize, [u8; MTU])> {
        #[cfg(feature = "eth-driver")]
        {
            match self.eth_dma.recv_next(None) {
                Ok(pkt) => {
                    let len = (*pkt).len();
                    let mut buf = [0u8; MTU];
                    let copy_len = core::cmp::min(len, MTU);
                    buf[..copy_len].copy_from_slice(&*pkt);
                    Some((copy_len, buf))
                }
                Err(_) => None,
            }
        }
        #[cfg(not(feature = "eth-driver"))]
        {
            None
        }
    }
}

// Configure RMII pins for STM32F429 (typical mapping)
// - REF_CLK: PA1 (AF11)
// - MDIO:    PA2 (AF11)
// - CRS_DV:  PA7 (AF11)
// - MDC:     PC1 (AF11)
// - RXD0:    PC4 (AF11)
// - RXD1:    PC5 (AF11)
// - TX_EN:   PG11 (AF11)
// - TXD0:    PG13 (AF11)
// - TXD1:    PG14 (AF11)
// This uses raw PAC register writes to avoid a HAL dependency.
pub unsafe fn configure_rmii_pins(dp: &pac::Peripherals) {
    unsafe { configure_port_af11_gpioa(&dp.GPIOA, &[1, 2, 7]); }
    unsafe { configure_port_af11_gpioc(&dp.GPIOC, &[1, 4, 5]); }
    unsafe { configure_port_af11_gpiog(&dp.GPIOG, &[11, 13, 14]); }
}

unsafe fn configure_port_af11_gpioa(gpio: &pac::gpioa::RegisterBlock, pins: &[u8]) {
    for &pin in pins {
        unsafe { set_pin_af11_gpioa(gpio, pin); }
    }
}

unsafe fn configure_port_af11_gpioc(gpio: &pac::gpioc::RegisterBlock, pins: &[u8]) {
    for &pin in pins {
        unsafe { set_pin_af11_gpioc(gpio, pin); }
    }
}

unsafe fn configure_port_af11_gpiog(gpio: &pac::gpiog::RegisterBlock, pins: &[u8]) {
    for &pin in pins {
        unsafe { set_pin_af11_gpiog(gpio, pin); }
    }
}

unsafe fn set_pin_af11_gpioa(gpio: &pac::gpioa::RegisterBlock, pin: u8) {
    let shift = (pin * 2) as usize;
    gpio.moder.modify(|r, w| {
        let mut bits = r.bits();
        bits &= !(0b11 << shift);
        bits |= 0b10 << shift;
        unsafe { w.bits(bits) }
    });
    gpio.ospeedr.modify(|r, w| {
        let mut bits = r.bits();
        bits &= !(0b11 << shift);
        bits |= 0b11 << shift;
        unsafe { w.bits(bits) }
    });
    gpio.otyper.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << pin)) });
    gpio.pupdr.modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << shift)) });
    if pin < 8 {
        let shift = (pin * 4) as usize;
        gpio.afrl.modify(|r, w| {
            let mut bits = r.bits();
            bits &= !(0xF << shift);
            bits |= 11 << shift;
            unsafe { w.bits(bits) }
        });
    } else {
        let shift = ((pin - 8) * 4) as usize;
        gpio.afrh.modify(|r, w| {
            let mut bits = r.bits();
            bits &= !(0xF << shift);
            bits |= 11 << shift;
            unsafe { w.bits(bits) }
        });
    }
}

unsafe fn set_pin_af11_gpioc(gpio: &pac::gpioc::RegisterBlock, pin: u8) {
    let shift = (pin * 2) as usize;
    gpio.moder.modify(|r, w| {
        let mut bits = r.bits();
        bits &= !(0b11 << shift);
        bits |= 0b10 << shift;
        unsafe { w.bits(bits) }
    });
    gpio.ospeedr.modify(|r, w| {
        let mut bits = r.bits();
        bits &= !(0b11 << shift);
        bits |= 0b11 << shift;
        unsafe { w.bits(bits) }
    });
    gpio.otyper.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << pin)) });
    gpio.pupdr.modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << shift)) });
    if pin < 8 {
        let shift = (pin * 4) as usize;
        gpio.afrl.modify(|r, w| {
            let mut bits = r.bits();
            bits &= !(0xF << shift);
            bits |= 11 << shift;
            unsafe { w.bits(bits) }
        });
    } else {
        let shift = ((pin - 8) * 4) as usize;
        gpio.afrh.modify(|r, w| {
            let mut bits = r.bits();
            bits &= !(0xF << shift);
            bits |= 11 << shift;
            unsafe { w.bits(bits) }
        });
    }
}

unsafe fn set_pin_af11_gpiog(gpio: &pac::gpiog::RegisterBlock, pin: u8) {
    let shift = (pin * 2) as usize;
    gpio.moder.modify(|r, w| {
        let mut bits = r.bits();
        bits &= !(0b11 << shift);
        bits |= 0b10 << shift;
        unsafe { w.bits(bits) }
    });
    gpio.ospeedr.modify(|r, w| {
        let mut bits = r.bits();
        bits &= !(0b11 << shift);
        bits |= 0b11 << shift;
        unsafe { w.bits(bits) }
    });
    gpio.otyper.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << pin)) });
    gpio.pupdr.modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << shift)) });
    if pin < 8 {
        let shift = (pin * 4) as usize;
        gpio.afrl.modify(|r, w| {
            let mut bits = r.bits();
            bits &= !(0xF << shift);
            bits |= 11 << shift;
            unsafe { w.bits(bits) }
        });
    } else {
        let shift = ((pin - 8) * 4) as usize;
        gpio.afrh.modify(|r, w| {
            let mut bits = r.bits();
            bits &= !(0xF << shift);
            bits |= 11 << shift;
            unsafe { w.bits(bits) }
        });
    }
}

// --- MDIO helpers using MAC registers (PAC) ---
// MACMIIAR fields from RM0090: PA (phy addr), RDA (reg addr), MB (busy), MW (write), CR (clock range)
#[cfg(not(feature = "eth-driver"))]
unsafe fn mdio_wait_not_busy(mac: &pac::ETHERNET_MAC) {
    while mac.macmiiar.read().mb().bit_is_set() {}
}

#[cfg(not(feature = "eth-driver"))]
unsafe fn mdio_set_cr(mac: &pac::ETHERNET_MAC, hclk_hz: u32) {
    // Choose CR based on HCLK; simple heuristic
    // 0b000: /16, 0b001: /26, 0b010: /42, 0b011: /62, 0b100: /102
    let cr_bits = if hclk_hz <= 20_000_000 { 0b000 } else if hclk_hz <= 35_000_000 { 0b001 } else if hclk_hz <= 60_000_000 { 0b010 } else if hclk_hz <= 100_000_000 { 0b011 } else { 0b100 };
    mac.macmiiar.modify(|r, w| unsafe {
        let mut bits = r.bits();
        // clear CR[4:2]
        bits &= !(0b111 << 2);
        // set CR bits
        bits |= (cr_bits as u32) << 2;
        w.bits(bits)
    });
}

#[cfg(not(feature = "eth-driver"))]
unsafe fn mdio_write(mac: &pac::ETHERNET_MAC, hclk_hz: u32, phy_addr: u8, reg_addr: u8, val: u16) {
    unsafe { mdio_wait_not_busy(mac); }
    unsafe { mdio_set_cr(mac, hclk_hz); }
    mac.macmiidr.write(|w| w.md().bits(val));
    mac.macmiiar.modify(|r, w| unsafe {
        let mut bits = r.bits();
        // Clear PA[15:11], RDA[10:6], MW, MB
        bits &= !(((0x1F as u32) << 11) | ((0x1F as u32) << 6) | (1 << 1) | (1 << 0));
        // Set PA, RDA, MW=1, MB=1
        bits |= ((phy_addr as u32 & 0x1F) << 11)
             | ((reg_addr as u32 & 0x1F) << 6)
             | (1 << 1)
             | 1;
        w.bits(bits)
    });
    unsafe { mdio_wait_not_busy(mac); }
}

#[cfg(not(feature = "eth-driver"))]
unsafe fn mdio_read(mac: &pac::ETHERNET_MAC, hclk_hz: u32, phy_addr: u8, reg_addr: u8) -> u16 {
    unsafe { mdio_wait_not_busy(mac); }
    unsafe { mdio_set_cr(mac, hclk_hz); }
    mac.macmiiar.modify(|r, w| unsafe {
        let mut bits = r.bits();
        // Clear PA[15:11], RDA[10:6], MW, MB
        bits &= !(((0x1F as u32) << 11) | ((0x1F as u32) << 6) | (1 << 1) | (1 << 0));
        // Set PA, RDA, MW=0, MB=1
        bits |= ((phy_addr as u32 & 0x1F) << 11)
             | ((reg_addr as u32 & 0x1F) << 6)
             | 1;
        w.bits(bits)
    });
    unsafe { mdio_wait_not_busy(mac); }
    mac.macmiidr.read().md().bits()
}
