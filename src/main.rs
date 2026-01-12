#![no_std]
#![no_main]

use cortex_m_rt::{entry, exception};
use panic_halt as _;

// Ensure defmt RTT logger is linked when defmt-logging is enabled
#[cfg(feature = "defmt-logging")]
use defmt_rtt as _;

mod eth;
mod log;
#[cfg(feature = "eth-driver")]
mod net;

#[entry]
fn main() -> ! {
    #[cfg(feature = "defmt-logging")]
    crate::log::info("Booting ethernet_driver...");

    // Safety: take device peripherals once
    let dp = stm32f4::stm32f429::Peripherals::take().unwrap();

    // Enable configurable fault exceptions so we get more precise diagnostics
    unsafe {
        let scb = &*cortex_m::peripheral::SCB::PTR;
        // Set MEMFAULTENA (bit16), BUSFAULTENA (bit17), USGFAULTENA (bit18)
        scb.shcsr.modify(|r| r | ((1 << 16) | (1 << 17) | (1 << 18)));
    }

    // Enable clocks needed for SYSCFG and GPIO used by RMII
    let rcc = &dp.RCC;
    let syscfg = &dp.SYSCFG;

    // Enable GPIOA, GPIOC, GPIOG and SYSCFG clocks
    // Also enable Ethernet clocks (MAC, TX, RX)
    rcc.ahb1enr.modify(|_, w| {
        w.gpioaen().enabled()
            .gpiocen().enabled()
            .gpiogen().enabled()
            .ethmacen().enabled()
            .ethmactxen().enabled()
            .ethmacrxen().enabled()
    });
    #[cfg(feature = "defmt-logging")]
    crate::log::info("AHB1 clocks enabled");
    // SYSCFG sits on APB2; enable its clock before accessing SYSCFG registers
    rcc.apb2enr.modify(|_, w| w.syscfgen().enabled());
    #[cfg(feature = "defmt-logging")]
    crate::log::info("APB2 SYSCFG clock enabled");

    // Reset Ethernet MAC
    rcc.ahb1rstr.modify(|_, w| w.ethmacrst().set_bit());
    rcc.ahb1rstr.modify(|_, w| w.ethmacrst().clear_bit());
    // Ensure writes complete to catch bus faults precisely
    cortex_m::asm::dsb();

    // Select RMII interface
    // Select RMII: set MII_RMII_SEL bit
    syscfg.pmc.modify(|_, w| w.mii_rmii_sel().set_bit());
    // Ensure SYSCFG write is committed before proceeding
    cortex_m::asm::dsb();
    #[cfg(feature = "defmt-logging")]
    crate::log::info("SYSCFG RMII selected");

    // Configure RMII pins to AF11, very high speed, push-pull, no pull
    unsafe { eth::configure_rmii_pins(&dp) };
    #[cfg(feature = "defmt-logging")]
    crate::log::info("RMII pins configured");

    // Initialize Ethernet MAC/DMA (PHY wiring will be completed once confirmed)
    let mut driver = eth::EthernetDriver::new(dp);
    #[cfg(feature = "defmt-logging")]
    crate::log::info("EthernetDriver constructed");

    // Bring up link (will block/poll until link is up or timeout internally)
    driver.init(/*phy_addr=*/0);
    #[cfg(feature = "defmt-logging")]
    crate::log::info("EthernetDriver init complete");

    #[cfg(feature = "eth-driver")]
    {
        // Ensure link is up before starting the network stack (bounded spin)
        let mut spins: u32 = 0;
        const MAX_SPINS: u32 = 5_000_000;
        while !driver.link_up() {
            spins = spins.wrapping_add(1);
            if spins >= MAX_SPINS { break; }
        }
        #[cfg(feature = "eth-driver")]
        {
            if spins >= MAX_SPINS {
                crate::log::warn("Link-up timeout; starting stack anyway");
            }
        }

        // Minimal smoltcp ICMP responder scaffold
        static mut SOCKET_STORAGE: [smoltcp::iface::SocketStorage<'static>; 4] = [smoltcp::iface::SocketStorage::EMPTY; 4];
        let mac = smoltcp::wire::EthernetAddress([0x02, 0x00, 0x00, 0x00, 0x00, 0x01]);
        let ip = smoltcp::wire::Ipv4Address::new(172, 16, 10, 200);
        let mut stack = unsafe { net::NetStack::new(&mut driver, mac, ip, &mut SOCKET_STORAGE[..]) };

        let mut ticks: i64 = 0;
        loop {
            stack.poll(ticks);
            ticks = ticks.wrapping_add(1);
        }
    }

    #[cfg(not(feature = "eth-driver"))]
    loop {
        let _link = driver.link_up();
        let frame = driver.receive_frame();
        if let Some(frame) = frame {
            crate::log::info("Received Ethernet frame");
        }
    }
}

// Fault handlers: prefer RTT logging when enabled; otherwise break for debugging
#[exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    #[cfg(feature = "defmt-logging")]
    {
        defmt::error!(
            "HardFault pc=0x{:08x} lr=0x{:08x} xpsr=0x{:08x}",
            ef.pc(),
            ef.lr(),
            ef.xpsr()
        );
        // Log SCB fault status registers to categorize the fault
        let scb = unsafe { &*cortex_m::peripheral::SCB::PTR };
        let cfsr = scb.cfsr.read(); // Configurable Fault Status Register
        let hfsr = scb.hfsr.read(); // HardFault Status Register
        let mmfar = scb.mmfar.read(); // MemManage Fault Address
        let bfar = scb.bfar.read(); // BusFault Address
        defmt::error!(
            "SCB: CFSR=0x{:08x} HFSR=0x{:08x} MMFAR=0x{:08x} BFAR=0x{:08x}",
            cfsr,
            hfsr,
            mmfar,
            bfar
        );
        // Give RTT time to flush, then break for debugging
        for _ in 0..1_000_000 { cortex_m::asm::nop(); }
        cortex_m::asm::bkpt();
        loop { cortex_m::asm::wfi(); }
    }
    #[cfg(not(feature = "defmt-logging"))]
    {
        loop {
            cortex_m::asm::bkpt();
        }
    }
}

#[exception]
unsafe fn BusFault() -> ! {
    #[cfg(feature = "defmt-logging")]
    {
        let scb = &*cortex_m::peripheral::SCB::PTR;
        let cfsr = scb.cfsr.read();
        let bfar = scb.bfar.read();
        defmt::error!("BusFault: CFSR=0x{:08x} BFAR=0x{:08x}", cfsr, bfar);
        for _ in 0..1_000_000 { cortex_m::asm::nop(); }
        cortex_m::asm::bkpt();
        loop { cortex_m::asm::wfi(); }
    }
    #[cfg(not(feature = "defmt-logging"))]
    {
        loop {
            cortex_m::asm::bkpt();
        }
    }
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    #[cfg(feature = "defmt-logging")]
    {
        defmt::error!("Unhandled IRQ {}", irqn);
        loop {
            cortex_m::asm::wfi();
        }
    }
    #[cfg(not(feature = "defmt-logging"))]
    {
        loop {
            cortex_m::asm::bkpt();
        }
    }
}
