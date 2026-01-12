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

    // Reset Ethernet MAC
    rcc.ahb1rstr.modify(|_, w| w.ethmacrst().set_bit());
    rcc.ahb1rstr.modify(|_, w| w.ethmacrst().clear_bit());

    // Select RMII interface
    // Select RMII: set MII_RMII_SEL bit
    syscfg.pmc.modify(|_, w| w.mii_rmii_sel().set_bit());

    // Configure RMII pins to AF11, very high speed, push-pull, no pull
    unsafe { eth::configure_rmii_pins(&dp) };

    // Initialize Ethernet MAC/DMA (PHY wiring will be completed once confirmed)
    let mut driver = eth::EthernetDriver::new(dp);

    // Bring up link (will block/poll until link is up or timeout internally)
    driver.init(/*phy_addr=*/0);

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

// Trap faults early with a clear symbol so the debugger halts here
#[exception]
unsafe fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[exception]
unsafe fn DefaultHandler(_irqn: i16) {
    loop {
        cortex_m::asm::bkpt();
    }
}
