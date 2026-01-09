#![cfg(feature = "eth-driver")]

use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
use smoltcp::phy::{Device, DeviceCapabilities, Medium};
use smoltcp::socket::icmp;
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpCidr, Ipv4Address};

use crate::eth::EthernetDriver;

pub struct EthDevice<'d> {
    pub driver: &'d mut EthernetDriver,
}

impl<'d> Device for EthDevice<'d> {
    type RxToken<'a> = EthRxToken where 'd: 'a;
    type TxToken<'a> = EthTxToken<'a> where 'd: 'a;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 1500;
        caps.medium = Medium::Ethernet;
        caps
    }

    fn receive<'a>(&'a mut self, _timestamp: Instant) -> Option<(Self::RxToken<'a>, Self::TxToken<'a>)> {
        if let Some((len, frame)) = self.driver.receive_frame() {
            Some((EthRxToken { buf: frame, len }, EthTxToken { driver: self.driver }))
        } else {
            None
        }
    }

    fn transmit<'a>(&'a mut self, _timestamp: Instant) -> Option<Self::TxToken<'a>> {
        Some(EthTxToken { driver: self.driver })
    }
}

pub struct EthRxToken {
    buf: [u8; 1536],
    len: usize,
}

impl smoltcp::phy::RxToken for EthRxToken {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        // Pass the received frame to smoltcp
        let mut data = self.buf;
        let cap = core::cmp::min(self.len, data.len());
        f(&mut data[..cap])
    }
}

pub struct EthTxToken<'a> {
    pub driver: &'a mut EthernetDriver,
}

impl<'a> smoltcp::phy::TxToken for EthTxToken<'a> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut buf = [0u8; 1536];
        let cap = core::cmp::min(len, buf.len());
        let r = f(&mut buf[..cap]);
        let _ = self.driver.transmit_frame(&buf[..cap]);
        r
    }
}

pub struct NetStack<'s, 'd> {
    pub iface: Interface,
    pub sockets: SocketSet<'s>,
    pub icmp_handle: smoltcp::iface::SocketHandle,
    pub device: EthDevice<'d>,
    _phantom: core::marker::PhantomData<&'d ()>,
}

impl<'s, 'd> NetStack<'s, 'd> {
    pub fn new(driver: &'d mut EthernetDriver, mac: EthernetAddress, ip: Ipv4Address,
               sockets_storage: &'s mut [SocketStorage<'s>]) -> Self {
        let mut device = EthDevice { driver };

        let cfg = Config::new(smoltcp::wire::HardwareAddress::Ethernet(mac));
        let mut iface = Interface::new(cfg, &mut device, Instant::from_millis(0));
        iface
            .update_ip_addrs(|ip_addrs| {
                let _ = ip_addrs.push(IpCidr::new(ip.into(), 24));
            });

        let mut sockets = SocketSet::new(sockets_storage);

        // ICMP socket
        static mut ICMP_RX_META: [icmp::PacketMetadata; 8] = [icmp::PacketMetadata::EMPTY; 8];
        static mut ICMP_TX_META: [icmp::PacketMetadata; 8] = [icmp::PacketMetadata::EMPTY; 8];
        static mut ICMP_RX_BUF: [u8; 512] = [0; 512];
        static mut ICMP_TX_BUF: [u8; 512] = [0; 512];
        let rx_buf = unsafe { icmp::PacketBuffer::new(&mut ICMP_RX_META[..], &mut ICMP_RX_BUF[..]) };
        let tx_buf = unsafe { icmp::PacketBuffer::new(&mut ICMP_TX_META[..], &mut ICMP_TX_BUF[..]) };
        let icmp_socket = icmp::Socket::new(rx_buf, tx_buf);
        let icmp_handle = sockets.add(icmp_socket);

        NetStack { iface, sockets, icmp_handle, device, _phantom: core::marker::PhantomData }
    }

    pub fn poll(&mut self, now_millis: i64) {
        let timestamp = Instant::from_millis(now_millis);
        let _ = self.iface.poll(timestamp, &mut self.device, &mut self.sockets);

        let socket = self.sockets.get_mut::<icmp::Socket>(self.icmp_handle);
        while socket.can_recv() {
            if let Ok((payload, _src)) = socket.recv() {
                let src_ip = self.iface.ipv4_addr().unwrap().into_address();
                let size = payload.len();
                let mut tmp = [0u8; 512];
                let copy_len = core::cmp::min(size, tmp.len());
                tmp[..copy_len].copy_from_slice(&payload[..copy_len]);
                if let Ok(buf) = socket.send(copy_len, src_ip) {
                    buf[..copy_len].copy_from_slice(&tmp[..copy_len]);
                }
            }
        }
    }
}
