#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use cyw43::NetDriver;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio;
use embassy_rp::pio::Pio;
use embassy_time::{Duration, Timer};
use embassy_net::{Config, Stack, StackResources};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_25, PIO0};
use embassy_rp::uart;
use gpio::{Level, Output};
use cyw43_pio::PioSpi;
use static_cell::*;
use rust_mqtt::{
    client::{client::MqttClient, client_config::ClientConfig},
    packet::v5::reason_codes::ReasonCode,
    utils::rng_generator::CountingRng,
};
use embassy_net::tcp::{TcpSocket};
use embassy_futures::yield_now;
use smoltcp::wire::DnsQueryType;
#[allow(unused_imports)] use panic_probe;
#[allow(unused_imports)] use defmt_rtt;

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static, PIN_23>, PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

async fn wait_for_config(stack: &'static Stack<NetDriver<'static>>) -> embassy_net::StaticConfigV4 {
    loop {
        if let Some(config) = stack.config_v4() {
            return config.clone();
        }
        yield_now().await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialise Peripherals
    let p = embassy_rp::init(Default::default());


    // Create LED
    let mut led = Output::new(p.PIN_2, Level::Low);

    let fw = include_bytes!("../wifi_firmware/43439A0.bin");
    let clm = include_bytes!("../wifi_firmware/43439A0_clm.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0);
    let spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);


    let state = make_static!(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(_spawner.spawn(wifi_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::Performance)
        .await;

    let config = Config::dhcpv4(Default::default());

    // Generate random seed
    let seed = 0x0123_4567_89ab_cdef; // chosen by fair dice roll. guarenteed to be random.

    // Init network stack
    let stack = &*make_static!(Stack::new(
        net_device,
        config,
        make_static!(StackResources::<15>::new()),
        seed
    ));


    unwrap!(_spawner.spawn(net_task(stack)));

    let uart_config = uart::Config::default();

    let mut uart = uart::Uart::new_blocking(p.UART0, p.PIN_0, p.PIN_1, uart_config);
    info!("info log");
    uart.blocking_write("rusty_iot starting!\r\n".as_bytes()).unwrap();


    let wifi_ssid = include!("../WIFI_SSID");
    let wifi_pwd = include!("../WIFI_PWD");


    loop {
        uart.blocking_write("Trying Wifi\r\n".as_bytes()).unwrap();
        match control.join_wpa2(wifi_ssid, wifi_pwd).await {
            Ok(_) => {
                uart.blocking_write("Connected to WIFI!!\r\n".as_bytes()).unwrap();
                break;
            },
            Err(_) => {
                uart.blocking_write("WIFI Failed!!\r\n".as_bytes()).unwrap();
            }
        }
    }

    uart.blocking_write("Waiting for DHCP\r\n".as_bytes()).unwrap();
    let cfg = wait_for_config(stack).await;
    let local_addr = cfg.address.address();
    info!("IP Address {:?}",local_addr);
    uart.blocking_write("Got IP Address\r\n".as_bytes()).unwrap();


    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    // Log
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(10)));
    uart.blocking_write("LED On\r\n".as_bytes()).unwrap();
    // Turn LED On
    led.set_high();
    uart.blocking_write("Connecting On\r\n".as_bytes()).unwrap();

    let host_addr = stack.dns_query("test.mosquitto.org",DnsQueryType::A).await.unwrap()[0];

    socket.connect((host_addr, 1883))
        .await
        .map_err(|_| ReasonCode::NetworkError)
        .unwrap();

    let mut mqtt_config = ClientConfig::new(
        rust_mqtt::client::client_config::MqttVersion::MQTTv5,
        CountingRng(20000),
    );
    mqtt_config.add_max_subscribe_qos(rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1);
    mqtt_config.add_client_id("client");
    mqtt_config.max_packet_size = 100;
    let mut recv_buffer = [0; 80];
    let mut write_buffer = [0; 80];
    let mut mqtt_client = MqttClient::<_,5,_>::new(
        socket,
        &mut write_buffer,
        80,
        &mut recv_buffer,
        80,
        mqtt_config
    );
    mqtt_client.connect_to_broker().await.unwrap();

    mqtt_client.subscribe_to_topic("alexr/in").await.unwrap();

    loop {
        uart.blocking_write("Sending hello\r\n".as_bytes()).unwrap();
        led.set_high();
        mqtt_client
            .send_message(
                "alexr/out",
                b"hello2",
                rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS0,
                true,
            )
            .await
            .unwrap();
        let msg = mqtt_client.receive_message().await.unwrap().1;
        uart.blocking_write(msg).unwrap();
        uart.blocking_write("\r\n".as_bytes()).unwrap();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
