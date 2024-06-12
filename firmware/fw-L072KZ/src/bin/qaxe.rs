#![no_std]
#![no_main]

use core::option::Option::Some;
use defmt::{panic, *};
use defmt_rtt as _; // global logger
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::i2c;
use embassy_stm32::i2c::I2c;
use embassy_stm32::rcc::*;
use embassy_stm32::time::{khz, Hertz};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel as PWMChannel;
use embassy_stm32::usart::BufferedUart;
use embassy_stm32::usb::{Driver, Instance};
use embassy_stm32::{bind_interrupts, peripherals, usart, usb, Config};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embedded_io_async::Read;
use static_cell::StaticCell;
use embassy_time::{Delay, Timer};
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::peripherals::ADC;
use embassy_stm32::adc;
use embassy_stm32::rcc::mux::Clk48sel;

use embassy_stm32::timer::OutputPolarity;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use embedded_io_async::Write;
use futures::future::join5;
use panic_probe as _;

extern crate alloc;
extern crate alloc_cortex_m;

mod uid;

mod protobuf;
use protobuf::coms::{QControl, QRequest, QResponse, QState};
use quick_protobuf::{self, MessageWrite};

use alloc::borrow::Cow;

use alloc_cortex_m::CortexMHeap;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

bind_interrupts!(struct Irqs {
    USB => usb::InterruptHandler<peripherals::USB>;
    USART1 => usart::BufferedInterruptHandler<peripherals::USART1>;
    ADC1_COMP => adc::InterruptHandler<ADC>;
    I2C1 => i2c::EventInterruptHandler<peripherals::I2C1>, i2c::ErrorInterruptHandler<peripherals::I2C1>;
});
use embassy_stm32::peripherals::*;

#[derive(PartialEq)]
enum ResetManagerCommand {
    Reset,
}

#[derive(PartialEq)]
enum PowerManagerCommand {
    BuckOn,
    BuckOff,
}

//static RESET_MANAGER_SIGNAL: Signal<CriticalSectionRawMutex, ResetManagerCommand> = Signal::new();
static POWER_MANAGER_SIGNAL: Signal<CriticalSectionRawMutex, PowerManagerCommand> = Signal::new();

static AXE_CONTROL: Channel<ThreadModeRawMutex, Control, 1> = Channel::new();
static AXE_STATUS: Mutex<ThreadModeRawMutex, Status> = Mutex::new(Status {
    power_enabled: false,
    temp: [0i32; 4],
    domain: [0i32; 4],
});

#[derive(Clone, Copy)]
struct Control {
    led2: bool,
    pwm: [i32; 4],
}

#[derive(Clone, Copy)]
struct Status {
    power_enabled: bool,
    temp: [i32; 4],
    domain: [i32; 4],
}

impl From<Status> for QState {
    fn from(value: Status) -> Self {
        QState {
            pgood_1v2: 0,
            power_enabled: if value.power_enabled { 1 } else { 0 },
            temp1: value.temp[0],
            temp2: value.temp[1],
            temp3: value.temp[2],
            temp4: value.temp[3],
            domain1: value.domain[0],
            domain2: value.domain[1],
            domain3: value.domain[2],
            domain4: value.domain[3],
        }
    }
}

impl From<QControl> for Control {
    fn from(value: QControl) -> Self {
        Control {
            led2: value.led2 == 1,
            pwm: [value.pwm1, value.pwm2, value.pwm3, value.pwm4],
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");

    // Initialize the allocator before using it
    let start = cortex_m_rt::heap_start() as usize;
    let size = 1024;
    unsafe { ALLOCATOR.init(start, size) }

    let mut config = Config::default();
    config.rcc.hsi48 = Some(Hsi48Config {
        sync_from_usb: true,
    }); // needed for USB
    config.rcc.sys = Sysclk::PLL1_R;
    config.rcc.hsi = true;
    config.rcc.pll = Some(Pll {
        source: PllSource::HSI,
        div: PllDiv::DIV3,
        mul: PllMul::MUL6,
    });
    config.rcc.mux.clk48sel = Clk48sel::HSI48;

    let p = embassy_stm32::init(config);

    let driver = Driver::new(p.USB, Irqs, p.PA12, p.PA11);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.max_packet_size_0 = 64;
    config.manufacturer = Some("microengineer");
    config.product = Some("0xAxe");
    config.serial_number = Some(uid::uid_hex());

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut device_descriptor = [0; 256];
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state_usb_asic = State::new();
    let mut state_usb_ctrl = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let class_usb_asic = CdcAcmClass::new(&mut builder, &mut state_usb_asic, 64);
    let (mut sender, mut receiver) = class_usb_asic.split();

    let mut class_usb_ctrl = CdcAcmClass::new(&mut builder, &mut state_usb_ctrl, 64);

    // Build the builder.
    let mut usb = builder.build();

    let mut config = usart::Config::default();
    config.baudrate = 115200;

    static TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 256])[..];
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 256])[..];

    let usart = BufferedUart::new(p.USART1, Irqs, p.PA10, p.PA9, tx_buf, rx_buf, config).unwrap();
    let (mut tx_ctrl, mut rx_ctrl) = usart.split();

    // Run the USB device.
    let usb_fut = usb.run();

    let pwr_en = Output::new(p.PB1, Level::Low, Speed::Low);
    let pwr_sdn = Output::new(p.PB0, Level::High, Speed::Low);
    let pwr_enabled = Input::new(p.PC14, Pull::None);
    let pgood_led = Output::new(p.PA6, Level::High, Speed::Low);
    let led2 = Output::new(p.PA7, Level::High, Speed::Low);

    let reset = Output::new(p.PA8, Level::Low, Speed::Low);

    let ch2 = PwmPin::new_ch2(p.PB3, OutputType::PushPull);
    let mut pwm1 = SimplePwm::new(
        p.TIM2,
        None,
        Some(ch2),
        None,
        None,
        khz(10),
        Default::default(),
    );
    pwm1.set_polarity(PWMChannel::Ch2, OutputPolarity::ActiveHigh);
    pwm1.set_duty(PWMChannel::Ch2, pwm1.get_max_duty() / 2);
    pwm1.enable(PWMChannel::Ch2);

    let i2c = I2c::new(
        p.I2C1,
        p.PB6,
        p.PB7,
        Irqs,
        NoDma,
        NoDma,
        Hertz(100_000),
        Default::default(),
    );

    let mut adc = Adc::new(p.ADC, Irqs, &mut Delay);
    adc.set_sample_time(SampleTime::CYCLES160_5);

    let mut vrefint = adc.enable_vref(&mut Delay);
    let mut adc_pin_domain1 = p.PA0;
    let mut adc_pin_domain2 = p.PA1;
    let mut adc_pin_domain3 = p.PA2;
    let mut adc_pin_domain4 = p.PA3;

    let vrefint_sample = adc.read(&mut vrefint).await;

    let convert_to_millivolts = |sample| {
        // From https://www.st.com/resource/en/datasheet/stm32l051c6.pdf
        // 6.3.3 Embedded internal reference voltage
        const VREFINT_MV: u32 = 1224; // mV

        (u32::from(sample) * VREFINT_MV / u32::from(vrefint_sample)) as u16
    };

    let adc_fut = async {
        loop {
            let mut samples = [0u16; 4];
            for (i, sample) in samples.iter_mut().enumerate().take(4) {
                let value = match i {
                    0 => adc.read(&mut adc_pin_domain1).await,
                    1 => adc.read(&mut adc_pin_domain2).await,
                    2 => adc.read(&mut adc_pin_domain3).await,
                    3 => adc.read(&mut adc_pin_domain4).await,
                    _ => 0,
                };
                *sample = convert_to_millivolts(value);
            }
            let mut state = AXE_STATUS.lock().await;
            state.domain[0] = samples[0] as i32;
            state.domain[1] = samples[1] as i32;
            state.domain[2] = samples[2] as i32;
            state.domain[3] = samples[3] as i32;
            drop(state);
            info!("adc values: domain1={}, domain2={}, domain3={}, domain4={}", samples[0], samples[1], samples[2], samples[3]);
            Timer::after_millis(1000).await;
        }
    };

//    unwrap!(spawner.spawn(reset_manager(reset)));
    unwrap!(spawner.spawn(power_manager(pwr_en, pwr_sdn, reset)));
    unwrap!(spawner.spawn(power_good_task(pwr_enabled, pgood_led)));
    unwrap!(spawner.spawn(axe_control_channel(pwm1, led2)));
    unwrap!(spawner.spawn(temp_manager(i2c)));

    let protobuf_rpc_fut = async {
        loop {
            class_usb_ctrl.wait_connection().await;
            info!("Connected");
            let _ = protobuf_rpc(&mut class_usb_ctrl).await;
            info!("Disconnected");
        }
    };

    let relay_receiver_fut = async {
        loop {
            let mut usb_buf = [0; 64];
            receiver.wait_connection().await;
            info!("Connected relay receiver");

            loop {
                let usb_read = match receiver.read_packet(&mut usb_buf).await {
                    Ok(n) => n,
                    Err(e) => {
                        error!("Error reading from USB: {:?}", e);
                        break;
                    }
                };

                if usb_read == 0 {
                    continue; // No data read, continue the loop
                }

                debug!("USB -> USART: {:x}", &usb_buf[..usb_read]);

                if let Err(e) = tx_ctrl.write_all(&usb_buf[..usb_read]).await {
                    error!("Error writing to USART: {:?}", e);
                    break;
                }
            }
        }
    };

    let relay_sender_fut = async {
        loop {
            sender.wait_connection().await;

            info!("Connected relay sender");

            let mut toggle = 0;
            let mut num_bytes = 0;
            let mut received = [0u8; 11];
            let preample = [0xaa, 0x55];
            loop {
                let mut byte = [0u8; 1];
                match rx_ctrl.read_exact(&mut byte).await {
                    Ok(_) => (),
                    Err(e) => {
                        error!("Error reading from USART: {:?}", e);
                    }
                };

                received[num_bytes] = byte[0];

                // try to sync on serial data
                match num_bytes {
                    0 | 1 =>
                    // wait for 0xaa 0x55
                    {
                        if received[num_bytes] != preample[num_bytes] {
                            debug!("unexpected start of serial data, trying to resync ...");
                            num_bytes = 0;
                            continue;
                        }
                    }
                    _ => {}
                };

                num_bytes += 1;

                if num_bytes != 11 {
                    continue;
                }
                num_bytes = 0;
                /*
                                // toggle led with each response received
                                toggle = 1 - toggle;
                                match toggle {
                                    0 => activity_led.set_high(),
                                    1 => activity_led.set_low(),
                                    _ => {}
                                };
                */
                debug!("USART -> USB: {:x}", &received[..]);
                if let Err(e) = sender.write_packet(&received[..]).await {
                    error!("Error writing to USB: {:?}", e);
                    break;
                }
            }
        }
    };

    let _ = join5(
        usb_fut,
        protobuf_rpc_fut,
        relay_receiver_fut,
        relay_sender_fut,
        adc_fut,
    )
    .await;
}

#[embassy_executor::task]
async fn power_good_task(power_enabled: Input<'static>, mut pgood_led: Output<'static>) {
    loop {
        let mut state = AXE_STATUS.lock().await;

        if power_enabled.is_high() {
            state.power_enabled = true;
            pgood_led.set_low();
        } else {
            state.power_enabled = false;
            pgood_led.set_high();
        }
        drop(state);
        Timer::after_millis(500).await;
    }
}
#[embassy_executor::task]
async fn power_manager(mut pwr_en: Output<'static>, mut pwr_sdn: Output<'static>, mut reset: Output<'static>) {
    loop {
        let signal = POWER_MANAGER_SIGNAL.wait().await;

        if signal == PowerManagerCommand::BuckOn {
            info!("switching buck on");
            // disable shutdown and set reset
            pwr_sdn.set_low();
            reset.set_low();
            Timer::after_millis(100).await;
            // pulse pwr_en
            pwr_en.set_high();
            Timer::after_millis(100).await;
            pwr_en.set_low();
            Timer::after_millis(1000).await;
            // release reset
            reset.set_high();
        }

        if signal == PowerManagerCommand::BuckOff {
            info!("switching buck off");
            pwr_sdn.set_high();
            Timer::after_millis(10).await;
        }
        POWER_MANAGER_SIGNAL.reset();
    }
}
/*
#[embassy_executor::task]
async fn reset_manager(mut reset: Output<'static>) {
    loop {
        let signal = RESET_MANAGER_SIGNAL.wait().await;

        if signal != ResetManagerCommand::Reset {
            continue;
        }

        info!("reset triggered!");
        reset.set_low();
        Timer::after_millis(100).await;
        reset.set_high();
    }
}
*/
#[embassy_executor::task]
async fn axe_control_channel(mut pwm1: SimplePwm<'static, TIM2>, mut led2: Output<'static>) {
    loop {
        let control = AXE_CONTROL.receive().await;

        for i in 0..4 {
            match i {
                0 => {
                    let max_duty = pwm1.get_max_duty() as u32;
                    let duty = max_duty * control.pwm[i] as u32 / 100;
                    info!("pwm{}: {}, max: {}", i, duty, max_duty);
                    pwm1.set_duty(
                        PWMChannel::Ch2,
                        if duty <= max_duty {
                            duty as u16
                        } else {
                            max_duty as u16
                        },
                    );
                }
                _ => { /* NOP */ }
            };
        }
        if control.led2 {
            led2.set_low();
        } else {
            led2.set_high();
        }

        Timer::after_millis(500).await;
    }
}
struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

enum Errors {
    None = 0,
    InvalidCommand = 1,
    ErrorDeserializingRequest = 2,
    ErrorSerializingResponse = 3,
    ErrorDeserializingRequestData = 4,
    ErrorSerializingResponseData = 5,
}

impl Errors {
    fn to_string(error: &Errors) -> &'static str {
        match error {
            Errors::InvalidCommand => "invalid command",
            Errors::ErrorDeserializingRequest => "error deserializing request",
            Errors::ErrorSerializingResponse => "error serializing response",
            Errors::ErrorDeserializingRequestData => "error deserializing request data",
            Errors::ErrorSerializingResponseData => "error serializing response data",
            _ => "unknown error",
        }
    }
}
enum Commands {
    NOP = 0,
    Control = 1,
    Status = 2,
    Reset = 3,
    Shutdown = 4,
    PowerOn = 5,
}

impl Commands {
    fn from_i32(value: i32) -> Option<Commands> {
        match value {
            0 => Some(Commands::NOP),
            1 => Some(Commands::Control),
            2 => Some(Commands::Status),
            3 => Some(Commands::Reset),
            4 => Some(Commands::Shutdown),
            5 => Some(Commands::PowerOn),
            _ => None,
        }
    }
}

impl QResponse<'_> {
    fn default() -> QResponse<'static> {
        QResponse {
            id: 0,
            error: 0,
            data: Cow::Borrowed(&[0u8]),
        }
    }
}

// The response_bytes should be a mutable slice of u8, not a slice of a mutable slice.
async fn process_request<'a>(
    request: &QRequest<'_>,
    response: &mut QResponse<'_>,
) -> Result<usize, Errors> {
    let mut response_data = [0u8; 32];
    let mut response_len = 0;
    let error = Errors::None as i32;

    let op = Commands::from_i32(request.op);
    if op.is_none() {
        return Err(Errors::InvalidCommand);
    }

    match op.unwrap() {
        Commands::NOP => {
            // nop
        }
        Commands::Control => {
            let cmd: QControl = quick_protobuf::deserialize_from_slice(&request.data)
                .map_err(|_| Errors::ErrorDeserializingRequestData)?;

            info!(
                "received ctrl command with parameters led2: {}, pwm1: {}, pwm2: {}, pwm3: {}, pwm4: {}",
                cmd.led2, cmd.pwm1, cmd.pwm2, cmd.pwm3, cmd.pwm4
            );

            AXE_CONTROL.send(Control::from(cmd)).await;
        }
        Commands::Status => {
            info!("status");
            let xstate = AXE_STATUS.lock().await;
            let state = QState::from(*xstate);
            drop(xstate);

            response_len = state.get_size() + 1 /* varint */;

            debug!("response-len: {}", response_len);

            quick_protobuf::serialize_into_slice(&state, &mut response_data[..])
                .map_err(|_| Errors::ErrorSerializingResponseData)?;
        }
        Commands::Reset => {}, //RESET_MANAGER_SIGNAL.signal(ResetManagerCommand::Reset),
        Commands::Shutdown => POWER_MANAGER_SIGNAL.signal(PowerManagerCommand::BuckOff),
        Commands::PowerOn => POWER_MANAGER_SIGNAL.signal(PowerManagerCommand::BuckOn),
    };

    response.id = request.id;
    response.error = error;
    response.data = Cow::Owned(response_data[..response_len].to_vec());
    debug!(
        "response.id: {}, response.error:{}, response.data: {:?}",
        response.id,
        response.error,
        response_data[..response_len]
    );
    Ok(response_len)
}

async fn protobuf_rpc<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut request_bytes = [0u8; 64];
    let mut response_bytes = [0u8; 64];

    loop {
        let n = class.read_packet(&mut request_bytes).await?;

        let mut response = QResponse::default();

        let request = match quick_protobuf::deserialize_from_slice(&request_bytes[..n]) {
            Ok(req) => Some(req),
            Err(_) => {
                error!("{}", Errors::to_string(&Errors::ErrorDeserializingRequest));
                response = QResponse::default();
                response.error = Errors::ErrorDeserializingRequest as i32;
                None
            }
        };

        // if request is some then we can process the request
        if request.is_some() {
            if let Err(e) = process_request(&request.unwrap(), &mut response).await {
                error!("{}", Errors::to_string(&e));
                response = QResponse::default();
                response.error = e as i32;
            }
        }

        let serialized_len = response.get_size() + 1 /* varint */;
        if quick_protobuf::serialize_into_slice(&response, &mut response_bytes).is_err() {
            error!("{}", Errors::to_string(&Errors::ErrorSerializingResponse));
            continue;
        }

        class
            .write_packet(&response_bytes[..serialized_len])
            .await?;
    }
}

#[embassy_executor::task]
async fn temp_manager(mut i2c: I2c<'static, I2C1>) {
    loop {
        for i in 0..2 {
            let mut data = [0u8; 2];
            if let Err(e) = i2c.blocking_read(0x48 + i, &mut data) {
                error!("i2c error: {:?}", e);
                continue;
            }

            let mut temp_data = ((data[0] as u16) << 4) | ((data[1] as u16) >> 4);

            if temp_data > 2047 {
                temp_data -= 4096
            }

            info!("read temp{}: {}", i, temp_data);

            let mut state = AXE_STATUS.lock().await;
            state.temp[i as usize] = temp_data as i32;
            drop(state);
        }

        Timer::after_millis(5000).await;
    }
}
