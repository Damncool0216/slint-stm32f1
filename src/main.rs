#![no_std]
#![no_main]
// #![cfg_attr(not(feature = "simulator"), no_main)]

extern crate alloc;
use alloc::rc::Rc;
use core::panic::PanicInfo;
use embedded_graphics_core::geometry::OriginDimensions;
use embedded_graphics_core::pixelcolor::BinaryColor;
use embedded_hal::digital::v2::InputPin;

use hal::i2c::{BlockingI2c, DutyCycle, Mode};
use hal::prelude::*;
use hal::time::Hertz;

use slint::platform::software_renderer::{
    MinimalSoftwareWindow, PremultipliedRgbaColor, RepaintBufferType, TargetPixel,
};
use slint::platform::{Key, Platform, WindowAdapter, WindowEvent};
use slint::PhysicalSize;

use stm32f1xx_hal as hal;

use cortex_m_rt::entry;

struct MyPlatform {
    window: Rc<MinimalSoftwareWindow>,
    timer: hal::time::Instant,
    freq: Hertz,
}

impl Platform for MyPlatform {
    fn create_window_adapter(&self) -> Result<Rc<dyn WindowAdapter>, slint::PlatformError> {
        // Since on MCUs, there can be only one window, just return a clone of self.window.
        // We'll also use the same window in the event loop.
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis((self.timer.elapsed() / self.freq.to_kHz()) as u64)
    }
}

slint::include_modules!();

#[entry]
fn main() -> ! {
    #[macro_export]
    macro_rules! rprintln {
        ($($arg:tt)*) => {
            #[cfg(feature = "rtt")]
            {
                rtt_target::rprintln!($($arg)*);
            }
        };
    }
    #[cfg(feature = "rtt")]
    rtt_target::rtt_init_print!();

    let dp = hal::pac::Peripherals::take().unwrap();
    let cp = hal::pac::CorePeripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(72.MHz())
        .freeze(&mut flash.acr);

    let mut gpiob = dp.GPIOB.split();
    let mut gpioe = dp.GPIOE.split();


    let btn_0 = gpioe.pe4.into_pull_up_input(&mut gpioe.crl);
    let btn_1 = gpioe.pe3.into_pull_up_input(&mut gpioe.crl);
    // DWT::enable_cycle_counter();
    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );
    use ssd1306::prelude::*;
    let interface = ssd1306::I2CDisplayInterface::new(i2c);
    let mut disp = ssd1306::Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    disp.init().unwrap();
    disp.flush().unwrap();
    disp.clear();

    // -------- Setup Allocator --------
    const HEAP_SIZE: usize = 30 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    #[global_allocator]
    static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();
    unsafe {
        ALLOCATOR.init(
            &mut HEAP as *const u8 as usize,
            core::mem::size_of_val(&HEAP),
        )
    }

    let window = MinimalSoftwareWindow::new(RepaintBufferType::ReusedBuffer);
    let timer = hal::time::MonoTimer::new(cp.DWT, cp.DCB, clocks);
    slint::platform::set_platform(alloc::boxed::Box::new(MyPlatform {
        window: window.clone(),
        timer: timer.now(),
        freq: timer.frequency(),
    }))
    .unwrap();

    let ui = MainWindow::new().unwrap();

    let s = disp.size();
    ui.window().set_size(PhysicalSize::new(s.width, s.height));
    ui.show().unwrap();

    let mut line = [GrayPixel(0); 128];

    let mut btns = [
        (&btn_0 as &dyn InputPin<Error = _>, Key::LeftArrow, false),
        (&btn_1 as &dyn InputPin<Error = _>, Key::RightArrow, false),
    ];

    rprintln!("init finish");
    loop {
        rprintln!("loop start");
        slint::platform::update_timers_and_animations();
        window.draw_if_needed(|renderer| {
            use embedded_graphics_core::prelude::*;
            struct DisplayWrapper<'a, T>(&'a mut T, &'a mut [GrayPixel]);
            impl<T: DrawTarget<Color = BinaryColor>>
                slint::platform::software_renderer::LineBufferProvider for DisplayWrapper<'_, T>
            {
                type TargetPixel = GrayPixel;
                fn process_line(
                    &mut self,
                    line: usize,
                    range: core::ops::Range<usize>,
                    render_fn: impl FnOnce(&mut [Self::TargetPixel]),
                ) {
                    let rect = embedded_graphics_core::primitives::Rectangle::new(
                        Point::new(range.start as _, line as _),
                        Size::new(range.len() as _, 1),
                    );
                    render_fn(&mut self.1[range.clone()]);
                    self.0
                        .fill_contiguous(
                            &rect,
                            self.1[range.clone()].iter().map(|src| {
                                if src.0 > 0x88 {
                                    BinaryColor::On
                                } else {
                                    BinaryColor::Off
                                }
                            }),
                        )
                        .map_err(drop)
                        .unwrap();
                }
            }

            renderer.render_by_line(DisplayWrapper(&mut disp, line.as_mut_slice()));
            disp.flush().unwrap();
        });
        for (btn, key, pressed) in &mut btns {
            let p = btn.is_high().unwrap();
            if p && !*pressed {
                window.window().dispatch_event(WindowEvent::KeyPressed {
                    text: (*key).into(),
                })
            } else if !p && *pressed {
                window.window().dispatch_event(WindowEvent::KeyReleased {
                    text: (*key).into(),
                })
            };
            *pressed = p;
        }

        rprintln!("loop end");
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct GrayPixel(pub u8);

impl TargetPixel for GrayPixel {
    fn blend(&mut self, color: PremultipliedRgbaColor) {
        let a = (u8::MAX - color.alpha) as u16;

        let c = (color.red as u16 + color.blue as u16 + color.green as u16) / 3;

        self.0 = (((c << 8) + self.0 as u16 * a) >> 8) as u8;
    }

    fn from_rgb(r: u8, g: u8, b: u8) -> Self {
        Self(((r as u16 + g as u16 + b as u16) / 3) as u8)
    }
}

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    rprintln!("{}", _info);
    loop {} // You might need a compiler fence in here.
}
