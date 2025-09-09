use esp_println::println;
use esp_hal::ledc::channel::{ChannelHW};
use crate::{BLUE_REF, GREEN_REF, RED_REF};

pub(crate) const RESET: &str = "\u{001B}[0m";
const RED: &str = "\u{001B}[31m";

use esp_backtrace::arch;
use esp_hal::delay::Delay;

fn _ll_led_on(pin_number: u32) {
    unsafe {
        let gpio = &*esp_hal::peripherals::GPIO::PTR;

        // Pins >= 32 use OUT1_*; bit index is (pin - 32).
        // 37 -> bit 5, 39 -> bit 7.
        let mask = 1u32 << (pin_number - 32);

        gpio.out1_w1tc().write(|w| w.bits(mask));

    }
}

fn _ll_heartbeat_off() {
    unsafe {
        let gpio = &*esp_hal::peripherals::GPIO::PTR;

        // Pins 32–39 use OUT1_*; bit index = pin - 32.
        const MASK: u32 = 1u32 << 3;

        // Drive HIGH (LED off for active-low).
        gpio.out1_w1ts().write(|w| w.bits(MASK));
    }
}

//Really crude delay function dont judge plz
fn _ll_delay(delay: u32) {

    Delay::new().delay_millis(delay);

}

fn _ll_short_delay() {
    _ll_delay(500);
}

fn _ll_clear_gb() {
    unsafe {
        if GREEN_REF.is_some() {
            crate::GREEN_REF.as_mut().unwrap().set_duty_hw(4096); //Safe
        }

        if BLUE_REF.is_some() {
            crate::BLUE_REF.as_mut().unwrap().set_duty_hw(4096); //Safe
        }

        _ll_heartbeat_off();
    }
}

fn _ll_flash_r() {
    unsafe {
        if RED_REF.is_some() {
            crate::RED_REF.as_mut().unwrap().set_duty_hw(0); //Safe
        }

        _ll_short_delay();

        if RED_REF.is_some() {
            crate::RED_REF.as_mut().unwrap().set_duty_hw(4096); //Safe
        }

        _ll_short_delay();
    }
}


#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo<'_>) -> ! {

    println!("{}", RED);

    println!();
    println!("Custom panic handler");
    println!("====================== PANIC ======================");

    println!("{}", info);

    println!();
    println!("Backtrace:");
    println!();

    _ll_led_on(37); //Turn on the panic LED

    let backtrace = esp_backtrace::Backtrace::capture();


    for frame in backtrace.frames() {
        println!("0x{:x}", frame.program_counter());
    }

    _ll_clear_gb();


    loop {


        for _ in 0..10 {

            _ll_flash_r();
        }


        println!("{}", info);

        println!();
        println!();


    }
}

#[cfg(target_arch = "xtensa")]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".rwtext")]
unsafe fn __user_exception(cause: arch::ExceptionCause, context: arch::Context) {

    _ll_clear_gb();

    _ll_led_on(39); //Turn on the exception LED

    println!("{}", RED);

    println!("\n\nException occurred '{}'", cause);
    println!("{:?}\n\n", context);

    loop {


        for _ in 0..10 {

            _ll_flash_r();
        }


        println!("Exception occurred '{}'", cause);
        println!("{:?}", context);

        println!();
        println!();


    }
}
