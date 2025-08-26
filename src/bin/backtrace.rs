use core::pin::pin;
use esp_println::println;
use esp_backtrace::arch;
use esp_backtrace::arch::backtrace;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::ledc::{LSGlobalClkSource, Ledc};
use esp_hal::timer::systimer::SystemTimer;
use static_cell::StaticCell;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::channel::ChannelIFace;

const RESET: &str = "\u{001B}[0m";
const RED: &str = "\u{001B}[31m";


//Really crude delay function dont judge plz
fn _ll_delay(mut delay: u64) {

    while delay != 0 {
        unsafe { core::arch::asm!("nop"); }
        delay -= 1;
    }

}

fn _ll_force_gpio(pin: u32) {
    const GPIO_BASE: usize = 0x6000_4000;
    const IO_MUX_BASE: usize = 0x6000_9000;
    const GPIO_FUNC_OUT_SEL_BASE: usize = 0x6000_9554;
    const GPIO_FUNC_IN_SEL_BASE: usize = 0x6000_9154;
    const GPIO_ENABLE_W1TS: *mut u32 = (GPIO_BASE + 0x24) as *mut u32;

    // Map of supported pins to IO_MUX register offset
    let io_mux_offset = match pin {
        10 => 0x2C,
        20 => 0x54,
        21 => 0x58,
        _ => return, // Unsupported
    };

    unsafe {
        let mask = 1 << pin;

        // Step 1: Disconnect GPIO matrix input (detach peripheral from GPIO input)
        let in_sel = (GPIO_FUNC_IN_SEL_BASE + (pin * 4) as usize) as *mut u32;
        core::ptr::write_volatile(in_sel, 0x80);

        // Step 2: Disconnect GPIO matrix output (detach peripheral from GPIO output)
        let out_sel = (GPIO_FUNC_OUT_SEL_BASE + (pin * 4) as usize) as *mut u32;
        core::ptr::write_volatile(out_sel, 256);

        // Step 3: Configure IO_MUX for FUNC0 (GPIO), drive strength 3, OE enabled
        let io_mux = (IO_MUX_BASE + io_mux_offset) as *mut u32;
        let mut reg = core::ptr::read_volatile(io_mux);
        reg &= !(0b111 | (3 << 7) | (1 << 9)); // Clear FUNC_SEL, drive, OE_EN
        reg |= 0b000;        // FUNC0 (GPIO)
        reg |= 3 << 7;       // Max drive strength
        reg |= 1 << 9;       // Enable output
        core::ptr::write_volatile(io_mux, reg);

        // Step 4: Enable GPIO output driver
        core::ptr::write_volatile(GPIO_ENABLE_W1TS, mask);
    }
}




fn _ll_led_clear(pin_number: u32) {
    const GPIO_BASE: usize = 0x6000_4000;
    const GPIO_OUT_W1TC: *mut u32 = (GPIO_BASE + 0x0C) as *mut u32;
    unsafe {
        core::ptr::write_volatile(GPIO_OUT_W1TC, 1 << pin_number);
    }
}

fn _ll_led_set(pin_number: u32) {
    const GPIO_BASE: usize = 0x6000_4000;
    const GPIO_OUT_W1TS: *mut u32 = (GPIO_BASE + 0x08) as *mut u32;
    unsafe {
        core::ptr::write_volatile(GPIO_OUT_W1TS, 1 << pin_number);
    }
}


#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {

    println!("{}", RED);

    println!("");
    println!("Custom panic handler");
    println!("====================== PANIC ======================");

    println!("{}", info);

    println!("");
    println!("Backtrace:");
    println!("");

    let backtrace = esp_backtrace::Backtrace::capture();
    #[cfg(target_arch = "riscv32")]
    if backtrace.frames().is_empty() {
        println!(
            "No backtrace available - make sure to force frame-pointers. (see https://crates.io/crates/esp-backtrace)"
        );
    }
    for frame in backtrace.frames() {
        println!("0x{:x}", frame.program_counter());
    }

    println!("{}", RESET);

    println!("Done:");

    _ll_force_gpio(10);

    loop {

        _ll_led_set(10);

        _ll_delay(10_000_000);

        _ll_led_clear(10);

        _ll_delay(10_000_000);



    }
}

