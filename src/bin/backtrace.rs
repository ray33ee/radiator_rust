use core::fmt::Display;
use esp_println::println;
use esp_hal::ledc::channel::{ChannelHW};
use crate::{BLUE_REF, GREEN_REF, RED_REF};

const RESET: &str = "\u{001B}[0m";
const RED: &str = "\u{001B}[31m";

#[derive(Debug)]
pub(crate) struct TrapFrame {
    /// Return address, stores the address to return to after a function call or
    /// interrupt.
    pub ra: usize,
    /// Temporary register t0, used for intermediate values.
    pub t0: usize,
    /// Temporary register t1, used for intermediate values.
    pub t1: usize,
    /// Temporary register t2, used for intermediate values.
    pub t2: usize,
    /// Temporary register t3, used for intermediate values.
    pub t3: usize,
    /// Temporary register t4, used for intermediate values.
    pub t4: usize,
    /// Temporary register t5, used for intermediate values.
    pub t5: usize,
    /// Temporary register t6, used for intermediate values.
    pub t6: usize,
    /// Argument register a0, typically used to pass the first argument to a
    /// function.
    pub a0: usize,
    /// Argument register a1, typically used to pass the second argument to a
    /// function.
    pub a1: usize,
    /// Argument register a2, typically used to pass the third argument to a
    /// function.
    pub a2: usize,
    /// Argument register a3, typically used to pass the fourth argument to a
    /// function.
    pub a3: usize,
    /// Argument register a4, typically used to pass the fifth argument to a
    /// function.
    pub a4: usize,
    /// Argument register a5, typically used to pass the sixth argument to a
    /// function.
    pub a5: usize,
    /// Argument register a6, typically used to pass the seventh argument to a
    /// function.
    pub a6: usize,
    /// Argument register a7, typically used to pass the eighth argument to a
    /// function.
    pub a7: usize,
    /// Saved register s0, used to hold values across function calls.
    pub s0: usize,
    /// Saved register s1, used to hold values across function calls.
    pub s1: usize,
    /// Saved register s2, used to hold values across function calls.
    pub s2: usize,
    /// Saved register s3, used to hold values across function calls.
    pub s3: usize,
    /// Saved register s4, used to hold values across function calls.
    pub s4: usize,
    /// Saved register s5, used to hold values across function calls.
    pub s5: usize,
    /// Saved register s6, used to hold values across function calls.
    pub s6: usize,
    /// Saved register s7, used to hold values across function calls.
    pub s7: usize,
    /// Saved register s8, used to hold values across function calls.
    pub s8: usize,
    /// Saved register s9, used to hold values across function calls.
    pub s9: usize,
    /// Saved register s10, used to hold values across function calls.
    pub s10: usize,
    /// Saved register s11, used to hold values across function calls.
    pub s11: usize,
    /// Global pointer register, holds the address of the global data area.
    pub gp: usize,
    /// Thread pointer register, holds the address of the thread-local storage
    /// area.
    pub tp: usize,
    /// Stack pointer register, holds the address of the top of the stack.
    pub sp: usize,
    /// Program counter, stores the address of the next instruction to be
    /// executed.
    pub pc: usize,
    /// Machine status register, holds the current status of the processor,
    /// including interrupt enable bits and privilege mode.
    pub mstatus: usize,
    /// Machine cause register, contains the reason for the trap (e.g.,
    /// exception or interrupt number).
    pub mcause: usize,
    /// Machine trap value register, contains additional information about the
    /// trap (e.g., faulting address).
    pub mtval: usize,
}


//Really crude delay function dont judge plz
fn _ll_delay(mut delay: u64) {

    while delay != 0 {
        unsafe { core::arch::asm!("nop"); }
        delay -= 1;
    }

}

fn _ll_short_delay() {
    _ll_delay(10_000_000);
}

fn _ll_clear_gb() {
    unsafe {
        if GREEN_REF.is_some() {
            crate::GREEN_REF.as_mut().unwrap().set_duty_hw(256); //Safe
        }

        if BLUE_REF.is_some() {
            crate::BLUE_REF.as_mut().unwrap().set_duty_hw(256); //Safe
        }
    }
}

fn _ll_flash_r() {
    unsafe {
        if RED_REF.is_some() {
            crate::RED_REF.as_mut().unwrap().set_duty_hw(0); //Safe
        }

        _ll_short_delay();

        if RED_REF.is_some() {
            crate::RED_REF.as_mut().unwrap().set_duty_hw(256); //Safe
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



#[cfg(target_arch = "riscv32")]
#[unsafe(export_name = "ExceptionHandler")]
fn exception_handler(context: &TrapFrame) -> ! {

    let mepc = context.pc;
    let code = context.mcause & 0xff;
    let mtval = context.mtval;

    let message = match code {
        0 => "Instruction address misaligned",
        1 => "Instruction access fault",
        2 => "Illegal instruction",
        3 => "Breakpoint",
        4 => "Load address misaligned",
        5 => "Load access fault",
        6 => "Store/AMO address misaligned",
        7 => "Store/AMO access fault",
        8 => "Environment call from U-mode",
        9 => "Environment call from S-mode",
        10 => "Reserved",
        11 => "Environment call from M-mode",
        12 => "Instruction page fault",
        13 => "Load page fault",
        14 => "Stack overflow",
        15 => "Store/AMO page fault",
        _ => "UNKNOWN",
    };

    if code == 14 {
        println!();
        println!(
            "Stack overflow detected at 0x{:x} called by 0x{:x}",
            mepc, context.ra
        );
        println!();
    } else {

        println!(
            "Exception '{}' mepc=0x{:08x}, mtval=0x{:08x}",
            message, mepc, mtval
        );

        println!("{:?}", context);

    }

    _ll_clear_gb();

    loop {

        _ll_flash_r();

        println!(
            "Exception '{}' mepc=0x{:08x}, mtval=0x{:08x}",
            message, mepc, mtval
        );

        println!("{:?}", context);

        println!();
        println!();

    }
}

