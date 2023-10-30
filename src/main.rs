#![no_std] // don't link the Rust standard library
#![no_main] // disable all Rust-level entry points

use core::arch::global_asm;
use core::{arch::asm, panic::PanicInfo};

// #define MTIME_LOW       (*((volatile uint32_t *)0x40000008))
// #define MTIME_HIGH      (*((volatile uint32_t *)0x4000000C))
// #define MTIMECMP_LOW    (*((volatile uint32_t *)0x40000010))
// #define MTIMECMP_HIGH   (*((volatile uint32_t *)0x40000014))
// #define CONTROLLER      (*((volatile uint32_t *)0x40000018))

extern "C" {
    static _erodata: *mut u8;
    static _data: *mut u8;
    static _edata: *mut u8;
    static _sdata: *mut u8;
    static _esdata: *mut u8;
    static _bss: *mut u8;
    static _ebss: *mut u8;
}

// const MTIME_LOW_ADDR     : *mut u32 = 0x40000008 as *mut u32;
// const MTIME_HIGH_ADDR    : *mut u32 = 0x4000000C as *mut u32;
const MTIMECMP_LOW_ADDR: *mut u32 = 0x40000010 as *mut u32;
const MTIMECMP_HIGH_ADDR: *mut u32 = 0x40000014 as *mut u32;
const CONTROLLER_ADDR: *mut u32 = 0x40000018 as *mut u32;

#[allow(non_camel_case_types)]
#[allow(non_upper_case_globals)]
#[no_mangle]
pub static mut controller_status: u32 = 0;

global_asm!(
    r#".section .text, "ax"
.global _interrupt_handler
_interrupt_handler:
    addi	sp,sp,-40
    sw	    ra,36(sp)
    sw	    t0,32(sp)
    sw	    t1,28(sp)
    sw	    t2,24(sp)
    sw	    a0,20(sp)
    sw	    a1,16(sp)
    sw	    a2,12(sp)
    sw	    a3,8(sp)
    sw	    a4,4(sp)
    sw	    a5,0(sp)
    call    c_interrupt_handler
    lw	    ra,36(sp)
    lw	    t0,32(sp)
    lw	    t1,28(sp)
    lw	    t2,24(sp)
    lw	    a0,20(sp)
    lw	    a1,16(sp)
    lw	    a2,12(sp)
    lw	    a3,8(sp)
    lw	    a4,4(sp)
    lw	    a5,0(sp)
    addi    sp,sp,40
    mret
"#
);

#[no_mangle]
unsafe fn rust_func() {
    let addr_1 = 4 as *mut u32;
    *(8 as *mut u32) = *addr_1;
}

#[link_section = ".init"]
#[no_mangle]
pub unsafe extern "C" fn _start() -> ! {
    // this function is the entry point, since the linker looks for a function
    // named `_start` by default

    asm!(
        r#"
        .option push
        .option norelax
        la gp, __global_pointer$
        .option pop
        la sp, __stack_top
        add s0, sp, zero
        la  a5, _interrupt_handler
        csrw mtvec, a5
        jal ra, init
        nop
        jal zero, main
        "#
    );
    rust_func();

    loop {}
}

#[no_mangle] // don't mangle the name of this function
pub unsafe extern "C" fn init() {
    let mut source = _erodata;
    let mut base = if _data < _sdata { _data } else { _sdata };
    let mut end = if _edata > _esdata { _edata } else { _esdata };

    while base < end {
        *base = *source;
        base = base.wrapping_add(1);
        source = source.wrapping_add(1);
    }
    base = _bss;
    end = _ebss;
    while base < end {
        *base = 0;
        base = base.wrapping_add(1);
    }

    // csr_write_mie(0x888);       // Enable all interrupt soruces
    // asm!("csrw mie, 0x888");
    asm!(
        "li {tmp}, 0x888",
        "csrw mie, {tmp}",
        tmp = out(reg) _
    );

    // csr_enable_interrupts();    // Global interrupt enable
    asm!("csrsi mstatus, 0x8");

    *MTIMECMP_LOW_ADDR = 1;
    *MTIMECMP_HIGH_ADDR = 0;
}

#[no_mangle] // don't mangle the name of this function
pub unsafe extern "C" fn c_interrupt_handler() {
    // // this function is the entry point, since the linker looks for a function
    // // named `_start` by default

    // let mut new_compare: u64 = (*MTIMECMP_HIGH_ADDR as u64) << 32 | *MTIMECMP_LOW_ADDR as u64;
    // new_compare += 100;
    // *MTIMECMP_HIGH_ADDR = (new_compare >> 32) as u32;
    // *MTIMECMP_LOW_ADDR = (new_compare) as u32;
    // controller_status = *CONTROLLER_ADDR;
}

fn printf(text: &[u8]){
    let video_memory = (0x50000000 + 0xF4800) as *mut u8;

    for (i, &byte) in text.iter().enumerate() {
        unsafe {
            *video_memory.offset(i as isize) = byte;
        }
    }
}

#[no_mangle] // don't mangle the name of this function
pub unsafe extern "C" fn main() {
    //set mode to graphics
    *(0x500F6780 as *mut u32) = 0x0;
    printf(b"Hello World!");

    loop{} 
    //set mode to graphics
    *(0x500F6780 as *mut u32) = 0x1;

    //set background 0 position
    // *(0x500F5100 as *mut u32) = 0x0 
    // | ((0b10_1110_0000) << 12)
    // | ((0b10_0000_0000) << 2)
    // | 1 << 29;
    *(0x500F5100 as *mut u32) = 0x0 
    | ((288) << 12)
    | ((512) << 2)
    | 1 << 29;

    //set background0 palette
    let background0_palette = 0x500FC000 as *mut u32;
    for i in 0..256 {
        *background0_palette.offset(i as isize) = 0xAAAA_AAAA as u32;
    }
    let background_image_data = (0x50000000) as *mut u8; 
    for i in 0..0x24000 {
        *background_image_data.offset(i as isize) = 1;
    }
}

// This function is called on panic.
#[panic_handler]
#[no_mangle]
pub unsafe fn panic(_info: &PanicInfo) -> ! {
    *(0x500F6780 as *mut u32) = 0x0;
    printf(b"System panic :(");
    loop {}
}
