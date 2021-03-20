use core::mem;
use core::ffi::c_void;

use esp_idf_sys::*;

#[derive(Debug, Clone)]
pub struct Frame {
    pc: u32,
    sp: u32
}

// SAFETY: Can be used from any thread.
unsafe impl Send for Frame {}
unsafe impl Sync for Frame {}

impl Frame {
    pub fn ip(&self) -> *mut c_void {
        esp_cpu_process_stack_pc(self.pc) as *mut _
    }

    pub fn sp(&self) -> *mut c_void {
        self.sp as *mut _
    }

    pub fn symbol_address(&self) -> *mut c_void {
        self.ip()
    }

    pub fn module_base_address(&self) -> Option<*mut c_void> {
        None
    }
}

pub unsafe fn trace<F: FnMut(&super::Frame) -> bool>(cb: F) {
    trace_depth(u32::MAX, cb);
}

pub unsafe fn trace_depth<F: FnMut(&super::Frame) -> bool>(mut depth: u32, mut cb: F) {
    let mut stk_frame_buf = [0; mem::size_of::<esp_backtrace_frame_t>()];
    let stk_frame: &mut esp_backtrace_frame_t = mem::transmute(&mut stk_frame_buf);

    esp_backtrace_get_start(&mut (stk_frame.pc) as *mut _, &mut (stk_frame.sp) as *mut _, &mut (stk_frame.next_pc) as *mut _);

    let mut corrupted = !esp_stack_ptr_is_sane(stk_frame.sp)
        || !esp_ptr_executable(esp_cpu_process_stack_pc(stk_frame.pc));

    cb(&super::Frame { inner: Frame {pc: stk_frame.pc, sp: stk_frame.sp} });

    while !corrupted && depth > 0 && stk_frame.next_pc != 0 {
        if !esp_backtrace_get_next_frame(stk_frame as *mut _) {
            corrupted = true;
        }

        cb(&super::Frame { inner: Frame {pc: stk_frame.pc, sp: stk_frame.sp} });

        depth -= 1;
    }
}

// esp_cpu_process_stack_pc() is - unfortunately - an inline function in ESP-IDF
// Hence we had to copy it here
fn esp_cpu_process_stack_pc(mut pc: u32) -> u32 {
    if pc & 0x80000000 != 0 {
        // Top two bits of a0 (return address) specify window increment. Overwrite to map to address space.
        pc = (pc & 0x3fffffff) | 0x40000000;
    }

    // Minus 3 to get PC of previous instruction (i.e. instruction executed before return address)
    pc - 3
}

// Ditto
fn esp_stack_ptr_is_sane(sp: u32) -> bool {
    //Check if stack ptr is in between SOC_DRAM_LOW and SOC_DRAM_HIGH, and 16 byte aligned.
    !(sp < SOC_DRAM_LOW + 0x10 || sp > SOC_DRAM_HIGH - 0x10 || ((sp & 0xF) != 0))
}

// Ditto
fn esp_ptr_executable(ip: u32) -> bool {
    (ip >= SOC_IROM_LOW && ip < SOC_IROM_HIGH)
        || (ip >= SOC_IRAM_LOW && ip < SOC_IRAM_HIGH)
        || (ip >= SOC_IROM_MASK_LOW && ip < SOC_IROM_MASK_HIGH)
// TODO
// #if defined(SOC_CACHE_APP_LOW) && defined(CONFIG_FREERTOS_UNICORE)
//         || (ip >= SOC_CACHE_APP_LOW && ip < SOC_CACHE_APP_HIGH)
// #endif
        || (ip >= SOC_RTC_IRAM_LOW && ip < SOC_RTC_IRAM_HIGH)
}
