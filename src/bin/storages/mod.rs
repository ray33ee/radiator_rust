mod interface;
mod config;

pub(crate) use interface::SdInterface;



/*


use serde::{Serialize, Deserialize};
use esp_hal::rom::spiflash::{
    esp_rom_spiflash_erase_sector,
    esp_rom_spiflash_write,
    esp_rom_spiflash_read,
    ESP_ROM_SPIFLASH_RESULT_OK,
    ESP_ROM_SPIFLASH_RESULT_ERR,
    ESP_ROM_SPIFLASH_RESULT_TIMEOUT,
};
use smoltcp::wire::IpAddress;

const SUMMER_PAGE: u32 = 1018;
const WINTER_PAGE: u32 = 1019;
const NIGHT_PAGE: u32 = 1020;
const PROPERTIES_PAGE: u32 = 1021;
const POSITION_PAGE: u32 = 1022;
const PANIC_PAGE: u32 = 1023;

enum Error {
    Err,
    Timeout,
}

fn code_to_result(code: i32) -> Result<(), Error> {
    if code == ESP_ROM_SPIFLASH_RESULT_OK {
        Ok(())
    } else if code == ESP_ROM_SPIFLASH_RESULT_ERR {
        Err(Error::Err)
    } else if code == ESP_ROM_SPIFLASH_RESULT_TIMEOUT {
        Err(Error::Timeout)
    } else {
        unreachable!()
    }
}

unsafe fn erase_wrapper(sector_number: u32) -> Result<(), Error> {

    let result = esp_rom_spiflash_erase_sector(sector_number);

    code_to_result(result)
}

unsafe fn write_wrapper(dest_addr: u32, data: *const u32, len: u32,) -> Result<(), Error> {

    let result = esp_rom_spiflash_write(dest_addr, data, len);

    code_to_result(result)
}

unsafe fn read_wrapper(src_addr: u32, data: *const u32, len: u32,) -> Result<(), Error> {

    let result = esp_rom_spiflash_read(src_addr, data, len);

    code_to_result(result)
}

#[derive(Serialize, Deserialize)]
enum Schedule {
    Summer,
    Winter,
}

#[derive(Serialize, Deserialize)]
struct Properties {
    magic: u32, //If this value if 0xffffffff or serde fails, then the flash is empty

    ssid: heapless::String<64>,
    password: heapless::String<64>,
    static_ip: Option<u32>,
    dns_ip: Option<u32>,

    max: u32,

    variant: Schedule,

    key: heapless::Vec<u8, 128>,
    iv: heapless::Vec<u8, 16>,

    temperature: f32,

}

struct Storage {
    properties: Option<Properties>,
}

impl Storage {
    pub fn new() -> Self {
        const PAGE_SIZE: usize = 4096;
        const FLASH_ADDR: u32 = PROPERTIES_PAGE * PAGE_SIZE as u32;

        let mut buffer = [0u8; PAGE_SIZE];

        let result = unsafe {
            read_wrapper(
                FLASH_ADDR,
                buffer.as_mut_ptr() as *mut u32,
                PAGE_SIZE as u32,
            )
        };

        let properties = match result {
            Ok(()) => {
                // Check for erased flash
                let magic = u32::from_le_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]);
                if magic == 0xFFFF_FFFF {
                    None
                } else {
                    postcard::from_bytes::<Properties>(&buffer).ok()
                }
            }
            Err(_) => None,
        };

        Self { properties }
    }
}
*/