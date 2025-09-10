

use esp_hal::rom::spiflash::{
    esp_rom_spiflash_erase_sector,
    esp_rom_spiflash_write,
    esp_rom_spiflash_read,
};
use serde::{Serialize};
use serde::de::DeserializeOwned;
use crate::flashstore::FlashStorage;

const PAGE_SIZE: u32 = 4096;


pub(crate) const SCHEDULE_ADDRESS: u32 = 995;
pub(crate) const MOTOR_ADDRESS: u32 = 994;
pub(crate) const THERMO_ADDRESS: u32 = 993;
pub(crate) const WIFI_ADDRESS: u32 = 992;
const POSITION_PAGE: u32 = 1022; //


const POSITION_LOCATION: u32 = POSITION_PAGE * PAGE_SIZE;

pub(crate) fn lock() {
    //Erase the 'Positon/Lock' sector of flash
    critical_section::with(|_| {

        unsafe {
            let _ = esp_rom_spiflash_erase_sector(POSITION_PAGE);
        }
    });
}

pub(crate) fn unlock_and_set_pos(position: u32) {
    //Write the position to the 'Positon/Lock' sector of flash at the first 4 bytes
    critical_section::with(|_| {
        unsafe {
            println!("Start");

            let a = esp_rom_spiflash_erase_sector(POSITION_PAGE);

            println!("Between");

            let b = esp_rom_spiflash_write(POSITION_LOCATION, &position as *const u32, 4);

            use esp_println::println;
            println!("A: {}, B: {}", a, b);
        }
    });

}

pub(crate) fn is_locked() -> bool {
    //If its 0xffffffff then the device is locked.
    // If position is greater than ABSOLUTE_MAX_POSITION then it corrupt, treat this as locked too
    let position = get_position();
    position > crate::motor::ABSOLUTE_MAX_POSITION
}

pub(crate) fn get_position() -> u32 {
    //Read the first u32
    let mut position = 0xFFFFFFFF;
    critical_section::with(|_| {
        unsafe {
            let _ = esp_rom_spiflash_read(POSITION_LOCATION, &mut position as *mut u32, 4);
        }
    });
    position
}




pub(crate) fn save_to_page<T: Serialize>(page_number: u32, value: T) {
    let serialized = serde_json::to_vec(&value).unwrap();

    let length = serialized.len();

    let mut flash_storage = FlashStorage::new();

    flash_storage.write(page_number * PAGE_SIZE, length.to_be_bytes().as_ref()).unwrap();

    flash_storage.write(page_number * PAGE_SIZE + 4, serialized.as_ref()).unwrap();
}

pub(crate) fn load_from_page<T: DeserializeOwned>(page_number: u32) -> Option<T> {
    let mut len_bytes = [0u8; 4];

    let mut flash_storage = FlashStorage::new();

    flash_storage.read(page_number * PAGE_SIZE, len_bytes.as_mut_slice()).unwrap();

    let len = usize::from_be_bytes(len_bytes);

    if len > PAGE_SIZE as usize {
        return None;
    }

    let mut buff = [0u8; 4092];

    flash_storage.read(page_number * PAGE_SIZE + 4, & mut buff[0..len]).unwrap();

    serde_json::from_slice::<T>(&buff[..len]).ok()

}