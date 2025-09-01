use core::mem::MaybeUninit;
use core::ops::{Deref, DerefMut};
use esp_hal::rom::spiflash::{
    esp_rom_spiflash_read,
    esp_rom_spiflash_write,
    esp_rom_spiflash_unlock,
    esp_rom_spiflash_erase_sector,
};

#[repr(C, align(4))]
pub(crate) struct FlashSectorBuffer {
    // NOTE: Ensure that no unaligned fields are added above `data` to maintain its required alignment
    data: [u8; FlashStorage::SECTOR_SIZE as usize],
}

impl Deref for FlashSectorBuffer {
    type Target = [u8; FlashStorage::SECTOR_SIZE as usize];

    fn deref(&self) -> &Self::Target {
        &self.data
    }
}

impl DerefMut for FlashSectorBuffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.data
    }
}

#[derive(Debug)]
#[non_exhaustive]
pub(crate) enum FlashStorageError {
    IoError,
    IoTimeout,
    CantUnlock,
    OutOfBounds,
    Other,
}

#[inline(always)]
pub(crate) fn check_rc(rc: i32) -> Result<(), FlashStorageError> {
    match rc {
        0 => Ok(()),
        1 => Err(FlashStorageError::IoError),
        2 => Err(FlashStorageError::IoTimeout),
        _ => Err(FlashStorageError::Other),
    }
}

#[derive(Debug)]
pub(crate) struct FlashStorage {
    pub(crate) capacity: usize,
    unlocked: bool,
}

impl Default for FlashStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl FlashStorage {
    pub(crate) const WORD_SIZE: u32 = 4;
    pub(crate) const SECTOR_SIZE: u32 = 4096;

    pub(crate) fn new() -> FlashStorage {
        let mut storage = FlashStorage {
            capacity: 0,
            unlocked: false,
        };

        storage.capacity = 4 * 1024 * 1024;

        storage
    }


    #[inline(always)]
    pub(crate) fn check_bounds(&self, offset: u32, length: usize) -> Result<(), FlashStorageError> {
        let offset = offset as usize;
        if length > self.capacity || offset > self.capacity - length {
            return Err(FlashStorageError::OutOfBounds);
        }
        Ok(())
    }

    #[allow(clippy::all)]
    #[inline(never)]
    #[link_section = ".rwtext"]
    pub(crate) fn internal_read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), FlashStorageError> {
        check_rc(unsafe { esp_rom_spiflash_read(
            offset,
            bytes.as_ptr() as *mut u32,
            bytes.len() as u32,
        ) } )
    }

    #[inline(always)]
    fn unlock_once(&mut self) -> Result<(), FlashStorageError> {
        if !self.unlocked {
            if unsafe { esp_rom_spiflash_unlock() } != 0 {
                return Err(FlashStorageError::CantUnlock);
            }
            self.unlocked = true;
        }
        Ok(())
    }

    #[inline(never)]
    #[link_section = ".rwtext"]
    pub(crate) fn internal_erase(&mut self, sector: u32) -> Result<(), FlashStorageError> {
        self.unlock_once()?;

        check_rc(unsafe { esp_rom_spiflash_erase_sector(sector) } )
    }

    #[inline(never)]
    #[link_section = ".rwtext"]
    pub(crate) fn internal_write(
        &mut self,
        offset: u32,
        bytes: &[u8],
    ) -> Result<(), FlashStorageError> {
        self.unlock_once()?;

        check_rc(unsafe { esp_rom_spiflash_write(
            offset,
            bytes.as_ptr() as *const u32,
            bytes.len() as u32,
        ) } )
    }

    pub(crate) fn read(&mut self, offset: u32, mut bytes: &mut [u8]) -> Result<(), FlashStorageError> {
        self.check_bounds(offset, bytes.len())?;

        let mut data_offset = offset % Self::WORD_SIZE;
        let mut aligned_offset = offset - data_offset;

        // Bypass clearing sector buffer for performance reasons
        let mut sector_data = MaybeUninit::<FlashSectorBuffer>::uninit();
        let sector_data = unsafe { sector_data.assume_init_mut() };

        while !bytes.is_empty() {
            let len = bytes.len().min((Self::SECTOR_SIZE - data_offset) as _);

            let aligned_end = (data_offset as usize + len + (Self::WORD_SIZE - 1) as usize)
                & !(Self::WORD_SIZE - 1) as usize;

            // Read only needed data words
            self.internal_read(aligned_offset, &mut sector_data[..aligned_end])?;

            bytes[..len].copy_from_slice(&sector_data[data_offset as usize..][..len]);

            aligned_offset += Self::SECTOR_SIZE;
            data_offset = 0;
            bytes = &mut bytes[len..];
        }

        Ok(())
    }


    pub(crate) fn write(&mut self, offset: u32, mut bytes: &[u8]) -> Result<(), FlashStorageError> {
        self.check_bounds(offset, bytes.len())?;

        let mut data_offset = offset % Self::SECTOR_SIZE;
        let mut aligned_offset = offset - data_offset;

        // Bypass clearing sector buffer for performance reasons
        let mut sector_data = MaybeUninit::<FlashSectorBuffer>::uninit();
        let sector_data = unsafe { sector_data.assume_init_mut() };

        while !bytes.is_empty() {
            self.internal_read(aligned_offset, &mut sector_data[..])?;

            let len = bytes.len().min((Self::SECTOR_SIZE - data_offset) as _);

            sector_data[data_offset as usize..][..len].copy_from_slice(&bytes[..len]);
            self.internal_erase(aligned_offset / Self::SECTOR_SIZE)?;
            self.internal_write(aligned_offset, &sector_data[..])?;

            aligned_offset += Self::SECTOR_SIZE;
            data_offset = 0;
            bytes = &bytes[len..];
        }

        Ok(())
    }
}
