
use alloc::string::String;
use serde::{Serialize, Deserialize};

#[derive(Debug, Serialize, Deserialize)]
pub(crate) enum Function {

    UnlockAndZero,
    Lock,
    GetLock,
    GetPosition,
    CalibratePush(u32),
    CalibratePull(u32),
    GetMax,
    SetMax(u32),
    GetThermostat,
    SetThermostat(f32),
    ReadTemperature,
    GetMacAddress,
    GetResetReason,
    SoftReset,
    SyncTime(i64),
    GetTime,
    CurrentState,

    Calibrate,
    SafeMode,
    Descale,
    Cancel,
    ShortBoost,
    LongBoost,
    Rainbow,

    GetBoostDuration,

    SetShortDuration(u32),
    SetLongDuration(u32),

    SetVariant(String),
    GetVariant,

    AddSummerSlot(String),
    ClearSummer(String),

    AddWinterSlot(String),
    ClearWinter(String),

    AddBrightnessSlot(String),
    ClearBrightness(String),

    GetSummer,
    GetWinter,
    GetBrightness,

    RemoveSummerSlot(String),
    RemoveWinterSlot(String),
    RemoveBrightnessSlot(String),

    StartPanic(String),
    StartException(String),

    GetLEDBrightness,

    GetUpTime,

}

#[derive(Debug, Serialize, Deserialize)]
pub(crate) struct Message {
    pub function: Function,
}

