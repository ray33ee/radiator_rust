
use serde::{Serialize, Deserialize};

#[derive(Debug, Serialize, Deserialize)]
pub(crate) enum Function {

    Unlock,
    GetLock,
    GetPosition,
    CalibratePush(u32),
    CalibratePull(u32),
    Zero,
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

    Calibrate,
    SafeMode,
    Descale,
    Cancel,
    ShortBoost,
    LongBoost,



}

#[derive(Debug, Serialize, Deserialize)]
pub(crate) struct Message {
    pub function: Function,
}

