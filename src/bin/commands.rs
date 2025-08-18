
use heapless::{String, Vec};
use serde::{Serialize, Deserialize};

#[derive(Debug, Serialize, Deserialize)]
pub enum Function {
    Test,
    CalibratePush(u32),
    CalibratePull(u32),
    Unlock,
    DebugSetPosition(u32),
    DebugGetPosition,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Message {
    pub function: Function,
}

