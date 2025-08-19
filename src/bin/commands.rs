
use heapless::{String, Vec};
use serde::{Serialize, Deserialize};

#[derive(Debug, Serialize, Deserialize)]
pub(crate) enum Function {
    Test,
    CalibratePush(u32),
    CalibratePull(u32),
    Unlock,
    DebugSetPosition(u32),
    DebugGetPosition,
    DebugOpen,
    DebugClose,
}

#[derive(Debug, Serialize, Deserialize)]
pub(crate) struct Message {
    pub function: Function,
}

