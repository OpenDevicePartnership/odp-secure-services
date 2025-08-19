use crate::{exec_simple, Error, ExecResult, Function, FunctionId, SmcParams};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NotificationBind {
    sender_id: u16,
    receiver_id: u16,
    flags: NotificationBindFlags,
    notification_bitmap: u64,
}

#[derive(Default, Debug, Clone, Copy, PartialEq, num_enum::TryFromPrimitive, num_enum::IntoPrimitive)]
#[repr(u64)]
pub enum NotificationBindFlags {
    #[default]
    Global = 0,
    PerVCpu = 1,
}

impl NotificationBind {
    pub fn new(sender_id: u16, receiver_id: u16, flags: NotificationBindFlags, notification_bitmap: u64) -> Self {
        Self {
            sender_id,
            receiver_id,
            flags,
            notification_bitmap,
        }
    }
}

impl Function for NotificationBind {
    const ID: FunctionId = FunctionId::NotificationBind;
    type ReturnType = ();

    fn exec(self) -> ExecResult<Self::ReturnType> {
        exec_simple(self, |_| Ok(()))
    }
}

impl TryInto<SmcParams> for NotificationBind {
    type Error = Error;

    fn try_into(self) -> Result<SmcParams, Self::Error> {
        let bitmap_low = self.notification_bitmap & 0xffffffff;
        let bitmap_high = self.notification_bitmap >> 32;
        Ok(SmcParams {
            x1: ((self.sender_id as u64) << 16) | (self.receiver_id as u64),
            x2: self.flags.into(),
            x3: bitmap_low,
            x4: bitmap_high,
            ..Default::default()
        })
    }
}
