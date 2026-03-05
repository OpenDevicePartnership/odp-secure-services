use log::error;
use odp_ffa::{FunctionId, MsgSendDirectReq2, MsgSendDirectResp2};
use uuid::Uuid;

use crate::Result;

pub trait Service {
    const UUID: Uuid;
    const NAME: &'static str;

    fn ffa_msg_send_direct_req2(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        self.handler_unimplemented(msg)
    }
}

pub(crate) trait ServiceImpl: Service {
    fn handler_unimplemented(&self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        error!("MsgSendDirectReq2 is unimplemented in {}: {:?}", Self::NAME, msg);
        Err(odp_ffa::Error::UnexpectedFunctionId(FunctionId::MsgSendDirectReq2))
    }
}

impl<T: Service + ?Sized> ServiceImpl for T {}
