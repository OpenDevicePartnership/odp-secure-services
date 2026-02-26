use log::error;
use odp_ffa::{FunctionId, MsgSendDirectReq2, MsgSendDirectResp2};
use uuid::Uuid;

use crate::msg_loop;

pub type Result<T> = core::result::Result<T, odp_ffa::Error>;

pub trait Service {
    fn service_name(&self) -> &'static str;
    fn service_uuid(&self) -> Uuid;

    fn ffa_msg_send_direct_req2(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        self.handler_unimplemented(msg)
    }
}

pub trait ServiceNodeHandler {
    fn handle(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2>;
}

pub struct ServiceNode<This: Service, Next: ServiceNodeHandler> {
    service: This,
    next: Next,
}

impl<This: Service, Next: ServiceNodeHandler> ServiceNode<This, Next> {
    pub fn run_message_loop(
        &mut self,
        mut before_handle_message: impl FnMut(&MsgSendDirectReq2) -> core::result::Result<(), odp_ffa::Error>,
    ) -> Result<()> {
        msg_loop(|msg| self.handle(msg), |msg| before_handle_message(msg))
    }
}

pub struct ServiceNodeNone;
impl ServiceNodeHandler for ServiceNodeNone {
    fn handle(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        error!("Unknown UUID {}", msg.uuid());
        Err(odp_ffa::Error::Other("Unknown UUID"))
    }
}

impl<S: Service, N: ServiceNodeHandler> ServiceNode<S, N> {
    pub fn new(service: S, next: N) -> Self {
        Self { service, next }
    }
}

impl<This: Service, Next: ServiceNodeHandler> ServiceNodeHandler for ServiceNode<This, Next> {
    fn handle(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        if msg.uuid() == self.service.service_uuid() {
            self.service.ffa_msg_send_direct_req2(msg)
        } else {
            log::debug!(
                "UUID does not match {} ({}), forwarding to next node",
                self.service.service_name(),
                self.service.service_uuid()
            );
            self.next.handle(msg)
        }
    }
}

#[macro_export]
macro_rules! service_list {
    ($service:expr, $($next:expr),+$(,)?) => {
        $crate::ServiceNode::new($service, $crate::service_list!($($next),*))
    };

    ($service:expr) => {
        $crate::ServiceNode::new($service, $crate::ServiceNodeNone)
    };

    () => {
        $crate::ServiceNodeNone
    };
}

pub(crate) trait ServiceImpl: Service {
    fn handler_unimplemented(&self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        error!(
            "MsgSendDirectReq2 is unimplemented in {}: {:?}",
            self.service_name(),
            msg
        );
        Err(odp_ffa::Error::UnexpectedFunctionId(FunctionId::MsgSendDirectReq2))
    }
}

impl<T: Service + ?Sized> ServiceImpl for T {}
