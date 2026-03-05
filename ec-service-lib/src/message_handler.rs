use odp_ffa::{MsgSendDirectReq2, MsgSendDirectResp2};

use crate::{msg_loop, Service};

pub struct MessageHandler<N> {
    node: N,
}

impl MessageHandler<HandlerNodeTerminal> {
    pub fn new() -> Self {
        Self {
            node: HandlerNodeTerminal,
        }
    }
}

impl Default for MessageHandler<HandlerNodeTerminal> {
    fn default() -> Self {
        Self::new()
    }
}

impl<N> MessageHandler<N>
where
    N: HandlerNode,
{
    pub fn append<S: Service>(self, service: S) -> MessageHandler<HandlerNodeInner<S, N>> {
        let node = HandlerNodeInner {
            service,
            next: self.node,
        };
        MessageHandler { node }
    }

    pub fn run_message_loop(mut self) -> core::result::Result<(), odp_ffa::Error> {
        msg_loop(|msg| self.node.handle(msg), |_| Ok(()))
    }
}

// A node in the linked list of services which handle FFA Direct Request messages
pub trait HandlerNode: Sized {
    fn handle(&mut self, msg: MsgSendDirectReq2) -> core::result::Result<MsgSendDirectResp2, odp_ffa::Error>;
}

// Inner node in the linked list of services
pub struct HandlerNodeInner<S, N> {
    service: S,
    next: N,
}

// Terminal node of the linked list of services
pub struct HandlerNodeTerminal;

impl HandlerNode for HandlerNodeTerminal {
    fn handle(&mut self, _: MsgSendDirectReq2) -> core::result::Result<MsgSendDirectResp2, odp_ffa::Error> {
        Err(odp_ffa::Error::Other("Unknown UUID"))
    }
}

impl<S, N> HandlerNode for HandlerNodeInner<S, N>
where
    S: Service,
    N: HandlerNode,
{
    fn handle(&mut self, msg: MsgSendDirectReq2) -> core::result::Result<MsgSendDirectResp2, odp_ffa::Error> {
        if S::UUID == msg.uuid() {
            self.service.ffa_msg_send_direct_req2(msg)
        } else {
            self.next.handle(msg)
        }
    }
}
