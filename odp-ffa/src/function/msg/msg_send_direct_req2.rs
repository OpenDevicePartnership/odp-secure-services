use crate::*;
use uuid::Uuid;

#[derive(Debug, Clone, PartialEq)]
pub struct MsgSendDirectReq2(pub(crate) DirectMessage);

impl MsgSendDirectReq2 {
    pub fn new(source_id: u16, destination_id: u16, uuid: Uuid, payload: impl Into<DirectMessagePayload>) -> Self {
        Self(DirectMessage {
            source_id,
            destination_id,
            uuid,
            payload: payload.into(),
        })
    }

    pub fn source_id(&self) -> u16 {
        self.0.source_id
    }

    pub fn destination_id(&self) -> u16 {
        self.0.destination_id
    }

    pub fn uuid(&self) -> Uuid {
        self.0.uuid
    }
}

impl HasRegisterPayload for MsgSendDirectReq2 {
    fn payload(&self) -> &DirectMessagePayload {
        &self.0.payload
    }
}

impl Function for MsgSendDirectReq2 {
    const ID: FunctionId = FunctionId::MsgSendDirectReq2;
    type ReturnType = SmcCall;

    fn exec(self) -> ExecResult<Self::ReturnType> {
        exec_simple(self, Ok)
    }
}

impl TryInto<SmcParams> for MsgSendDirectReq2 {
    type Error = Error;

    fn try_into(self) -> Result<SmcParams, Self::Error> {
        self.0.try_into()
    }
}

impl TryFrom<SmcParams> for MsgSendDirectReq2 {
    type Error = Error;

    fn try_from(value: SmcParams) -> Result<Self, Self::Error> {
        Ok(MsgSendDirectReq2(DirectMessage::try_from(value)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::function::msg::DirectMessagePayload;
    use rstest::rstest;
    use uuid::uuid; // Required for uuid! macro // Explicit import for RegisterPayload

    #[rstest]
    #[case::sample_req(
        10, 20,
        uuid!("123e4567-e89b-12d3-a456-426614174000"),
        DirectMessagePayload::from_iter((0..112).map(|i| i as u8))
    )]
    fn test_msg_send_direct_req2_round_trip(
        #[case] source_id: u16,
        #[case] destination_id: u16,
        #[case] uuid: Uuid,
        #[case] payload: DirectMessagePayload,
    ) {
        let original_req = MsgSendDirectReq2::new(source_id, destination_id, uuid, payload.clone());

        let params: SmcParams = original_req.clone().try_into().unwrap();
        let new_req: MsgSendDirectReq2 = params.try_into().unwrap();

        assert_eq!(original_req, new_req);
    }
}
