use crate::*;
use uuid::Uuid;

#[derive(Debug, Clone, PartialEq)]
pub struct MsgSendDirectResp2(DirectMessage);

impl Function for MsgSendDirectResp2 {
    const ID: FunctionId = FunctionId::MsgSendDirectResp2;
    type ReturnType = SmcCall;

    fn exec(self) -> ExecResult<Self::ReturnType> {
        exec_simple(self, Ok)
    }
}

impl MsgSendDirectResp2 {
    pub fn new(source_id: u16, destination_id: u16, uuid: Uuid, data: impl Into<DirectMessagePayload>) -> Self {
        Self(DirectMessage {
            source_id,
            destination_id,
            uuid,
            payload: data.into(),
        })
    }

    pub fn from_req_with_payload(req: &MsgSendDirectReq2, payload: impl Into<DirectMessagePayload>) -> Self {
        Self(DirectMessage {
            source_id: req.0.destination_id,
            destination_id: req.0.source_id,
            uuid: req.0.uuid,
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

impl HasRegisterPayload for MsgSendDirectResp2 {
    fn payload(&self) -> &DirectMessagePayload {
        &self.0.payload
    }
}

impl TryInto<SmcParams> for MsgSendDirectResp2 {
    type Error = Error;

    fn try_into(self) -> Result<SmcParams, Self::Error> {
        self.0.try_into()
    }
}

impl TryFrom<SmcParams> for MsgSendDirectResp2 {
    type Error = Error;

    fn try_from(value: SmcParams) -> Result<Self, Self::Error> {
        Ok(MsgSendDirectResp2(DirectMessage::try_from(value)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::function::msg::DirectMessagePayload; // Explicit import
    use rstest::rstest;
    use uuid::uuid; // Required for uuid! macro // Explicit import

    #[rstest]
    #[case::sample_resp(
        30, 40,
        uuid!("789a123b-c45d-6789-e012-345f678901ab"),
        DirectMessagePayload::from_iter((112..224).map(|i| i as u8))
    )]
    fn test_msg_send_direct_resp2_round_trip(
        #[case] source_id: u16,
        #[case] destination_id: u16,
        #[case] uuid: Uuid,
        #[case] payload: DirectMessagePayload,
    ) {
        let original_resp = MsgSendDirectResp2::new(source_id, destination_id, uuid, payload.clone());

        let params: SmcParams = original_resp.clone().try_into().unwrap();
        let new_resp: MsgSendDirectResp2 = params.try_into().unwrap();

        assert_eq!(original_resp, new_resp);
    }
}
