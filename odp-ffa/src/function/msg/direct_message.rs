use uuid::Uuid;

use crate::{smc::SmcParams, util::combine_low_high_u16, DirectMessagePayload, Error, HasRegisterPayload};

#[derive(Debug, Clone, PartialEq)]
pub struct DirectMessage {
    pub source_id: u16,
    pub destination_id: u16,
    pub uuid: Uuid,
    pub payload: DirectMessagePayload,
}

impl HasRegisterPayload for DirectMessage {
    fn payload(&self) -> &DirectMessagePayload {
        &self.payload
    }
}

impl TryFrom<DirectMessage> for SmcParams {
    type Error = Error;

    fn try_from(msg: DirectMessage) -> Result<Self, Self::Error> {
        let (uuid_high, uuid_low) = msg.uuid.as_u64_pair();
        SmcParams::try_from_iter(
            [
                combine_low_high_u16(msg.source_id, msg.destination_id),
                uuid_high.to_be(),
                uuid_low.to_be(),
            ]
            .into_iter()
            .chain(msg.payload.registers_iter()),
        )
    }
}

impl TryFrom<SmcParams> for DirectMessage {
    type Error = Error;

    fn try_from(value: SmcParams) -> Result<Self, Self::Error> {
        let source_id = (value.x1 & 0xFFFF) as u16;
        let destination_id = (value.x1 >> 16) as u16;

        let uuid_high = u64::from_be(value.x2);
        let uuid_low = u64::from_be(value.x3);
        let uuid = Uuid::from_u64_pair(uuid_high, uuid_low);

        // x4-x17 are for payload (14 registers)
        let payload_regs = [
            value.x4, value.x5, value.x6, value.x7, value.x8, value.x9, value.x10, value.x11, value.x12, value.x13,
            value.x14, value.x15, value.x16, value.x17,
        ];
        let payload_bytes_iter = payload_regs.iter().flat_map(|&reg| u64::to_le_bytes(reg).into_iter());

        let payload = DirectMessagePayload::from_iter(payload_bytes_iter);

        Ok(DirectMessage {
            source_id,
            destination_id,
            uuid,
            payload,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;
    use uuid::uuid;

    #[rstest]
    #[case::simple_message(
        1, 2,
        uuid!("67e55044-10b1-426f-9247-bb680e5fe0c8"),
        DirectMessagePayload::from_iter((0..112).map(|i| i as u8))
    )]
    #[case::zero_ids(
        0, 0,
        uuid!("00000000-0000-0000-0000-000000000000"),
        DirectMessagePayload::from_iter(core::iter::repeat(0u8).take(112))
    )]
    #[case::max_ids(
        u16::MAX, u16::MAX,
        uuid!("ffffffff-ffff-ffff-ffff-ffffffffffff"),
        DirectMessagePayload::from_iter((0..112).map(|i| (i % 256) as u8))
    )]
    fn test_direct_message_round_trip(
        #[case] source_id: u16,
        #[case] destination_id: u16,
        #[case] uuid: Uuid,
        #[case] payload: DirectMessagePayload,
    ) {
        let original_message = DirectMessage {
            source_id,
            destination_id,
            uuid,
            payload: payload.clone(), // Clone payload for the original message
        };

        let params: SmcParams = original_message.clone().try_into().unwrap();
        let new_message: DirectMessage = params.try_into().unwrap();

        assert_eq!(original_message, new_message);
    }
}
