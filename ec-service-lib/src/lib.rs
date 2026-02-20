#![no_std]

mod service;
pub mod services;
pub mod sp_logger;

use log::{debug, error, info};
use odp_ffa::{Function, MsgSendDirectReq2, MsgSendDirectResp2, MsgWait, RxTxMap, TryFromSmcCall};
pub use service::{Result, Service, ServiceNode, ServiceNodeHandler, ServiceNodeNone};

// For reference, here are the UUIDs for services that ec-service-lib defines (not all of them are implemented)
// const UUID_EC_SVC_NOTIFY: Uuid = uuid!("B510B3A3-59F6-4054-BA7A-FF2EB1EAC765");
// const UUID_EC_SVC_MANAGEMENT: Uuid = uuid!("330c1273-fde5-4757-9819-5b6539037502");
// const UUID_EC_SVC_POWER: Uuid = uuid!("7157addf-2fbe-4c63-ae95-efac16e3b01c");
// const UUID_EC_SVC_BATTERY: Uuid = uuid!("25cb5207-ac36-427d-aaef-3aa78877d27e");
// const UUID_EC_SVC_THERMAL: Uuid = uuid!("31f56da7-593c-4d72-a4b3-8fc7171ac073");
// const UUID_EC_SVC_UCSI: Uuid = uuid!("65467f50-827f-4e4f-8770-dbf4c3f77f45");
// const UUID_EC_SVC_TIME_ALARM: Uuid = uuid!("23ea63ed-b593-46ea-b027-8924df88e92f");
// const UUID_EC_SVC_DEBUG: Uuid = uuid!("0bd66c7c-a288-48a6-afc8-e2200c03eb62");
// const UUID_EC_SVC_OEM: Uuid = uuid!("9a8a1e88-a880-447c-830d-6d764e9172bb");

#[derive(PartialOrd, Ord, PartialEq, Eq, Debug)]
pub enum HafEcError {
    Ok,
    InvalidParameters,
}

#[derive(Default)]
pub struct HafEcService {
    _tx_buffer_base: u64,
    _rx_buffer_base: u64,
    _rxtx_page_count: u32,
}

impl HafEcService {
    pub fn new() -> Self {
        Self { ..Default::default() }
    }

    pub fn map_rxtx_buffers(&mut self, tx_base: u64, rx_base: u64, page_count: u32) -> HafEcError {
        // Map in shared RX/TX buffers
        debug!(
            "Mapping shared RX/TX buffers:
               TX_BUFFER_BASE: 0x{:x}
               RX_BUFFER_BASE: 0x{:x}
               RXTX_PAGE_COUNT: 0x{:x}",
            tx_base, rx_base, page_count
        );

        let result = RxTxMap::new(tx_base, rx_base, page_count).exec();
        match result {
            Ok(()) => {
                debug!("Successfully mapped RXTX buffers");
                HafEcError::Ok
            }
            Err(e) => {
                // This is fatal, terminate SP
                debug!("Error mapping RXTX buffers: {:?}", e);
                HafEcError::InvalidParameters
            }
        }
    }
}

async fn async_msg_loop(
    mut handler: impl AsyncFnMut(MsgSendDirectReq2) -> core::result::Result<MsgSendDirectResp2, odp_ffa::Error>,
    mut before_handle_message: impl AsyncFnMut(&MsgSendDirectReq2) -> core::result::Result<(), odp_ffa::Error>,
) -> core::result::Result<(), odp_ffa::Error> {
    info!("async_msg_loop: start");
    let mut msg = MsgWait::new().exec()?;
    info!("async_msg_loop: msg: {:?}", msg);
    loop {
        msg = if let Ok(request) = MsgSendDirectReq2::try_from_smc_call(msg.clone()) {
            info!("async_msg_loop: request: {:?}", request);
            before_handle_message(&request).await?;
            match handler(request).await {
                Ok(response) => {
                    info!("async_msg_loop: response: {:?}", response);
                    response.exec()?
                }
                Err(e) => {
                    error!("Error handling FFA message: {:?}", e);
                    MsgWait::new().exec()?
                }
            }
        } else {
            error!("Unexpected FFA message: {:?}", msg);
            MsgWait::new().exec()?
        }
    }
}
