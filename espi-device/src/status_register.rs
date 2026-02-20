bit_register! {
    /// Status register bits as defined in the eSPI specification
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    pub struct StatusRegister: little_endian u16 {
        /// Peripheral Posted/Completion Rx Queue Free
        pub pc_free: bool => [0],
        /// Peripheral Non-Posted Rx Queue Free
        pub np_free: bool => [1],
        /// Virtual Wire Rx Queue Free (Always '1')
        pub vwire_free: bool => [2],
        /// OOB Posted Rx Queue Free
        pub oob_free: bool => [3],
        /// Peripheral Posted/Completion Tx Queue Available
        pub pc_avail: bool => [4],
        /// Peripheral Non-Posted Tx Queue Available
        pub np_avail: bool => [5],
        /// Virtual Wire Tx Queue Available
        pub vwire_avail: bool => [6],
        /// OOB Posted Tx Queue Available
        pub oob_avail: bool => [7],
        /// Flash Completion Rx Queue Free (Always '1')
        pub flash_c_free: bool => [8],
        /// Flash Non-Posted Rx Queue Free
        pub flash_np_free: bool => [9],
        /// Flash Completion Tx Queue Available
        pub flash_c_avail: bool => [12],
        /// Flash Non-Posted Tx Queue Available
        pub flash_np_avail: bool => [13],
    }
}
