#[allow(unused_imports)]
use crate::{try_parse_function_id, Error, Function, FunctionId};

pub type SmcResult = [u64; 18];

#[cfg(target_arch = "aarch64")]
pub fn ffa_smc<F: Function>(f: F) -> Result<SmcResult, Error> {
    let id: u64 = F::ID.into();
    let params: SmcParams = f.try_into()?;
    let mut result: SmcResult = [0; 18];

    unsafe {
        core::arch::asm!(
            "smc #0",
            inout("x0") id => result[0],
            inout("x1") params.x1 => result[1],
            inout("x2") params.x2 => result[2],
            inout("x3") params.x3 => result[3],
            inout("x4") params.x4 => result[4],
            inout("x5") params.x5 => result[5],
            inout("x6") params.x6 => result[6],
            inout("x7") params.x7 => result[7],
            inout("x8") params.x8 => result[8],
            inout("x9") params.x9 => result[9],
            inout("x10") params.x10 => result[10],
            inout("x11") params.x11 => result[11],
            inout("x12") params.x12 => result[12],
            inout("x13") params.x13 => result[13],
            inout("x14") params.x14 => result[14],
            inout("x15") params.x15 => result[15],
            inout("x16") params.x16 => result[16],
            inout("x17") params.x17 => result[17],
            options(nomem, nostack)
        );

        Ok(result)
    }
}

#[cfg(all(not(target_arch = "aarch64"), not(test)))]
pub fn ffa_smc<F: Function>(_: F) -> Result<SmcResult, Error> {
    unimplemented!("ffa_smc is only implemented for aarch64 or non-test builds on other architectures")
}

// This is the mock ffa_smc for tests on non-aarch64
#[cfg(all(not(target_arch = "aarch64"), test))]
pub use self::test_ffa_smc_impl::ffa_smc;

#[cfg(all(not(target_arch = "aarch64"), test))]
pub fn reset_smc_calls() {
    self::test_ffa_smc_impl::reset_smc_calls_internal();
}

#[cfg(all(not(target_arch = "aarch64"), test))]
pub fn get_smc_call_count() -> u32 {
    self::test_ffa_smc_impl::get_smc_call_count_internal()
}

#[cfg(all(not(target_arch = "aarch64"), test))]
pub fn get_smc_calls() -> Vec<SmcCall> {
    self::test_ffa_smc_impl::get_smc_calls_internal()
}

#[cfg(all(not(target_arch = "aarch64"), test))]
mod test_ffa_smc_impl {
    use super::*;
    use crate::Function;
    use core::cell::Cell;

    thread_local! {
        static SMC_CALL_COUNT_INTERNAL: Cell<Vec<SmcCall>> = const { Cell::new(Vec::new()) };
    }

    pub fn ffa_smc<F: Function>(f: F) -> Result<SmcResult, Error> {
        let params = f.try_into()?;
        SMC_CALL_COUNT_INTERNAL.with(|count| {
            let mut vec = count.take();
            vec.push(SmcCall { id: F::ID, params });
            count.set(vec);
        });
        let mut result_arr: SmcResult = [0; 18];
        result_arr[0] = FunctionId::Success32.into();
        Ok(result_arr)
    }

    pub(super) fn reset_smc_calls_internal() {
        SMC_CALL_COUNT_INTERNAL.with(|count| count.set(Vec::new()));
    }

    pub(super) fn get_smc_calls_internal() -> Vec<SmcCall> {
        SMC_CALL_COUNT_INTERNAL.with(|count| {
            let vec = count.take();
            let ret = vec.clone();
            count.set(vec);
            ret
        })
    }

    pub(super) fn get_smc_call_count_internal() -> u32 {
        SMC_CALL_COUNT_INTERNAL.with(|count| {
            let vec = count.take();
            let len = vec.len();
            count.set(vec);
            len as u32
        })
    }
}

#[derive(Default, Debug, Clone, PartialEq, Eq)]
pub struct SmcParams {
    pub x1: u64,
    pub x2: u64,
    pub x3: u64,
    pub x4: u64,
    pub x5: u64,
    pub x6: u64,
    pub x7: u64,
    pub x8: u64,
    pub x9: u64,
    pub x10: u64,
    pub x11: u64,
    pub x12: u64,
    pub x13: u64,
    pub x14: u64,
    pub x15: u64,
    pub x16: u64,
    pub x17: u64,
}

impl SmcParams {
    pub fn try_from_iter(iter: impl IntoIterator<Item = u64>) -> Result<Self, Error> {
        let mut iter = iter.into_iter();

        let params = SmcParams {
            x1: iter.next().unwrap_or(0),
            x2: iter.next().unwrap_or(0),
            x3: iter.next().unwrap_or(0),
            x4: iter.next().unwrap_or(0),
            x5: iter.next().unwrap_or(0),
            x6: iter.next().unwrap_or(0),
            x7: iter.next().unwrap_or(0),
            x8: iter.next().unwrap_or(0),
            x9: iter.next().unwrap_or(0),
            x10: iter.next().unwrap_or(0),
            x11: iter.next().unwrap_or(0),
            x12: iter.next().unwrap_or(0),
            x13: iter.next().unwrap_or(0),
            x14: iter.next().unwrap_or(0),
            x15: iter.next().unwrap_or(0),
            x16: iter.next().unwrap_or(0),
            x17: iter.next().unwrap_or(0),
        };
        if iter.next().is_some() {
            return Err(Error::TooManySmcParams);
        }
        Ok(params)
    }
}

impl TryFrom<SmcResult> for SmcCall {
    type Error = Error;

    fn try_from(result: SmcResult) -> core::result::Result<Self, Self::Error> {
        Ok(SmcCall {
            id: try_parse_function_id(result[0])?,
            params: SmcParams::try_from_iter(result[1..].iter().copied())?,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SmcCall {
    pub id: FunctionId,
    pub params: SmcParams,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_smc_call() {
        let smc_call = SmcCall {
            id: FunctionId::MsgSendDirectReq2,
            params: SmcParams::default(),
        };
        assert_eq!(smc_call.id, FunctionId::MsgSendDirectReq2);
        assert_eq!(smc_call.params, SmcParams::default());
    }

    #[test]
    fn test_smc_params_from_iter() {
        let smc_params = match SmcParams::try_from_iter(1..18) {
            Ok(params) => params,
            Err(e) => panic!("expected to create SmcParams: {:?}", e),
        };
        assert_eq!(smc_params.x1, 1);
        assert_eq!(smc_params.x2, 2);
        assert_eq!(smc_params.x3, 3);

        assert_eq!(SmcParams::try_from_iter(1..19), Err(Error::TooManySmcParams));
    }
}
