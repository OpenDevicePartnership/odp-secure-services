use crate::{exec_simple, Error, ExecResult, Function, FunctionId, SmcParams};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Interrupt {
    endpoint_id: u16,
    vcpu_id: u16,
    interrupt_id: u32,
}

impl Interrupt {
    pub fn new(endpoint_id: u16, vcpu_id: u16, interrupt_id: u32) -> Self {
        Self {
            endpoint_id,
            vcpu_id,
            interrupt_id,
        }
    }
}

impl Function for Interrupt {
    const ID: FunctionId = FunctionId::Interrupt;
    type ReturnType = ();

    fn exec(self) -> ExecResult<Self::ReturnType> {
        exec_simple(self, |_| Ok(()))
    }
}

impl TryInto<SmcParams> for Interrupt {
    type Error = Error;

    fn try_into(self) -> Result<SmcParams, Self::Error> {
        let endpoint_and_vcpu_id = ((self.endpoint_id as u64) << 16) | (self.vcpu_id as u64);
        Ok(SmcParams {
            x1: endpoint_and_vcpu_id,
            x2: self.interrupt_id as u64,
            ..Default::default()
        })
    }
}
