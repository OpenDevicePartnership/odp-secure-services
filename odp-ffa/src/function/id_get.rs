use crate::{exec_simple, Error, ExecResult, Function, FunctionId, SmcParams};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IdGet;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IdGetResult {
    pub id: u16,
}

impl Function for IdGet {
    const ID: FunctionId = FunctionId::IdGet;
    type ReturnType = IdGetResult;

    fn exec(self) -> ExecResult<Self::ReturnType> {
        exec_simple(self, |result| {
            Ok(IdGetResult {
                id: result.params.x2 as u16,
            })
        })
    }
}

impl TryInto<SmcParams> for IdGet {
    type Error = Error;

    fn try_into(self) -> Result<SmcParams, Self::Error> {
        Ok(SmcParams { ..Default::default() })
    }
}
