use crate::{exec_simple, Error, ExecResult, Function, FunctionId, SmcParams};
use core::{fmt, iter::once};

const FFA_MAX_CHAR_COUNT: usize = 128;

#[derive(Default, Clone)]
pub struct Console<'bytes> {
    bytes: &'bytes [u8],
}

impl<'bytes> Console<'bytes> {
    pub fn new(bytes: &'bytes [u8]) -> (Self, &'bytes [u8]) {
        let (bytes, remaining) = bytes.split_at(FFA_MAX_CHAR_COUNT.min(bytes.len()));
        (Self { bytes }, remaining)
    }
}

impl Function for Console<'_> {
    const ID: FunctionId = FunctionId::ConsoleLog;
    type ReturnType = ();

    fn exec(self) -> ExecResult<Self::ReturnType> {
        if self.bytes.is_empty() {
            Ok(())
        } else {
            exec_simple(self, |_| Ok(()))
        }
    }
}

impl TryInto<SmcParams> for Console<'_> {
    type Error = Error;

    fn try_into(self) -> Result<SmcParams, Self::Error> {
        let char_data_iter = self.bytes.chunks(8).map(|c| {
            let mut buf = [0u8; 8];
            let len = 8.min(c.len());
            buf[..len].copy_from_slice(&c[..len]);
            u64::from_le_bytes(buf)
        });

        SmcParams::try_from_iter(once(self.bytes.len() as u64).chain(char_data_iter))
    }
}

struct ConsoleWriter;
impl fmt::Write for ConsoleWriter {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let mut s = s.as_bytes();
        loop {
            if s.is_empty() {
                break;
            }
            let (console, remaining) = Console::new(s);
            s = remaining;
            console.exec().map_err(|_| fmt::Error)?;
        }
        Ok(())
    }
}

#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    use fmt::Write;
    ConsoleWriter.write_fmt(args).unwrap();
}

/// Prints without a newline.
///
/// Carbon copy from <https://doc.rust-lang.org/src/std/macros.rs.html>
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => ($crate::_print(format_args!($($arg)*)));
}

/// Prints with a newline.
///
/// Carbon copy from <https://doc.rust-lang.org/src/std/macros.rs.html>
#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => ({
        $crate::_print(format_args!($($arg)*));
        $crate::_print(format_args!("\n"));
    })
}

use log::{Metadata, Record};

pub struct SpLogger;

impl log::Log for SpLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let module_path = record.module_path().unwrap_or("unknown");
            _print(format_args!(
                "{:<5} - {} - {}",
                record.level(),
                module_path,
                record.args()
            ));
        }
    }

    fn flush(&self) {}
}

#[cfg(all(not(target_arch = "aarch64"), test))]
mod tests {
    use super::*;
    use crate::smc::{get_smc_call_count, get_smc_calls, reset_smc_calls, SmcCall};

    #[track_caller]
    fn exec_test_helper(mut bytes: &[u8], expected_smc_calls: u32) {
        reset_smc_calls();

        loop {
            let (console, remaining) = Console::new(bytes);
            assert_eq!(console.exec(), Ok(()));
            bytes = remaining;
            if bytes.is_empty() {
                break;
            }
        }
        assert_eq!(get_smc_call_count(), expected_smc_calls);
    }

    #[test]
    fn test_console_exec_empty() {
        let bytes = [0u8; 0];
        exec_test_helper(&bytes, 0);
    }

    #[test]
    fn test_console_exec_exact_max_chars() {
        let bytes = [0u8; FFA_MAX_CHAR_COUNT];
        exec_test_helper(&bytes, 1);
    }

    #[test]
    fn test_console_exec_just_over_max_chars() {
        let bytes = [0u8; FFA_MAX_CHAR_COUNT + 1];
        exec_test_helper(&bytes, 2);
    }

    #[test]
    fn test_console_exec_multiple_of_max_chars() {
        let mut bytes = [0u8; FFA_MAX_CHAR_COUNT * 2];
        bytes[0..5].copy_from_slice(b"01234");
        bytes[FFA_MAX_CHAR_COUNT..FFA_MAX_CHAR_COUNT + 5].copy_from_slice(b"ABCDE");
        exec_test_helper(&bytes, 2);
        assert_eq!(
            get_smc_calls(),
            vec![
                SmcCall {
                    id: FunctionId::ConsoleLog,
                    params: SmcParams {
                        x1: 128,
                        x2: u64::from_le_bytes(*b"01234\0\0\0"),
                        ..Default::default()
                    },
                },
                SmcCall {
                    id: FunctionId::ConsoleLog,
                    params: SmcParams {
                        x1: 128,
                        x2: u64::from_le_bytes(*b"ABCDE\0\0\0"),
                        ..Default::default()
                    },
                },
            ]
        );
    }

    #[test]
    fn test_console_exec_long_string_multiple_chunks() {
        // FFA_MAX_CHAR_COUNT = 128. 300 bytes should be 3 chunks.
        // 300 / 128 = 2 with remainder 44.
        // So, chunk1=128, chunk2=128, chunk3=44.
        let bytes = [0u8; 300];
        exec_test_helper(&bytes, 3);
    }

    #[test]
    fn test_console_try_into_smc_params_short_string() {
        exec_test_helper(b"Hi", 1);
        assert_eq!(
            get_smc_calls(),
            vec![SmcCall {
                id: FunctionId::ConsoleLog,
                params: SmcParams {
                    x1: 2,
                    x2: u64::from_le_bytes(*b"Hi\0\0\0\0\0\0"),
                    ..Default::default()
                },
            }]
        );
    }

    #[test]
    fn test_console_try_into_smc_params_exact_8_bytes() {
        exec_test_helper(b"12345678", 1);
        assert_eq!(
            get_smc_calls(),
            vec![SmcCall {
                id: FunctionId::ConsoleLog,
                params: SmcParams {
                    x1: 8,
                    x2: u64::from_le_bytes(*b"12345678"),
                    ..Default::default()
                },
            }]
        );
    }

    #[test]
    fn test_console_try_into_smc_params_gt_8_bytes_not_multiple() {
        exec_test_helper(b"123456789", 1);
        assert_eq!(
            get_smc_calls(),
            vec![SmcCall {
                id: FunctionId::ConsoleLog,
                params: SmcParams {
                    x1: 9,
                    x2: u64::from_le_bytes(*b"12345678"),
                    x3: u64::from_le_bytes(*b"9\0\0\0\0\0\0\0"),
                    ..Default::default()
                },
            }]
        );
    }

    #[test]
    fn test_console_writer() {
        log::set_logger(&SpLogger).unwrap();
        log::set_max_level(log::LevelFilter::Debug);

        #[allow(unused)]
        #[derive(Debug)]
        struct InterruptId(u32);

        #[derive(Debug)]
        enum IrqType {
            Irq,
        }

        reset_smc_calls();
        log::debug!(
            "hf_interrupt_set: {:?} - {} - {:?}",
            InterruptId(543),
            true,
            IrqType::Irq
        );

        let parts = [
            "DEBUG",
            " - ",
            "odp_ffa:",
            " - ",
            "hf_interrupt_set: ",
            "InterruptId",
            "(",
            "543",
            ")",
            " - ",
            "true",
            " - ",
            "Irq",
        ];

        fn show(bs: &[u8]) -> String {
            String::from_utf8_lossy(bs).into_owned()
        }

        for (i, (expected, actual)) in parts.into_iter().zip(get_smc_calls().into_iter()).enumerate() {
            assert_eq!(FunctionId::ConsoleLog, actual.id);
            let mut expected_bytes = [0u8; 8];
            let to_copy = expected.len().min(8);
            expected_bytes[..to_copy].copy_from_slice(&expected.as_bytes()[..to_copy]);
            assert_eq!(
                u64::from_le_bytes(expected_bytes),
                actual.params.x2,
                "part {i}: expected: {:?}, actual: {:?}",
                show(&expected_bytes[..to_copy]),
                show(&actual.params.x2.to_le_bytes()[..to_copy])
            );
        }
    }
}
