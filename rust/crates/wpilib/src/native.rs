//! Runtime loading of native shared libraries.
//!
//! Libraries are declared by base name (`"wpiHal"`) and opened by the platform file
//! name (`libwpiHal.so`, `wpiHal.dll`, `libwpiHal.dylib`), so the dynamic linker search
//! path decides where they come from: `/usr/local/frc/third-party/lib` on the roboRIO,
//! the folder `rust/native/<platform>` on a desktop (`cargo xtask sim` sets it).

/// Declares a struct of C function pointers resolved from one shared library.
///
/// ```ignore
/// native_lib! {
///     pub struct Hal = "wpiHal" {
///         fn HAL_Initialize(timeout: i32, mode: i32) -> i32;
///     }
/// }
/// ```
#[macro_export]
macro_rules! native_lib {
    (
        $vis:vis struct $name:ident = $base:literal {
            $( fn $f:ident( $( $arg:ident : $ty:ty ),* $(,)? ) $( -> $ret:ty )?; )*
        }
    ) => {
        #[allow(non_snake_case)]
        $vis struct $name {
            _lib: $crate::native::Library,
            $( pub $f: unsafe extern "C" fn( $( $ty ),* ) $( -> $ret )?, )*
        }

        #[allow(non_snake_case)]
        impl $name {
            pub const BASE: &'static str = $base;

            /// The platform file name of the library.
            pub fn file() -> ::std::string::String {
                $crate::native::file_name($base)
            }

            /// Opens the library and resolves every symbol.
            ///
            /// # Safety
            /// The declared signatures must match the library's C ABI.
            pub unsafe fn load() -> ::std::result::Result<Self, $crate::native::Error> {
                // SAFETY: running the library initializers is the caller's contract.
                let lib = unsafe { $crate::native::Library::new(Self::file()) }?;
                $(
                    // SAFETY: the signature comes from the vendor C header.
                    let $f = unsafe {
                        *lib.get::<unsafe extern "C" fn( $( $ty ),* ) $( -> $ret )?>(
                            concat!(stringify!($f), "\0").as_bytes(),
                        )?
                    };
                )*
                Ok(Self { _lib: lib, $( $f, )* })
            }
        }
    };
}

pub use libloading::{Error, Library};

/// The platform file name for a library base name.
pub fn file_name(base: &str) -> String {
    libloading::library_filename(base)
        .to_string_lossy()
        .into_owned()
}

fn search_path_var(os: &str) -> &'static str {
    match os {
        "windows" => "PATH",
        "macos" => "DYLD_LIBRARY_PATH",
        _ => "LD_LIBRARY_PATH",
    }
}

/// Panic text for a library that failed to load. `vendor` is "WPILib" or "CTRE".
pub fn load_failure(file: &str, error: &dyn std::fmt::Display, vendor: &str) -> String {
    format!(
        "cannot load {file}: {error}\nSet {} to the folder with the {vendor} natives \
         (rust/native/<platform>, fetched by `cargo xtask fetch-natives`; \
         /usr/local/frc/third-party/lib on the roboRIO). `cargo xtask sim` sets it for you.",
        search_path_var(std::env::consts::OS)
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn search_path_var_per_os() {
        assert_eq!(search_path_var("windows"), "PATH");
        assert_eq!(search_path_var("macos"), "DYLD_LIBRARY_PATH");
        assert_eq!(search_path_var("linux"), "LD_LIBRARY_PATH");
    }

    #[test]
    fn file_name_has_base_name() {
        let name = file_name("wpiHal");
        assert!(name.contains("wpiHal"));
        assert!(name.ends_with(std::env::consts::DLL_EXTENSION));
    }

    #[test]
    fn failure_text_names_file_and_vendor() {
        let text = load_failure("x.so", &"nope", "CTRE");
        assert!(text.contains("x.so") && text.contains("CTRE") && text.contains("nope"));
    }
}
