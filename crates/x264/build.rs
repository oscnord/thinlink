//! Generates the libx264 bindings from the system `x264.h`, so their layout is always the
//! installed library's own.

use std::env;
use std::error::Error;
use std::path::PathBuf;

const FUNCTIONS: &str = "x264_(param_default_preset|picture_init|encoder_open_[0-9]+|encoder_encode|encoder_close|encoder_reconfig)";
const TYPES: &str = "x264_(t|param_t|picture_t|nal_t)";
const VARS: &str = "X264_(CSP_I420|TYPE_.*|RC_ABR|AQ_VARIANCE|LOG_.*)";

fn main() -> Result<(), Box<dyn Error>> {
    let library = pkg_config::Config::new()
        .atleast_version("0.160")
        .probe("x264")?;
    // `x264_encoder_open` is a macro for the symbol with the library's build number, which
    // pkg-config reports as the version's second component.
    let build = library
        .version
        .split('.')
        .nth(1)
        .ok_or("x264 version has no build number")?;

    let mut builder = bindgen::Builder::default()
        .header_contents("wrapper.h", "#include <stdint.h>\n#include <x264.h>")
        .allowlist_function(FUNCTIONS)
        .allowlist_type(TYPES)
        .allowlist_var(VARS)
        .default_macro_constant_type(bindgen::MacroTypeVariation::Signed)
        .raw_line(format!(
            "pub unsafe fn x264_encoder_open(param: *mut x264_param_t) -> *mut x264_t {{\n    unsafe {{ x264_encoder_open_{build}(param) }}\n}}"
        ))
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()));
    for path in &library.include_paths {
        builder = builder.clang_arg(format!("-I{}", path.display()));
    }
    let out = PathBuf::from(env::var_os("OUT_DIR").ok_or("OUT_DIR is not set")?).join("x264.rs");
    builder.generate()?.write_to_file(out)?;
    Ok(())
}
