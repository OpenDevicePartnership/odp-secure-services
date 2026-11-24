// Copyright 2025 Microsoft Corporation
// This project is dual-licensed under Apache 2.0 and MIT terms.
// See LICENSE-APACHE and LICENSE-MIT for details.

use std::env;
use std::path::PathBuf;

#[path = "../build_time.rs"]
mod build_time;

fn main() {
    println!(
        "cargo:rustc-env=BUILD_TIME={}",
        build_time::format(std::time::SystemTime::now())
    );
    if env::var("CARGO_CFG_TARGET_OS").unwrap() == "none" {
        let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
        let image_ld_path = manifest_dir.join("linker").join("image.ld");
        let ihv1_ld_path = manifest_dir.join("linker").join("ihv1.ld");

        println!("cargo:rustc-link-arg=-T{}", image_ld_path.display());
        println!("cargo:rustc-link-arg=-T{}", ihv1_ld_path.display());
        println!("cargo:rerun-if-changed={}", image_ld_path.display());
        println!("cargo:rerun-if-changed={}", ihv1_ld_path.display());
    }
}
