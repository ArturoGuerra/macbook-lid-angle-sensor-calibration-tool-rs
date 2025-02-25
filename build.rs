use std::{fs::File, io::Write, path::PathBuf};

fn main() {
    let out = PathBuf::from(std::env::var_os("OUT_DIR").expect("failed to get target dir"));
    let memory_x = include_bytes!("memory.x");
    File::create(out.join("memory.x"))
        .expect("failed to create memory.x file in target dir")
        .write_all(memory_x)
        .expect("failed to write memory.x file to target dir");

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=memory.x");
}
