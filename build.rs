use std::env;
use std::path::Path;
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=analyzer-core/main.cpp");
    println!("cargo:rerun-if-changed=analyzer-core/Makefile");

    let analyzer_path = Path::new("analyzer-core/stream-analyzer-core");
    if !analyzer_path.exists() {
        let status = Command::new("make")
            .args(["-C", "analyzer-core", "stream-analyzer-core"])
            .status()
            .expect("failed to invoke make for analyzer-core");

        if !status.success() {
            panic!("failed to build analyzer-core/stream-analyzer-core");
        }
    }

    let _ = env::var("CARGO_CFG_TARGET_OS");
    tauri_build::build()
}
