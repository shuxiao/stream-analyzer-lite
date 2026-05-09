#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::path::PathBuf;
use std::process::Command;
use tauri::Manager;
use tauri_plugin_dialog::DialogExt;

const ANALYZER_BINARY_NAME: &str = "stream-analyzer-core";

fn executable_name(name: &str) -> String {
    if cfg!(target_os = "windows") {
        format!("{name}.exe")
    } else {
        name.to_string()
    }
}

fn workspace_analyzer_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("analyzer-core")
        .join(executable_name(ANALYZER_BINARY_NAME))
}

fn resolve_analyzer_path(app: &tauri::AppHandle) -> Result<PathBuf, String> {
    let resource_candidate = app
        .path()
        .resource_dir()
        .map_err(|e| e.to_string())?
        .join("analyzer-core")
        .join(executable_name(ANALYZER_BINARY_NAME));

    if resource_candidate.exists() {
        return Ok(resource_candidate);
    }

    let workspace_candidate = workspace_analyzer_path();
    if workspace_candidate.exists() {
        return Ok(workspace_candidate);
    }

    Err(format!(
        "Analyzer binary not found. Checked: {}, {}",
        resource_candidate.display(),
        workspace_candidate.display()
    ))
}

#[tauri::command]
fn analyze(
    app: tauri::AppHandle,
    file: String,
    thumbnails: Option<bool>,
    start: Option<i32>,
    count: Option<i32>,
) -> Result<String, String> {
    let bin = resolve_analyzer_path(&app)?;

    let mut cmd = Command::new(&bin);
    if thumbnails.unwrap_or(false) {
        cmd.arg("--thumbnails");
    }
    if let Some(s) = start {
        cmd.arg("--range").arg(s.to_string()).arg(count.unwrap_or(50).to_string());
    }
    cmd.arg(&file);

    let output = cmd
        .output()
        .map_err(|e| format!("Failed to run analyzer: {}", e))?;

    if !output.status.success() {
        return Err(String::from_utf8_lossy(&output.stderr).to_string());
    }
    Ok(String::from_utf8_lossy(&output.stdout).to_string())
}

#[tauri::command]
async fn pick_file(app: tauri::AppHandle) -> Result<String, String> {
    let (tx, rx) = std::sync::mpsc::channel();
    app.dialog()
        .file()
        .add_filter("Video", &["mp4", "mkv", "avi", "flv", "ts", "mov", "h264", "264", "h265", "265", "hevc"])
        .pick_file(move |f| { let _ = tx.send(f); });

    match rx.recv().map_err(|e| e.to_string())? {
        Some(f) => Ok(f.to_string()),
        None => Err("cancelled".to_string()),
    }
}

fn main() {
    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .invoke_handler(tauri::generate_handler![analyze, pick_file])
        .run(tauri::generate_context!())
        .expect("error running tauri app");
}
