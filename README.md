# Stream Analyzer Lite

`Stream Analyzer Lite` is a lightweight desktop tool for inspecting H.264 and H.265/HEVC video bitstreams frame by frame.

It is built for engineers who need to understand compressed video structure quickly: frame size distribution, GOP boundaries, packet PTS progression, NALU composition, and decoded thumbnail navigation are all available in a single local desktop app.

## What It Does

- Open a local video file and analyze its compressed video stream locally.
- Support H.264 and H.265/HEVC elementary video streams carried in common containers.
- Visualize per-frame packet size with an interactive bar chart.
- Show frame index, frame type, packet PTS, and packet size.
- Group frames by GOP and browse them in a thumbnail-oriented view.
- Inspect NALUs inside each packet, including parsed details for common parameter and slice units.
- Decode thumbnails and full-frame previews on demand for GOP browsing.
- Jump to an approximate position by PTS.

## Typical Use Cases

- Debug unexpected GOP structure or keyframe spacing.
- Inspect whether SPS/PPS/VPS and slice layout look reasonable.
- Compare packet size patterns across frames.
- Quickly review a compressed stream without opening a full media analysis suite.
- Use as a local visualization aid while debugging encoder, packager, or streaming pipeline issues.

## Current Capabilities

The app currently exposes the following analysis views:

- `Bar chart view`
  Visualizes packet size per frame and lets you hover or click frames to inspect metadata and NALUs.
- `Thumbnail view`
  Loads decoded thumbnails GOP by GOP and shows approximate frame dependency relationships for the selected frame.
- `AV sync view`
  Draws audio and video decode timestamps on a shared timeline, supports zooming, and highlights segments whose A/V drift exceeds `200ms`.
- `NALU detail panel`
  Parses and displays structured fields for common H.264/H.265 units such as:
  - H.264: `SPS`, `PPS`, `SEI`, slice headers
  - H.265/HEVC: `VPS`, `SPS`, `PPS`, `SEI`, slice headers

## Important Scope Notes

- This is a bitstream inspection tool, not a media player or transcoder.
- Full analysis is focused on `H.264` and `H.265/HEVC` video streams.
- The main frame list is packet-oriented. In normal mode, frame type in the bar chart is inferred conservatively from packet flags.
- Thumbnail mode uses actual decoder output and is more representative for displayed decode picture type.
- Reference arrows in thumbnail view are a lightweight visualization, not a full standards-accurate decoded picture buffer model.
- NALU field parsing is intentionally pragmatic and partial. It is designed for debugging convenience, not for full spec validation.

## Architecture

The repository contains three main parts:

- `src/`
  Tauri desktop shell written in Rust. It opens files and invokes the native analyzer binary.
- `ui/`
  Static HTML/CSS/JavaScript frontend bundled into the desktop app.
- `analyzer-core/`
  Native C++ analyzer built on top of FFmpeg libraries. It parses packets, extracts NALUs, and optionally decodes thumbnails/previews.

There is also an optional local development backend:

- `tools/dev-backend/`
  A tiny Go HTTP wrapper used when you want to run the frontend in a browser instead of through the Tauri shell.

## How It Works

1. The Tauri app asks you to choose a local video file.
2. Rust launches `analyzer-core/stream-analyzer-core`.
3. The C++ analyzer uses FFmpeg to:
   - locate the first H.264 or H.265 video stream
   - parse packets and NALUs
   - extract codec metadata
   - optionally decode frames for thumbnails and previews
4. The analyzer returns JSON to the frontend.
5. The frontend renders interactive views for stream inspection.

## Requirements

You need the following toolchain installed locally:

- `Rust` and `cargo`
- `Tauri CLI`
- `g++` or another compatible C++ compiler
- `make`
- `pkg-config`
- FFmpeg development libraries:
  - `libavformat`
  - `libavcodec`
  - `libavutil`
  - `libswscale`

On Linux, you will also need the usual Tauri native dependencies for building desktop applications.

At this stage, the project is only supported on Linux, and the verified environment is `Ubuntu 25.10` with `Wayland`.

## Build

Build from the repository root.

### Platform Support

- The app currently supports `Linux` only.
- It has only been tested on `Ubuntu 25.10` running `Wayland`.
- `macOS` and `Windows` are not supported at this time.

### Desktop Development

```bash
make desktop-dev
```

This will:

- build `analyzer-core/stream-analyzer-core`
- start the Tauri desktop app in development mode

### Desktop Release Bundles

```bash
make desktop-build
```

This builds Linux desktop bundles via Tauri and currently normalizes generated package names to the project slug:

- `target/release/bundle/deb/stream-analyzer-lite_0.1.0_amd64.deb`
- `target/release/bundle/rpm/stream-analyzer-lite-0.1.0-1.x86_64.rpm`

### Build Only the Native Analyzer

```bash
make analyzer
```

Or:

```bash
make -C analyzer-core stream-analyzer-core
```

## Run

After starting the desktop app:

1. Open a local video file.
2. Analyze the stream.
3. Switch between `BarChart`, `Thumbnails`, and `AVSync`.
4. Click a frame to inspect NALUs or frame preview data.

## Browser-Only Local UI Development

If you want to work on the frontend without launching Tauri, the repository includes a minimal local backend in Go.

Start the backend:

```bash
cd tools/dev-backend
go run .
```

It exposes:

- `GET http://localhost:9210/analyze?file=/absolute/or/relative/path`

Optional query parameters:

- `thumbnails=1`
- `start=<frame_index>`
- `count=<frame_count>`

Then serve `ui/` with any static file server and let the page call the backend.

## Analyzer CLI

The native analyzer can also be run directly:

```bash
./analyzer-core/stream-analyzer-core <video_file>
```

Enable thumbnail output:

```bash
./analyzer-core/stream-analyzer-core --thumbnails <video_file>
```

Request thumbnails for a frame range:

```bash
./analyzer-core/stream-analyzer-core --thumbnails --range 100 50 <video_file>
```

The analyzer writes JSON to stdout.

## Project Layout

```text
.
├── analyzer-core/        # C++ analyzer built on FFmpeg
├── docs/ref/             # screenshots and references
├── icons/                # app icons and bundle assets
├── packaging/            # Linux desktop packaging templates/assets
├── src/                  # Rust + Tauri shell
├── tools/dev-backend/    # optional Go HTTP wrapper for frontend-only dev
├── ui/                   # static frontend
├── Cargo.toml
├── Makefile
└── tauri.conf.json
```

## Known Limitations

- Only the first detected `H.264` or `H.265/HEVC` video stream is analyzed.
- The UI is desktop-first and currently optimized around local file inspection.
- Packet-level and decoded-frame-level semantics are intentionally mixed depending on view mode.
- Some slice header parsing is simplified and not exhaustive.
- The tool does not aim to replace full FFmpeg/codec conformance analyzers.
- There is no export/report workflow yet.

## Why Tauri + Native C++

This project uses Tauri for a lightweight desktop shell and keeps codec-heavy work in a native C++ binary:

- the UI stays simple and portable
- analysis remains local
- FFmpeg integration is direct
- iteration on UI and parser logic stays decoupled

## License

This project is licensed under the `GNU GPLv3`. See [LICENSE](LICENSE).
