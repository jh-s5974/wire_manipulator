<!-- .github/copilot-instructions.md for rtfw_project_purism -->
# Copilot / AI assistant instructions — RT Framework (rtfw_project_purism)

Keep suggestions focused, minimal, and aligned with the project's runtime guarantees.

- **High-level intent**: This repository is a C++ real-time framework (RTFW) and a set of example apps. The framework enforces deterministic, data-driven scheduling (DAG of single-writer data keys), N-way buffering for timeline isolation, and shared-memory observability.

- **Key directories to inspect**: `rtfw/` (core engine), `rtfw-common/` (shared types/shm layout), `rtfw-connect/` (client APIs), `apps/` (example applications: `poc_app`, `rtmonitor`, `rtviewer`, `json_exporter`). See `README.txt` for conceptual notes.

- **Entry points and important files**:
  - Framework core: `rtfw/src/rt_framework.cpp` (initialization, dependency analysis, SHM layout, timeline creation)
  - Task interface: `rtfw/include/rtfw/rt_framework.h` / task headers (implement `rt::ITask::setup()` and `execute()`).
  - CMake: root `CMakeLists.txt`, `rtfw/CMakeLists.txt` (target `rtfw::rtfw` alias)

- **Design patterns & constraints the AI must preserve**:
  - Single-Writer Principle: each data key (e.g. `"sensors.high_freq"`) must have exactly one `DataWriter`. Do NOT add or suggest a second writer for an existing key.
  - Data-driven execution: tasks declare dependencies via `TaskRegistry` in `setup()` (example: `r.add_dependency(sensors_writer_)`) — code changes must keep dependency declarations in `setup()`.
  - RT vs Non-RT: use `registerTask(...)` for RT tasks (with frequency and CPU affinity) and `registerNonRtTask(...)` for non-real-time tasks (no affinity). CPU pinning is significant — preserve affinity semantics.
  - Shared memory (SHM) and logging: logging is wired to a SHM ring buffer via `ShmRingbufferSink`; avoid changes that break SHM initialization order in `rt_framework.cpp` (logger must be set up after SHM build/populate).

- **Build / run / debug** (how devs actually build and run locally):
  - Basic out-of-source build:
    ```bash
    mkdir -p build && cd build
    cmake ..
    cmake --build .
    ```
  - The project uses CMake with `-g` enabled in `rtfw/CMakeLists.txt`; use `gdb`/`rr` for debugging. Logs appear in stdout and the SHM log buffer.
  - Example: build and run `poc_app` (from `build/apps/poc_app/`), or run monitor tools `rtmonitor`, `rtviewer` from `build/apps/`.

- **Important external deps**: Boost (headers), `spdlog` (async logger), POSIX libs `pthread` and `rt`. Keep suggestions compatible with these dependencies and the CMake usage in `rtfw/CMakeLists.txt`.

- **Typical code edits**:
  - Adding a new RT task: implement a class inheriting `rt::ITask`, implement `setup()` (declare dependencies) and `execute()` (write/read via DataWriter/DataReader), then register with `framework.registerTask(std::make_unique<MyTask>(), frequency, cpu_core);` in `main()`.
  - Use provided helper types where available (see `apps/poc_app/*` for small examples).

- **What to avoid changing**:
  - SHM layout and `rtfw-common` public headers; they define binary layouts and are consumed by external client tools.
  - Dependency analysis ordering in `rt_framework.cpp` — subtle changes may break determinism.
  - Single-writer keys and data type definitions in `rtfw-common`.

- **Search pointers for AI** (quick grep targets):
  - `registerTask(` — where tasks are registered
  - `TaskRegistry` / `add_dependency` — how tasks declare data IO
  - `ShmRingbufferSink` / `spdlog::async_logger` — logging tied to SHM
  - `FrameworkConfig` / `Mode::RECORDING` — blackbox/record/replay behavior

- **When proposing new APIs or refactors**:
  - Keep backward compatibility for SHM layout and public headers in `rtfw-common`.
  - Prefer small, incremental changes; propose follow-up migrations (with tests or migration steps) when larger changes are needed.

- **Examples to copy from repository**:
  - Task example and types: `apps/poc_app/biped_data_types.h`, `apps/poc_app/main.cpp`
  - Timeline/initialization patterns: `rtfw/src/rt_framework.cpp` (initialize sequence: collect tasks → analyze deps → build SHM → create timelines → start)

- **Blackbox / Snapshot (new runtime APIs)**:
  - The framework supports dynamic start/stop recording with an optional checkpoint buffer. The interface exposed by `rtfw::blackbox::IBlackbox` includes `start_dynamic(path, checkpoint, start_tick)` and `stop_dynamic()`; the concrete `FileBlackbox` implements these.
  - Checkpoints are written as a special log entry using a reserved key hash `CHECKPOINT_KEY_HASH` (see `rtfw-common/include/rtfw_common/log_format.h`). The checkpoint data should be the contiguous `_state_buffer` from the framework.
  - Tasks that carry state should derive from `rt::Task<StateT>` where `StateT` is trivially-copyable; the framework copies `StateT` into the `_state_buffer` and the checkpoint is written as raw bytes. Keep `StateT` POD to ensure snapshot correctness.
  - The runtime control path: `handleAction()` in `rtfw/src/rt_framework.cpp` calls `_record_backend->start_dynamic(...)` and `_record_backend->stop_dynamic()` when processing `FrameworkAction::START_LOGGING` / `STOP_LOGGING`.
  - When proposing format changes, preserve backwards compatibility: prefer adding a reserved key/type for new records rather than changing existing binary header sizes.

If anything here is unclear or you'd like more detail (e.g., a checklist for adding a new task, or sample `FrameworkConfig` fields), tell me which area to expand. After your feedback I'll iterate on this file.
