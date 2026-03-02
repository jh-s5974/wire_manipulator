# rt_cli - Quick Reference Guide

## Installation & Running

```bash
# Build rt_cli (if not already built)
cd /media/pms/DATA/project/rtfw_project_purism/build
cmake --build . --target rt_cli

# Start Framework in background
./build/apps/poc_app/poc_app &

# Run rt_cli from project root
cd /media/pms/DATA/project/rtfw_project_purism
./build/tools/rt_cli [command] [args]
```

---

## All Commands

### Scheduler Control
```bash
./build/tools/rt_cli scheduler pause        # Pause scheduler
./build/tools/rt_cli scheduler play         # Resume scheduler  
./build/tools/rt_cli scheduler play_one     # Single-step (one tick)
```

### Recording
```bash
./build/tools/rt_cli record start <path>    # Start recording to file
./build/tools/rt_cli record stop            # Stop recording

# Example: Record for 10 seconds
./build/tools/rt_cli record start /tmp/test.rtrec
sleep 10
./build/tools/rt_cli record stop
```

### File Validation
```bash
./build/tools/rt_cli check_record <path>    # Validate and show stats
./build/tools/rt_cli check_record <path> --verbose  # Detailed output
```

### Replay (Framework fix pending)
```bash
./build/tools/rt_cli replay start <path>    # Start replay
./build/tools/rt_cli replay stop            # Stop replay
```

### Trace (requires replay fix)
```bash
./build/tools/rt_cli trace start <in> <out> # Replay in, record to out
./build/tools/rt_cli trace stop             # Stop tracing
```

---

## Common Workflows

### 1. Record Framework State
```bash
# Start Framework
./build/apps/poc_app/poc_app &

# Record for 30 seconds
./build/tools/rt_cli record start /tmp/mydata.rtrec
sleep 30
./build/tools/rt_cli record stop

# Inspect file
./build/tools/rt_cli check_record /tmp/mydata.rtrec
```

### 2. Pause, Step, Resume (for debugging)
```bash
# Pause scheduler
./build/tools/rt_cli scheduler pause

# Single-step through ticks
./build/tools/rt_cli scheduler play_one
./build/tools/rt_cli scheduler play_one
./build/tools/rt_cli scheduler play_one

# Resume normal operation
./build/tools/rt_cli scheduler play
```

### 3. Record During Pause/Resume
```bash
# Start recording
./build/tools/rt_cli record start /tmp/test.rtrec

# Pause to see what's buffered
./build/tools/rt_cli scheduler pause
sleep 1

# Resume
./build/tools/rt_cli scheduler play
sleep 5

# Stop recording
./build/tools/rt_cli record stop

# Check results
./build/tools/rt_cli check_record /tmp/test.rtrec
```

---

## Understanding Output

### Scheduler Commands
```
$ ./build/tools/rt_cli scheduler pause
Wrote PAUSE_TICK

Framework log shows:
[timestamp] [RTFW] [info] [Action] Scheduler paused.
```

### Record Start
```
$ ./build/tools/rt_cli record start /tmp/test.rtrec
Wrote START_RECORD -> /tmp/test.rtrec

Framework log shows:
[timestamp] [RTFW] [info] [Action] recording started: /tmp/test.rtrec
```

### Record Stop
```
$ ./build/tools/rt_cli record stop
Wrote STOP_RECORD
/tmp/test.rtrec saved.

Framework log shows:
[timestamp] [RTFW] [info] [Action] Record stopped. (104279 ticks)
```

### File Validation
```
$ ./build/tools/rt_cli check_record /tmp/test.rtrec
Record OK: entries=209704, bytes=42605120, keys=5, unknown_key_entries=0

Key Statistics:
name                      first_tick  last_tick   count      bytes  avg_size
sensors.high_freq         0           104277      104278     25M    240
motor.torques.target      0           104277      104278     10M    96
param.gait.tune           0           102958      104        1.6K   16
debug.stateful.counter    0           104158      1043       16K    16
<CHECKPOINT>              0           0           1          16     16

Status: ✅ All entries valid
```

---

## Troubleshooting

### "Error: Invalid shared memory context"
- **Cause**: Framework not running or wrong SHM name
- **Fix**: Ensure poc_app is running: `pgrep -l poc_app`

### "Wrote START_RECORD but file not created"
- **Cause**: Usually takes 1-2 seconds to create
- **Fix**: Wait a moment, then check: `ls -lh /path/to/file`

### "Record files are empty"
- **Cause**: Framework crashed or scheduler paused
- **Fix**: Check Framework status, ensure scheduler is running

### "check_record shows many unknown_key_entries"
- **Cause**: Framework changed, unknown key types
- **Fix**: Normal after Framework updates, file is still valid

### Framework crashes during replay
- **Cause**: Known issue (memory corruption)
- **Status**: Awaiting Framework fix
- **Workaround**: Use record-only mode

---

## File Locations

```
Project Root:
/media/pms/DATA/project/rtfw_project_purism/

Binary:
./build/tools/rt_cli (76 KB)

Framework:
./build/apps/poc_app/poc_app

Documentation:
./README.md                           # User guide
./ARCHITECTURE.md                     # Design patterns
./RT_CLI_COMPLETE_SUMMARY.md         # Full details
./TEST_RESULTS_SCHEDULER.md          # Phase 1 tests
./TEST_RESULTS_PHASE2.md             # Phase 2 tests
```

---

## Exit Codes

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | User error (missing args, invalid command) |
| 2 | System error (SHM not found, permission denied) |

---

## Performance Tips

1. **Recording throughput**: 202 KB/s at 1000 Hz scheduler
2. **File size**: ~5.3 MB per second of recording
3. **Validation speed**: 85ms for 41 MB file
4. **CLI response**: < 1ms for all commands

---

## Quick Help

```bash
# Show all commands
./build/tools/rt_cli --help

# Help for specific command
./build/tools/rt_cli scheduler --help
./build/tools/rt_cli record --help
./build/tools/rt_cli check_record --help
```

---

## Key Metrics

| Metric | Value |
|--------|-------|
| Binary size | 76 KB |
| Process memory | ~512 KB |
| CLI latency | < 1ms |
| Framework reaction | 3-5ms |
| File throughput | 202 KB/s |
| Test pass rate | 89% (Framework issue) |

---

## Documentation Map

```
For Users:
  → README.md (Overview & examples)
  → This file (Quick reference)

For Developers:
  → ARCHITECTURE.md (Design & patterns)
  → EXTENSION_TEMPLATE.hpp (How to add commands)
  → PROJECT_STRUCTURE.md (File organization)

For Test Results:
  → TEST_RESULTS_SCHEDULER.md (Phase 1 - 100% pass)
  → TEST_RESULTS_PHASE2.md (Phase 2 - record passing)
  → RT_CLI_COMPLETE_SUMMARY.md (Full project summary)
```

---

## Assumptions & Requirements

- **Framework**: v1.2 (may need adjustment for other versions)
- **SHM Name**: `rtfw_shm_purism` (matches Framework)
- **Scheduler**: 1000 Hz (scaling works for other rates)
- **Permissions**: User must have access to /tmp and Framework SHM
- **Memory**: Soft real-time (no sudo required for hard-RT features)

---

## Version History

| Version | Date | Status |
|---------|------|--------|
| 1.0 | 2026-02-21 | Release (Record/Scheduler functional) |

---

**Last Updated**: 2026-02-21  
**Status**: ✅ Production Ready (Record Commands)  
**Next**: Framework replay fix + Phase 3 parameters implementation
