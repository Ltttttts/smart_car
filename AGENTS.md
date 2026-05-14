# AGENTS.md

## Project Shape
- This is a small Linux C11 robot-control project, not the aspirational ROS2 tree described later in `PROPOSAL.md`; trust the current `Makefile`, `include/`, `modules/`, and `app/` sources over proposal sections.
- Entry points are `app/joystick/main.c`, `app/ai/main.c`, and `app/camera/main.c`; shared modules under `modules/` are compiled into joystick and AI executables.
- Public headers are included as `"hal/..."`, `"driver/..."`, `"control/..."`, `"comm/..."` via `-Iinclude -Imodules`.

## Commands
- Build everything with `make`; outputs are `build/joystick`, `build/ai_control`, and `build/camera_stream`.
- Clean with `make clean`; package source with `make dist`.
- Hardware run targets use `/dev/ttyUSB0`: `make run-joystick`, `make calibrate-joystick`, and `make run-ai`.
- Joystick mode can start camera streaming with `make run-joystick JOYSTICK_ARGS='--camera --camera-device /dev/video0 --camera-res 640x480 --camera-fps 30 --camera-quality 60 --camera-port 8080'`.
- Camera streaming runs `make run-camera CAMERA_ARGS='--device /dev/video0 --res 640x480 --fps 30 --quality 60 --port 8080'`; it execs an existing `mjpg_streamer` install rather than vendoring it.
- Default motor device in code is usually `/dev/ttyS1` at `921600` baud (`include/common.h`); Makefile run targets override it with `/dev/ttyUSB0`.
- AI mode requires `LLM_API_KEY`; optional env vars are `LLM_API_URL` and `LLM_MODEL` (default `deepseek-v4-flash`).
- This Windows workspace may not have `make` installed; verify on the Orange Pi/Linux target when local `make` is unavailable.

## Build Gotchas
- App entry objects are explicit (`joystick_main.o`, `ai_control.o`, `camera_stream.o`) because multiple entry files are named `main.c`; keep it that way if adding more apps.
- Link flags include `-lm -lpthread -lcurl`; libcurl development headers/libraries are required for the default `make` build even though camera mode does not call LLM APIs.
- There are no checked-in tests, scripts, config files, CI workflows, or formatter/linter configs at the repo root.

## Hardware Safety
- Treat motor-control edits as safety-critical embedded C. Load/use the repo-local `embedded-c-coding` OpenCode skill before changing C, HAL, driver, RTOS-like loop, or hardware-interaction code.
- `app/ai/main.c` has `#define HARDWARE_ENABLED 0`; leave it `0` for simulation unless the user explicitly wants real motor output.
- Joystick mode always initializes and enables motors; use calibration/test paths carefully around real hardware.
- Four EMM_V5 drivers share one UART bus; motor order is always `[FL, FR, RL, RR]`, but addresses are `FL=2`, `FR=4`, `RL=1`, `RR=3` in `include/common.h`.
- Multi-motor velocity updates normally buffer each motor with `sync=true`, then call `emm_motor_sync_trigger()` so all wheels start together.

## Code Conventions
- The project uses opaque-handle style for device objects (`SerialPort_t`, `EmmMotor_t`); do not expose struct internals in headers.
- Non-public functions and file globals are `static`; errors use `ErrorCode_t` negative values from `include/common.h`.
- Existing comments and user-facing strings are Chinese and include `@author Ltttttts`; match that style when adding similar C documentation.
- Avoid adding new dependencies casually: `json_helper` is intentionally a tiny flat-JSON extractor, not a full JSON parser.

## Repo Notes
- `docs/reference/Emm_V5/` contains vendor/reference driver material; do not treat it as the active driver implementation.
- `.gitignore` excludes `build/`, object/dependency files, `.env`, `*.tar.gz`, IDE directories, and `node_modules/`.
