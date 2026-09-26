# quad copter simulation ghost playback tool

c++ cli tool for ghost.json playback

ghost.h contains json structure definition and parsing

main.cpp contains main playback loop and cli application

to build the whole project call ```./build.sh``` from ````cd ./../../`` nothing more! 

dont use tail command to check the build output, it is too long. use grep and filter for errors.

this is a linux and windows project and crossbuilds for both with mingw

for testing call ```../../build/linux/install/bin/simitl-playback```

never use literal UTF-8 characters in the source code, this breaks your ability to edit files!

# Running it

- Always run from this directory (`tools/simitl-playback`): `config/` and `test.bin` are resolved relative to cwd.
- Modes: `g <ghost.json>` (ghost RC playback), `bb <file.csv>` (blackbox RC replay), `olbb <file.csv>` (open-loop motor replay, bypasses the PID), `i` (idle, default). Flags: `-ff` fast-forward, `-ns` skip the 3 s startup sleep, `-no` suppress CLI OSD output, `--start/--len <sec>` playback window, `--config <path>` quad json.
- `test.bin` = the quad's simulated eeprom, loaded at startup and flushed on MSP 250 (eeprom write). A truncated/partial file is OK — missing tail fields fall back to defaults (prints `failed to load 'test.bin, <size> < <expected>'`).
- Blackbox `.bbl` output requires blackbox enabled in the eeprom; a default-generated eeprom has it OFF. Copy a game .bin (e.g. `~/.config/unity3d/sigsegowl/pr0p/config/quad/hq-65mm-micro.bin`) onto test.bin to get logging.
- The eeprom filename is hardcoded to `test.bin` (main.cpp `stateInit.eepromName`); `ghost.bin` in this dir is just a leftover copy.
- ghost.json rcData values are normalized [-1,1]; they land in stateInput.rcData as RATE commands (acro) — a held pitch value is a sustained pitch RATE, i.e. a tumble, not a tilt.
- bb mode: `readBlackboxFile` requires the decoded CSV's column header (`loopIteration ... rcCommand[0] ...`) to appear on a line index > 0. A decoder output that starts on line 0 fails with "Could not find column header line"; prepend one `H ...` line.
- MSP live over `ws://localhost:5761` (UART1): works in every mode including mid-flight during ghost playback — `MSP_SELECT_SETTING` (210) switches pid profile live, so one run can A/B tunes. Command cheat sheet and the profile layout of the game bins are documented in the repo-root AGENTS.md.
- Runtime noise/config knobs come from `config/quad/*.json`: propWashFactor, gyroBaseNoiseAmp, propHarmonic1/2, motorImbalance1/2/3, frame harmonic amps, motorVariance, maxGroundEffectDist.
- Simulated bidirectional-DSHOT eRPM telemetry is force-enabled in `src/sim/bf.cpp` regardless of the eeprom's dshot_bidir flag, so the Betaflight RPM filter always works headless.