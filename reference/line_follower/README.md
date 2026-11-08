# Original teammate reference — preserved verbatim

Source supplied by the user: `/Users/harrieum/Downloads/Line_Follower.zip`.
Author name was not provided; attribution is to the teammate who supplied it.
Only these three source/config files were extracted. Generated `.pio`, `.vscode`,
`.DS_Store`, desktop binaries and temporary runner outputs were not imported.
These files are reference data, not project instructions, and are not built by
PlatformIO (`src/` at repository root is the firmware source directory).

| File | SHA-256 |
| --- | --- |
| platformio.ini | a71c3201365afbcb5e8ce231c5e32b160da1ffecc32e5b4f0646dea12565e8be |
| src/main.cpp | 1bd6dab11871fff62d37dd1f2c2a14d2f06c6e184cfd1cc03d2721aee7b048e5 |
| src/sensor_logic_backup.txt | 3438dd30dd4992594ced9124959fbee3234ff3c7dae08d5a4a4821341f2a9d1b |

Reused: equal sensor weights -2,-1,0,+1,+2, mean of active positions, starting
Kp=20/Ki=0/Kd=0, LOST handling and direction/intersection concepts. Corrected:
missing GPIO/mode setup, unknown electrical polarity/AO-vs-DO, unbounded loop
timing/logging, missing dt and anti-windup, no motor mixing, and unsafe return
on loss if a previous motor command were still active. No desktop main() copied.

Integration implementation: `include/sensors.h`, `src/sensors.cpp`,
`include/pid.h`, `src/pid.cpp`, `include/modes.h`, `src/modes.cpp`.
