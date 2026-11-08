# Hardware pin mapping — awaiting confirmation

Owner: Toan (pin map), Thang (HC-SR04P notes); code integrator: Nam.

Do **not** guess GPIO numbers. Fill this out after wiring is confirmed.

| Device | Signal | ESP32 GPIO | Verified? | Notes |
| --- | --- | --- | --- | --- |
| TCRT5000 #1 | AO or DO? | TBD | No | Confirm module output type |
| TCRT5000 #2 | AO or DO? | TBD | No | |
| TCRT5000 #3 | AO or DO? | TBD | No | |
| TCRT5000 #4 | AO or DO? | TBD | No | |
| TCRT5000 #5 | AO or DO? | TBD | No | |
| TB6612 | AIN1, AIN2, PWMA | TBD | No | |
| TB6612 | BIN1, BIN2, PWMB | TBD | No | |
| TB6612 | STBY | TBD | No | |
| HC-SR04P | TRIG, ECHO | TBD | No | Confirm voltage/logic compatibility |
| Button (optional) | Input | TBD | No | Serial selection is an alternative |

Also confirm: exact ESP32 board model, motor supply voltage, common ground,
ESP32-safe logic levels, wheel/motor direction, and whether sensor outputs are analog or digital.

Five analog sensor outputs require five independent ADC inputs.
