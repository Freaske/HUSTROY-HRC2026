# HRC2026 Code Work — current status

Source: Team task list for 02–08 November 2026.
This starter includes only the first smoke-test milestone; it is not a complete robot firmware.

## Stage 1 (today): infrastructure
- [ ] GitHub repository published and teammates invited
- [ ] Board model confirmed
- [ ] PlatformIO build SUCCESS
- [ ] ESP32 LED blinking in simulation (screenshot or link)
- [ ] First branch / PR reviewed by appointed reviewer A

## Stage 2: hardware and module skeleton
- [ ] Toan provides pin mapping
- [ ] Thang provides ultrasonic sensor details
- [ ] config.h updated with verified GPIO mapping and thresholds
- [ ] motor module skeleton and sensor module skeleton compile

## Stage 3: bench tests
- [ ] MODE 1: left/right motor spin + inversion flags
- [ ] MODE 2: print all sensor values
- [ ] MODE 3: line-follow-only with challenges disabled
- [ ] Non-blocking control loop (millis(), no delay())

## Stage 4: integration and optional support features
- [ ] PID tuning (KP/KD) after actual motor/sensor validation
- [ ] Bluetooth parameter tuning (confirm ESP32 model and phone compatibility first)
- [ ] Bluetooth log for state/line/distance
- [ ] HC-SR04P verified against physical distances of 5–30 cm
- [ ] Results table: run number, parameters, time, result, failure
- [ ] Physical robot line follows on practice track

If the original 08/11 deadline is not achievable, report actual status and ask for a new agreed test date; do not mark incomplete work done.
