#pragma once
#include <algorithm>

void runPureLogicChecks() {
    const auto pattern = [](const std::string& bits) {
        std::array<bool, SENSOR_COUNT> values{};
        for (std::size_t i = 0; i < SENSOR_COUNT; ++i) values[i] = bits[i] == '1';
        return analyzeSensorPattern(values);
    };
    assert(pattern("00100").position == 0 && pattern("00100").direction == LineDirection::FORWARD);
    assert(pattern("01100").position == -0.5f && pattern("01100").direction == LineDirection::LEFT);
    assert(pattern("00110").position == 0.5f && pattern("00110").direction == LineDirection::RIGHT);
    assert(pattern("00000").state == LineState::LINE_LOST);
    assert(pattern("11111").allActive && pattern("11111").state == LineState::INTERSECTION);
    assert(pattern("10001").state == LineState::INTERSECTION);
    assert(pattern("01110").direction == LineDirection::FORWARD);
    assert(!pattern("01010").valid);
    for (unsigned int mask = 0; mask < 32; ++mask) {
        std::string bits(5, '0');
        int sum = 0;
        int count = 0;
        for (int i = 0; i < 5; ++i) if (mask & (1u << i)) {
            bits[i] = '1'; sum += i - 2; ++count;
        }
        const auto frame = pattern(bits);
        assert(frame.activeCount == count);
        if (count) assert(std::abs(frame.position - static_cast<float>(sum) / count) < 0.0001f);
        if (frame.state == LineState::LINE_DETECTED) assert(frame.valid && frame.position >= -2 && frame.position <= 2);
    }
    const std::array<int, 5> thresholds{{2000, 2100, 2200, 2300, 2400}};
    const std::array<int, 5> analog{{1000, 1000, 3000, 1000, 1000}};
    const std::array<int, 5> digital{{1, 1, 0, 1, 1}};
    auto frame = interpretSensorRaw(analog, SensorInputMode::ANALOG_AO, thresholds, false, true);
    assert(frame.valid && frame.rawAvailable && frame.position == 0 && frame.activeCount == 1);
    frame = interpretSensorRaw(digital, SensorInputMode::DIGITAL_DO, thresholds, true, true);
    assert(frame.valid && frame.activeCount == 1 && frame.active[2]);
    frame = interpretSensorRaw(analog, SensorInputMode::ANALOG_AO, thresholds, false, false);
    assert(frame.rawAvailable && !frame.valid && frame.active[2] && frame.state == LineState::INVALID);
    auto invalid = analog;
    invalid[1] = 4096;
    assert(!interpretSensorRaw(invalid, SensorInputMode::ANALOG_AO, thresholds, false, true).valid);
    invalid[1] = -1;
    assert(!interpretSensorRaw(invalid, SensorInputMode::ANALOG_AO, thresholds, false, true).valid);
    invalid = digital; invalid[0] = 2;
    assert(!interpretSensorRaw(invalid, SensorInputMode::DIGITAL_DO, thresholds, false, true).valid);
    auto badThresholds = thresholds; badThresholds[2] = 0;
    assert(!interpretSensorRaw(analog, SensorInputMode::ANALOG_AO, badThresholds, false, true).valid);
    auto boundary = thresholds;
    assert(interpretSensorRaw(boundary, SensorInputMode::ANALOG_AO, thresholds, false, true).allActive);
    const std::array<int, 5> benchThresholds{{265, 291, 284, 325, 550}};
    const std::array<bool, 5> splitLine{{true, false, true, false, false}};
    const std::array<int, 5> noisyLine{{268, 278, 351, 318, 359}};
    frame = interpretSensorRaw(noisyLine, SensorInputMode::ANALOG_AO, benchThresholds, false, true);
    assert(!frame.valid && frame.activeCount == 2 && frame.active == splitLine && frame.raw == noisyLine);
    const std::array<int, 5> shiftedLine{{282, 289, 332, 300, 327}};
    frame = interpretSensorRaw(shiftedLine, SensorInputMode::ANALOG_AO, benchThresholds, false, true);
    assert(!frame.valid && frame.activeCount == 2 && frame.active == splitLine && frame.raw == shiftedLine);
    auto ambiguous = noisyLine;
    ambiguous[0] = 320;
    assert(!interpretSensorRaw(ambiguous, SensorInputMode::ANALOG_AO, benchThresholds, false, true).valid);
    auto invertedNoise = benchThresholds;
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) invertedNoise[i] = 2 * benchThresholds[i] - noisyLine[i];
    frame = interpretSensorRaw(invertedNoise, SensorInputMode::ANALOG_AO, benchThresholds, true, true);
    assert(!frame.valid && frame.activeCount == 2 && frame.active == splitLine);

    PidController pid({20, 0, 0, 2, 50});
    assert(pid.update(0.5f, 0.02f).output == 10);
    assert(pid.update(2, 0.02f).output == 40);
    assert(pid.update(-2, 0.02f).output == -40);
    PidController derivative({0, 0, 1, 2, 100});
    assert(derivative.update(1, 0.02f).d == 0);
    assert(std::abs(derivative.update(1.5f, 0.1f).output - 5) < 0.0001f);
    derivative.reset();
    assert(derivative.update(-1, 0.02f).d == 0);
    PidController integral({0, 2, 0, 0.25f, 50});
    integral.update(1, 0.1f);
    assert(std::abs(integral.update(1, 0.1f).output - 0.4f) < 0.0001f);
    for (int i = 0; i < 100; ++i) integral.update(1, 0.1f);
    assert(std::abs(integral.integral() - 0.25f) < 0.0001f);
    PidController saturated({100, 10, 0, 2, 5});
    for (int i = 0; i < 100; ++i) assert(saturated.update(2, 0.02f).output == 5);
    assert(saturated.integral() == 0);
    assert(saturated.update(-2, 0.02f).output == -5);
    PidController iOnly({0, 10, 0, 2, 5});
    assert(iOnly.update(1, 1).output == 5 && iOnly.integral() == 0);
    assert(!pid.update(1, 0).valid);
    assert(!pid.update(1, 2).valid);
    assert(!pid.update(std::numeric_limits<float>::quiet_NaN(), 0.02f).valid);
    assert(!pid.update(std::numeric_limits<float>::infinity(), 0.02f).valid);
    PidController badPid({std::numeric_limits<float>::infinity(), 0, 0, 2, 50});
    assert(!badPid.update(1, 0.02f).valid);
    assert(forwardCorrectionLimit(50, 150, 100) == 50);
    assert(forwardCorrectionLimit(140, 150, 100) == 75);
    assert(forwardCorrectionLimit(150, 150, 50) == 50);
    assert(forwardCorrectionLimit(151, 150, 50) == 0);
    const auto speeds = mixForwardSpeeds(50, 20, 150);
    assert(speeds.valid && speeds.left == 70 && speeds.right == 30);
    const auto saturatedRight = mixForwardSpeeds(150, 20, 150);
    const auto saturatedLeft = mixForwardSpeeds(150, -20, 150);
    assert(saturatedRight.valid && saturatedRight.left == 150 && saturatedRight.right == 110);
    assert(saturatedLeft.valid && saturatedLeft.left == 110 && saturatedLeft.right == 150);
    assert(mixForwardSpeeds(50, 1000, 150).left == 150);
    assert(mixForwardSpeeds(50, 1000, 150).right == 0);
    assert(!mixForwardSpeeds(50, std::numeric_limits<float>::quiet_NaN(), 150).valid);
}

void runLineV5LogicChecks() {
    assert(lineCalibrationValid(LINE_BENCH_CALIBRATION, false));
    assert(!lineCalibrationValid(LINE_BENCH_CALIBRATION, true));
    auto broken = LINE_BENCH_CALIBRATION;
    broken.coordinates[2] = broken.coordinates[1];
    assert(!lineCalibrationValid(broken, false));
    auto sample = [](const std::array<int, SENSOR_COUNT>& raw) {
        return observeLine(interpretSensorRaw(raw, SensorInputMode::ANALOG_AO, SENSOR_ANALOG_THRESHOLDS,
                                              false, true), SensorInputMode::ANALOG_AO, LINE_BENCH_CALIBRATION);
    };
    const auto white = LINE_BENCH_CALIBRATION.white;
    auto raw = white;
    raw[2] = 590; // Historical S3 false positive with the legacy 572-count threshold.
    assert(sample(raw).reading == LineReading::Missing && !sample(raw).signal);
    raw[2] = 2550;
    raw[3] = 1500; // Useful analog edge below the legacy binary threshold.
    const auto interpolated = sample(raw);
    assert(interpolated.reading == LineReading::Position && interpolated.position > 0.20f &&
           interpolated.position < 0.40f && interpolated.confidence > 0.5f);
    raw = white;
    raw.front() = raw.back() = 3500;
    assert(sample(raw).reading == LineReading::Ambiguous && sample(raw).signal);
    assert(sample(LINE_BENCH_CALIBRATION.black).reading == LineReading::Ambiguous);
    raw[2] = 5000;
    assert(sample(raw).reading == LineReading::Invalid);

    LineFollower follow;
    auto settings = tuningParameters();
    settings.trim = 0;
    settings.start = {25, 30, 30, 35};
    settings.sustain = {15, 20, 20, 25};
    raw = white;
    raw[2] = LINE_BENCH_CALIBRATION.black[2];
    const auto center = sample(raw);
    unsigned now = 0;
    auto tick = [&](const LineObservation& line) { now += LINE_PERIOD_US; return follow.update(line, settings, now, LINE_PERIOD_US); };
    auto result = tick(center);
    assert(result.d == 0 && result.left == 0 && result.right == 0);
    result = tick(center);
    assert(result.left == 25 && result.right == 30);
    for (int i = 0; i < 30; ++i) result = tick(center);
    assert(result.left == 60 && result.right == 60);
    const auto missing = sample(white);
    result = tick(missing);
    assert(result.state == FollowState::Search && result.base > 0);
    tick(missing);
    result = tick(missing);
    assert(result.base == 0 && result.left + result.right <= 0);
    result = tick(center);
    assert(result.state == FollowState::Recover && result.d == 0 && result.left == 0 && result.right == 0);
    result = tick(missing);
    assert(result.state == FollowState::Search && result.base == 0);
    for (unsigned i = 0; i < LINE_REACQUIRE_SAMPLES; ++i) result = tick(center);
    assert(result.state == FollowState::Recover && result.base < 0.48f * settings.speed);
    for (unsigned i = 0; i < LINE_SETTLE_US / LINE_PERIOD_US; ++i) result = tick(center);
    assert(result.state == FollowState::Tracking && result.base < settings.speed);
    assert(follow.update(center, settings, now + LINE_STALE_US, LINE_STALE_US).fault);
}

void setPattern(const std::string& bits) {
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        const bool high = INVERT_LINE_SENSOR ? bits[i] != '1' : bits[i] == '1';
        rawInputs[SENSOR_PINS[i]] = SENSOR_INPUT_MODE == SensorInputMode::ANALOG_AO ?
                                   (high ? SENSOR_ADC_MAX : 0) : (high ? HIGH : LOW);
    }
}
void step(unsigned long milliseconds = CONTROL_PERIOD_MS) { nowMs += milliseconds; loop(); }
constexpr auto FOLLOW_MS = LINE_PERIOD_US / 1000;
void followTick() { step(FOLLOW_MS); }
void warmup() {
    for (unsigned int i = 0; i < LINE_START_VALID_SAMPLES; ++i) {
        if (modesStatus().mode == RobotMode::LINE_FOLLOW) followTick();
        else step();
    }
}
void applyCalibration() {
    if (SENSOR_INPUT_MODE != SensorInputMode::ANALOG_AO) return;
    const std::string low = "0,0,0,0,0", high = "4095,4095,4095,4095,4095";
    const auto& white = INVERT_LINE_SENSOR ? high : low;
    const auto& black = INVERT_LINE_SENSOR ? low : high;
    send("CAL=" + white + "/" + black + "/" + white + "\n");
    assert(lineCalibrationApplied());
}

void startFollow() {
    setPattern("00100");
    send("MODE LINE_FOLLOW\n");
    expectStopped();
    assert(!modesStatus().armed);
    warmup(); // A valid line alone must never authorize motion.
    send("RUN\n");
    assert(!modesStatus().running && !modesStatus().armed);
    expectStopped();
    send("ARM\n");
    assert(modesStatus().armed);
    warmup();
    send("RUN\n");
    if (!modesStatus().running) std::cerr << "RUN unexpectedly blocked at ms=" << nowMs << "\n" << Serial.output;
    assert(modesStatus().running);
    expectStopped(); // RUN itself must not issue a wheel command.
    for (int i = 0; i < 22; ++i) followTick();
    expectSpeeds(tuningParameters().speed + tuningParameters().trim,
                 tuningParameters().speed - tuningParameters().trim);
}

void runSystemChecks(const std::string& scenario) {
    expectStopped();
    if (scenario == "system-right-line") {
        setPattern("00000");
        send("MODE RIGHT_LINE\nARM\n"); warmup(); send("GO\n");
        assert(!modesStatus().armed && !modesStatus().running);
        expectStopped();
        setPattern("00100"); warmup(); send("GO\n");
        assert(!modesStatus().running); expectStopped();

        const auto startRightLine = [] {
            setPattern("00100"); send("ARM\n"); warmup(); send("GO\n");
            assert(modesStatus().running);
            expectStopped();
            step(); expectSpeeds(0, RIGHT_LINE_TEST_PWM);
        };
        startRightLine();
        const std::array<int, SENSOR_COUNT> shifted{{282, 289, 332, 300, 327}};
        for (std::size_t i = 0; i < SENSOR_COUNT; ++i) rawInputs[SENSOR_PINS[i]] = shifted[i];
        step(); expectSpeeds(0, RIGHT_LINE_TEST_PWM);
        const std::array<int, SENSOR_COUNT> wideLine{{256, 272, 321, 501, 1841}};
        for (std::size_t i = 0; i < SENSOR_COUNT; ++i) rawInputs[SENSOR_PINS[i]] = wideLine[i];
        step(); expectSpeeds(0, RIGHT_LINE_TEST_PWM);
        const std::array<int, SENSOR_COUNT> shiftedBackground{{277, 299, 304, 310, 330}};
        for (std::size_t i = 0; i < SENSOR_COUNT; ++i) rawInputs[SENSOR_PINS[i]] = shiftedBackground[i];
        step();
        assert(!modesStatus().armed && !modesStatus().running);
        assert(std::string(modesStatus().stopReason) == "RIGHT_LINE_LOST");
        expectStopped();
        setPattern("00100"); warmup(); expectStopped();

        startRightLine();
        for (unsigned long elapsed = CONTROL_PERIOD_MS; elapsed < RIGHT_LINE_TEST_MAX_RUN_MS;
             elapsed += CONTROL_PERIOD_MS) {
            if (elapsed == 1000) send("GO\n");
            step();
        }
        assert(!modesStatus().armed && !modesStatus().running);
        assert(std::string(modesStatus().stopReason) == "RUN_TIMEOUT");
        expectStopped();
        return;
    }
    if (scenario == "system-tuning") {
        setPattern("00100");
        send("MODE LINE_FOLLOW\nARM\n"); warmup(); send("GO\n");
        assert(!modesStatus().running && !lineCalibrationApplied());
        expectStopped();
        send("CAL=200,200,200,200,200/201,201,201,201,201/200,200,200,200,200\n");
        assert(!lineCalibrationApplied());
        applyCalibration();
        send("KP=20\nKD=0\nSPD=75\nTRIM=3\nX=-2,-1,0,1,2\nFF=35,30,33,30/25,25,25,25\nLIMIT=700\n");
        startFollow();
        send("KP=40\nSPD=120\nTRIM=31\nSAVE\n");
        assert(tuningParameters().speed == 75 && tuningParameters().kp == 20 && tuningParameters().trim == 3);
        assert(Serial.output.find("ERR STOP before changing settings") != std::string::npos);
        assert(Serial.output.find("ERR STOP before SAVE") != std::string::npos);
        assert(Serial.output.find("LINE_CFG FW=") != std::string::npos);
        assert(Serial.output.find("LINE_CAL INVERT=") != std::string::npos);
        send("STOP\n"); expectStopped();
        send("SPD=50\nTRIM=0\nKP=nan\nKD=-1\nGET\nCAL?\n");
        assert(tuningParameters().speed == 50 && tuningParameters().kp == 20 && tuningParameters().kd == 0);
        assert(Serial.output.find("KP=20 KI=0 KD=0 SPD=50 MAX=") != std::string::npos);
        startFollow();
        send("!\n"); expectStopped();
        return;
    }
    if (scenario == "system-gates") {
        send("MODE LINE_FOLLOW\nARM\n"); warmup(); send("RUN\n");
        assert(!modesStatus().running);
        expectStopped();
        if (sensorsIsReady()) {
            assert(modesStatus().sensors.rawAvailable);
            if (!SENSOR_CALIBRATION_CONFIRMED) assert(!modesStatus().sensors.valid);
        }
        return;
    }
    if (scenario == "system-auto") {
        assert(LINE_AUTO_RESUME && !AUTO_START_ON_BOOT);
        setPattern("00000");
        send("MODE LINE_FOLLOW\nRUN\n"); // Still needs ARM.
        assert(!modesStatus().running && !modesStatus().armed); expectStopped();
        send("ARM\nRUN\n"); // Off the line: starts paused, never moves blindly.
        assert(modesStatus().running && modesStatus().paused && modesStatus().armed);
        for (int i = 0; i < 5; ++i) { step(); expectStopped(); }
        setPattern("00100");
        for (unsigned int i = 1; i < LINE_START_VALID_SAMPLES; ++i) { step(); expectStopped(); }
        step(); expectSpeeds(BASE_SPEED, BASE_SPEED); assert(!modesStatus().paused);
        for (const char* bits : {"00000", "11111", "10001", "01010"}) {
            setPattern(bits); step();
            assert(modesStatus().running && modesStatus().paused && modesStatus().armed);
            expectStopped();
            step(); expectStopped();
            setPattern("00100"); warmup(); // Auto-resume after a stable line.
            expectSpeeds(BASE_SPEED, BASE_SPEED); assert(!modesStatus().paused);
        }
        for (unsigned long t = 0; t < LINE_FOLLOW_MAX_RUN_MS + 1000; t += CONTROL_PERIOD_MS) step();
        assert(modesStatus().running); expectSpeeds(BASE_SPEED, BASE_SPEED); // No RUN_TIMEOUT.
        step(CONTROL_MAX_GAP_MS + 1); // Stale control still latches.
        assert(!modesStatus().running && std::string(modesStatus().stopReason) == "STALE_CONTROL");
        expectStopped();
        send("ARM\nRUN\n"); warmup(); expectSpeeds(BASE_SPEED, BASE_SPEED);
        rawInputs[SENSOR_PINS[0]] = SENSOR_INPUT_MODE == SensorInputMode::ANALOG_AO ? 5000 : 2;
        step(); // ADC/raw fault never auto-resumes.
        assert(!modesStatus().running && !modesStatus().armed);
        assert(std::string(modesStatus().stopReason) == "SENSOR_FAULT"); expectStopped();
        setPattern("00100"); warmup(); expectStopped();
        send("ARM\nRUN\n"); warmup(); expectSpeeds(BASE_SPEED, BASE_SPEED);
        send("STOP\n"); expectStopped();
        assert(!modesStatus().running && modesStatus().armed);
        warmup(); expectStopped(); // STOP is final until a new RUN.
        send("RUN\n"); warmup(); expectSpeeds(BASE_SPEED, BASE_SPEED);
        send("!"); expectStopped(); assert(!modesStatus().armed); send("\n");
        warmup(); expectStopped();
        return;
    }
    if (scenario == "system-autostart" || scenario == "system-autostart-cancel") {
        assert(AUTO_START_ON_BOOT && LINE_AUTO_RESUME);
        expectStopped(); assert(!modesStatus().armed);
        if (scenario == "system-autostart-cancel") {
            send("\n");
            step(AUTO_START_DELAY_MS * 2); warmup();
            assert(!modesStatus().armed && modesStatus().mode == RobotMode::MOTOR_TEST);
            expectStopped();
            return;
        }
        step(AUTO_START_DELAY_MS - 1);
        assert(!modesStatus().armed); expectStopped();
        step(1);
        assert(modesStatus().mode == RobotMode::LINE_FOLLOW && modesStatus().armed &&
               modesStatus().running && modesStatus().paused);
        expectStopped();
        warmup(); expectSpeeds(BASE_SPEED, BASE_SPEED);
        send("STOP\n"); expectStopped();
        for (int i = 0; i < 300; ++i) step(); // Never re-triggers after STOP.
        assert(!modesStatus().running); expectStopped();
        return;
    }
    if (scenario == "system-button" || scenario == "system-held-button") {
        assert(MODE_BUTTON_ENABLED);
        if (scenario == "system-held-button") {
            step(MODE_BUTTON_DEBOUNCE_MS + 1);
            assert(modesStatus().mode == RobotMode::MOTOR_TEST);
            rawInputs[MODE_BUTTON_PIN] = HIGH;
            step(); step(MODE_BUTTON_DEBOUNCE_MS);
        }
        sendArmed("LEFT 30\n"); expectSpeeds(30, 0);
        rawInputs[MODE_BUTTON_PIN] = LOW; step(1);
        rawInputs[MODE_BUTTON_PIN] = HIGH; step(1); // bounce
        rawInputs[MODE_BUTTON_PIN] = LOW; step(1);
        assert(modesStatus().mode == RobotMode::MOTOR_TEST);
        step(MODE_BUTTON_DEBOUNCE_MS);
        assert(modesStatus().mode == RobotMode::SENSOR_TEST);
        assert(!modesStatus().armed);
        expectStopped();
        step(100); assert(modesStatus().mode == RobotMode::SENSOR_TEST);
        rawInputs[MODE_BUTTON_PIN] = HIGH; step(); step(MODE_BUTTON_DEBOUNCE_MS);
        rawInputs[MODE_BUTTON_PIN] = LOW; step(); step(MODE_BUTTON_DEBOUNCE_MS);
        assert(modesStatus().mode == RobotMode::LINE_FOLLOW && !modesStatus().running);
        expectStopped();
        return;
    }
    applyCalibration();
    startFollow();
    const auto checkSteer = [](const char* bits, int direction) {
        setPattern(bits);
        for (int i = 0; i < 12; ++i) followTick();
        const auto& robot = modesStatus();
        assert(robot.running && direction * (robot.leftCommand - robot.rightCommand) > 0);
        expectSpeeds(robot.leftCommand, robot.rightCommand);
    };
    const int rightSign = INVERT_PID_ERROR ? -1 : 1;
    checkSteer("00010", rightSign);
    checkSteer("01000", -rightSign);
    checkSteer("00001", rightSign);
    const int sharpL = modesStatus().leftCommand, sharpR = modesStatus().rightCommand;
    assert(std::min(sharpL, sharpR) <= 0 && std::min(sharpL, sharpR) >= -LINE_REVERSE_PWM_LIMIT);
    checkSteer("10000", -rightSign);
    expectSpeeds(sharpR, sharpL);
    send("MODE SENSOR_TEST\n"); expectStopped();
    send("ARM\nRUN\nLEFT 30\n"); expectStopped();
    assert(!modesStatus().armed && !modesStatus().running && modesStatus().mode == RobotMode::SENSOR_TEST);
    for (const std::string bits : {"00000", "10000", "01000", "00100", "00010",
                                  "00001", "01100", "11111", "00000"}) {
        setPattern(bits);
        Serial.output.clear();
        step(179); assert(Serial.output.empty());
        step(1);
        if (SENSOR_INPUT_MODE == SensorInputMode::ANALOG_AO) {
            assert(Serial.output.find("RAW:[") == 0 && Serial.output.find(" | THR:[") != std::string::npos);
        } else {
            assert(Serial.output.find("DIG:[") == 0);
        }
        assert(Serial.output.find(" | BIN:" + bits + " | ACT:" +
                                  std::to_string(std::count(bits.begin(), bits.end(), '1')) +
                                  "/5 | CLASS:UNVERIFIED") != std::string::npos);
        expectStopped();
    }

    for (const std::string bits : {"00000", "11111", "10001", "01010"}) {
        startFollow();
        Serial.output.clear();
        setPattern(bits); followTick();
        const bool missing = bits == "00000";
        const unsigned limit = (missing ? LINE_LOSS_LIMIT_US : LINE_AMBIGUITY_LIMIT_US) / LINE_PERIOD_US;
        if (!missing) expectStopped(); // Ambiguous frames cancel output on the first sample.
        for (unsigned elapsed = 1; elapsed < limit; ++elapsed) {
            followTick(); assert(modesStatus().running);
            if (missing && elapsed == 3) expectSpeeds(-LINE_SEARCH_PWM_LIMIT, LINE_SEARCH_PWM_LIMIT);
            if (missing && elapsed == 12) expectSpeeds(LINE_SEARCH_PWM_LIMIT, -LINE_SEARCH_PWM_LIMIT);
        }
        followTick();
        assert(!modesStatus().running && !modesStatus().armed);
        expectStopped();
        assert(std::string(modesStatus().stopReason) == (missing ? "LINE_LOST_TIMEOUT" : "LINE_AMBIGUOUS"));
        assert(Serial.output.find("LINE_STATS FW=") != std::string::npos);
        assert(Serial.output.find(missing ? "ZERO_EP=1" : "ZERO_EP=0") != std::string::npos);
        assert(Serial.output.find(missing ? "MAX_ZERO_US=300000" : "MAX_ZERO_US=0") != std::string::npos);
        assert(Serial.output.find("LINE_TIMING N=") != std::string::npos);
        assert(Serial.output.find("BITS=" + bits + " RAW_OK=1") != std::string::npos);
        setPattern("00100"); warmup();
        assert(!modesStatus().running); expectStopped();
    }
    // Recent two-frame directions survive crossing the center before losing the line.
    for (const int direction : {-1, 1}) {
        startFollow();
        setPattern(direction < 0 ? "01000" : "00010"); followTick(); followTick();
        setPattern("00100"); followTick();
        setPattern("00000"); for (int i = 0; i < 4; ++i) followTick();
        expectSpeeds(direction * rightSign * LINE_SEARCH_PWM_LIMIT, -direction * rightSign * LINE_SEARCH_PWM_LIMIT);
        setPattern("00100"); followTick(); expectStopped();
        for (unsigned i = 1; i < LINE_REACQUIRE_SAMPLES + 22; ++i) followTick();
        assert(modesStatus().running && modesStatus().error == 0);
        send("STOP\n"); expectStopped();
        assert(Serial.output.find("RECOVERIES=1") != std::string::npos);
    }
    startFollow();
    rawInputs[SENSOR_PINS[0]] = SENSOR_INPUT_MODE == SensorInputMode::ANALOG_AO ? 5000 : 2;
    followTick(); assert(!modesStatus().running); expectStopped();
    assert(std::string(modesStatus().stopReason) == "SENSOR_FAULT");
    startFollow(); step(LINE_STALE_US / 1000);
    assert(std::string(modesStatus().stopReason) == "STALE_CONTROL"); expectStopped();
    nowMs = (1ULL << 32) / 1000 - 100;
    startFollow(); // Cross the 32-bit micros rollover.
    assert(modesStatus().running);
    Serial.txAvailable = 0;
    for (int i = 0; i < 8; ++i) followTick();
    assert(modesStatus().running);
    send("!"); expectStopped();
    assert(!modesStatus().armed && Serial.output.rfind("TX_DROP=0") != Serial.output.rfind("TX_DROP="));
    Serial.txAvailable = 1024;
    send("\nLIMIT=500\n");
    startFollow();
    while (modesStatus().running) { send("RUN\n"); followTick(); }
    assert(std::string(modesStatus().stopReason) == "RUN_TIMEOUT"); expectStopped();
    startFollow(); send("DISARM\n"); expectStopped();
    warmup(); send("RUN\n");
    assert(!modesStatus().running && !modesStatus().armed);

    if (BOOT_START_BUTTON_ENABLED) {
        const auto pressBoot = [] {
            rawInputs[BOOT_START_BUTTON_PIN] = LOW;
            followTick(); step(MODE_BUTTON_DEBOUNCE_MS);
        };
        const auto releaseBoot = [] {
            rawInputs[BOOT_START_BUTTON_PIN] = HIGH;
            followTick(); step(MODE_BUTTON_DEBOUNCE_MS);
        };
        pressBoot(); releaseBoot();
        assert(modesCountdownActive() && modesStatus().armed && !modesStatus().running);
        send("STOP\n"); expectStopped();
        step(BOOT_START_COUNTDOWN_MS);
        assert(!modesCountdownActive() && !modesStatus().running);
        setPattern("00000");
        pressBoot(); releaseBoot();
        assert(modesCountdownActive());
        step(BOOT_START_COUNTDOWN_MS);
        assert(!modesStatus().running && !modesStatus().armed); expectStopped();
        setPattern("00100");
        pressBoot(); releaseBoot();
        for (unsigned t = 0; t < BOOT_START_COUNTDOWN_MS / FOLLOW_MS; ++t) followTick();
        assert(!modesCountdownActive() && modesStatus().running); expectStopped();
        for (int i = 0; i < 8; ++i) followTick();
        rawInputs[BOOT_START_BUTTON_PIN] = LOW; followTick();
        expectStopped();
        assert(std::string(modesStatus().stopReason) == "BOOT_STOP");
        releaseBoot();
        assert(!modesCountdownActive() && !modesStatus().running && !modesStatus().armed);
    }
    send("MODE MOTOR_TEST\nARM\nRIGHT -30\n"); expectSpeeds(0, -30);
    send("MODE SENSOR_TEST\n"); expectStopped();
}

void runArmingChecks() {
    assert(motorIsReady() && !modesStatus().armed && !modesStatus().running);
    send("LEFT 50\nRIGHT 50\nBOTH 50\nSET 100 60\nRUN\n");
    expectStopped();
    assert(!modesStatus().armed);

    send("ARM\n"); assert(modesStatus().armed); expectStopped();
    send("LEFT 50\n"); expectSpeeds(50, 0);
    send("RIGHT 50\n"); expectSpeeds(0, 50);
    send("BOTH 50\n"); expectSpeeds(50, 50);
    send("SET 100 60\n"); expectSpeeds(100, 60);
    send("LEFT -50\n"); expectSpeeds(-50, 0);
    send("RIGHT -50\n"); expectSpeeds(0, -50);
    send("BOTH -50\n"); expectSpeeds(-50, -50);
    send("SET -50 50\n"); expectSpeeds(-50, 50);
    send("STOP\n"); expectStopped(); assert(modesStatus().armed);
    send("LEFT 30\n"); expectSpeeds(30, 0); // STOP never restarts, but retains authorization.
    send("ARM\n"); expectStopped(); assert(modesStatus().armed); // Re-ARM stops first.
    send("SET 0 30\n"); expectSpeeds(0, 30);
    send("DISARM\n"); expectStopped(); assert(!modesStatus().armed);
    send("BOTH 50\n"); expectStopped(); assert(!modesStatus().armed);

    sendArmed("BOTH 30\n");
    step(MOTOR_TEST_TIMEOUT_MS);
    expectStopped(); assert(!modesStatus().armed);
    send("RIGHT 30\n"); expectStopped();

    const std::string overSpeed = std::to_string(MAX_SPEED + 1);
    const std::string invalid[] = {"ARM extra\n", "SET 30\n", "SET 30 20 extra\n",
        "SET " + overSpeed + " 20\n", "SET 20 -" + overSpeed + "\n", "SET abc 20\n", "SET 30 2.5\n",
        "SET 30 999999999999999999999999\n", "BOTH " + overSpeed + "\n", "STOP extra\n", "DISARM extra\n"};
    for (const auto& bad : invalid) {
        sendArmed("LEFT 30\n");
        send(bad); expectStopped(); assert(!modesStatus().armed);
    }
    sendArmed("BOTH 30\n"); send("!"); expectStopped(); assert(!modesStatus().armed);
    send("ARM\n"); assert(!modesStatus().armed); // Until Enter, discard the interrupted line.
    send("ARM\n"); assert(modesStatus().armed); expectStopped();
    send("SET 0 0\n"); expectStopped();
    send("MODE SENSOR_TEST\n"); expectStopped(); assert(!modesStatus().armed);
    send("ARM\nLEFT 50\n"); expectStopped(); assert(!modesStatus().armed);
    send("MODE LINE_FOLLOW\nRUN\n"); expectStopped(); assert(!modesStatus().armed);
    send("MODE MOTOR_TEST\nARM\nLEFT 30\n"); expectSpeeds(30, 0);
    modesInit(nowMs); expectStopped(); assert(!modesStatus().armed); // Startup-equivalent init.
    send("STATUS\n");
    assert(Serial.output.find("ARMED=0") != std::string::npos);
}
