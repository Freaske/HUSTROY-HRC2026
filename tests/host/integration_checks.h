#pragma once

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
    assert(forwardCorrectionLimit(140, 150, 100) == 10);
    const auto speeds = mixForwardSpeeds(50, 20, 150);
    assert(speeds.valid && speeds.left == 70 && speeds.right == 30);
    assert(mixForwardSpeeds(50, 1000, 150).left == 150);
    assert(mixForwardSpeeds(50, 1000, 150).right == 0);
    assert(!mixForwardSpeeds(50, std::numeric_limits<float>::quiet_NaN(), 150).valid);
}

void setPattern(const std::string& bits) {
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        const bool high = INVERT_LINE_SENSOR ? bits[i] != '1' : bits[i] == '1';
        rawInputs[SENSOR_PINS[i]] = SENSOR_INPUT_MODE == SensorInputMode::ANALOG_AO ?
                                   (high ? 3000 : 1000) : (high ? 1 : 0);
    }
}
void step(unsigned long milliseconds = CONTROL_PERIOD_MS) { nowMs += milliseconds; loop(); }
void warmup() { for (unsigned int i = 0; i < LINE_START_VALID_SAMPLES; ++i) step(); }
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
    step();
    expectSpeeds(BASE_SPEED, BASE_SPEED);
}

void runSystemChecks(const std::string& scenario) {
    expectStopped();
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
    startFollow();
    setPattern("00010"); step();
    const auto expected = mixForwardSpeeds(BASE_SPEED, INVERT_PID_ERROR ? -PID_KP : PID_KP, MAX_SPEED);
    expectSpeeds(expected.left, expected.right);
    setPattern("01000"); step();
    expectSpeeds(expected.right, expected.left);
    send("MODE SENSOR_TEST\n"); expectStopped();
    send("LEFT 30\n"); expectStopped(); // Wrong-mode motor command rejected.
    assert(!modesStatus().running && modesStatus().mode == RobotMode::SENSOR_TEST);
    Serial.output.clear();
    step(DEBUG_LOG_INTERVAL_MS - 1); assert(Serial.output.empty());
    step(1); assert(Serial.output.find("RAW=[") != std::string::npos);

    const std::string stopPatterns[] = {"00000", "11111", "10001", "01010"};
    for (const auto& bits : stopPatterns) {
        startFollow();
        setPattern(bits); step();
        assert(!modesStatus().running);
        assert(!modesStatus().armed);
        expectStopped();
        setPattern("00100"); warmup();
        assert(!modesStatus().running); expectStopped(); // No automatic recovery.
    }
    startFollow();
    rawInputs[SENSOR_PINS[0]] = SENSOR_INPUT_MODE == SensorInputMode::ANALOG_AO ? 5000 : 2;
    step(); assert(!modesStatus().running); expectStopped();
    startFollow(); step(CONTROL_MAX_GAP_MS + 1);
    assert(std::string(modesStatus().stopReason) == "STALE_CONTROL"); expectStopped();

    // millis rollover must not stall the scheduler or extend manual timeout.
    nowMs = ULONG_MAX - 100;
    startFollow();
    for (int i = 0; i < 10; ++i) step();
    assert(modesStatus().running);
    send("STOP\n"); expectStopped();

    startFollow();
    unsigned long elapsed = CONTROL_PERIOD_MS; // startFollow includes first tick.
    while (elapsed < LINE_FOLLOW_MAX_RUN_MS) {
        if (elapsed == 1000) send("RUN\n"); // Cannot extend trial.
        step(); elapsed += CONTROL_PERIOD_MS;
    }
    assert(!modesStatus().running && std::string(modesStatus().stopReason) == "RUN_TIMEOUT");
    expectStopped();
    startFollow(); send("DISARM\n"); expectStopped();
    assert(!modesStatus().armed);
    warmup(); send("RUN\n");
    assert(!modesStatus().running && !modesStatus().armed);
    startFollow(); send("!"); expectStopped(); send("\n");
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

    const std::string invalid[] = {"ARM extra\n", "SET 30\n", "SET 30 20 extra\n",
        "SET 151 20\n", "SET 20 -151\n", "SET abc 20\n", "SET 30 2.5\n",
        "SET 30 999999999999999999999999\n", "BOTH 151\n", "STOP extra\n", "DISARM extra\n"};
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
