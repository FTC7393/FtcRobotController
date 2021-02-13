package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class BlinkyControl {
    private final RevBlinkinLedDriver blinky;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern;
    private RevBlinkinLedDriver.BlinkinPattern newPattern;

    public BlinkyControl(RevBlinkinLedDriver blinky, RevBlinkinLedDriver.BlinkinPattern startPattern) {
        this.blinky = blinky;
        currentPattern = startPattern;
        newPattern = startPattern;
    }

    public void setNewPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        newPattern = pattern;
    }

    public void update() {
        if (newPattern != currentPattern) {
            blinky.setPattern(newPattern);
            currentPattern = newPattern;
        }
    }
}