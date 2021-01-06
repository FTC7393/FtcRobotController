package org.firstinspires.ftc.teamcode.GameChangersTester;

import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.servos.ServoControl;

public class WobbleGoalCollector {

    private final Motor rotation;
    private final ServoControl pinch;
    private Enum close;
    private Enum open;
    public WobbleGoalCollector(Motor rotation, ServoControl pinch, Enum close, Enum open) {
        this.rotation = rotation;
        this.pinch = pinch;
        this.close = close;
        this.open = open;
    }

    public void close() {
        pinch.goToPreset(close);
    }
    public void open() {
        pinch.goToPreset(open);
    }



}
