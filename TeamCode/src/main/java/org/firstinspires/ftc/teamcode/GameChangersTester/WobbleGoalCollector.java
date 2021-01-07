package org.firstinspires.ftc.teamcode.GameChangersTester;

import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.sensors.AnalogSensor;
import ftc.evlib.hardware.servos.ServoControl;

public class WobbleGoalCollector {

    private final Motor rotation;
    private final ServoControl pinch;
    private Enum close;
    private Enum open;
    private AnalogSensor potentiometer;

    public WobbleGoalCollector(Motor rotation, ServoControl pinch, Enum close, Enum open, AnalogSensor potentiometer) {
        this.rotation = rotation;
        this.pinch = pinch;
        this.close = close;
        this.open = open;
        this.potentiometer = potentiometer;
    }

    public void close() {
        pinch.goToPreset(close);
    }
    public void open() {
        pinch.goToPreset(open);
    }


    public void act() {
    }

    public void stop() {
    }
}
