package org.firstinspires.ftc.teamcode.GameChangersTester;

import ftc.electronvolts.util.PIDController;
import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.sensors.AnalogSensor;
import ftc.evlib.hardware.servos.ServoControl;

public class WobbleGoalCollector {

    private final Motor rotation;
    private final ServoControl pinch;
    private Preset close;
    private Preset open;
    private AnalogSensor potentiometer;
    private PIDController pidController;


    //positions
    private final double upPosition = 0.84;
    private final double downPosition = 0.3;
    private final double moreDownPosition = 0.2;
    private final double maxPosition = 0.9;
    private final double minPosition = 0.1;

    public double getTargetPosition() {
        return targetPosition;
    }

    private double targetPosition = upPosition;
    private double currentPosition = 0;
    private double rotationPower = 0;


    public PIDController getPidController() {
        return pidController;
    }

    public WobbleGoalCollector(Motor rotation, ServoControl pinch, Preset close, Preset open, AnalogSensor potentiometer) {
        this.rotation = rotation;
        this.pinch = pinch;
        this.close = close;
        this.open = open;
        this.potentiometer = potentiometer;
        // NEED TO TUNE IT!!!!!!!!!!!!!!!!!!!!!!!!!
        pidController = new PIDController(4,0,0,0.5); // original iGain: 0.002
    }

    public void close() {
        pinch.goToPreset((Enum) close);
    }
    public void open() {
        pinch.goToPreset((Enum) open);
    }

    public void moveArmUp() {
        targetPosition = upPosition;
    }

    public void moveArmDown(){
        targetPosition = downPosition;
    }

    public void moveArmMoreDown(){
        targetPosition = moreDownPosition;
    }

    public void act() {
        if(targetPosition > maxPosition){
            targetPosition = maxPosition;
        } else if(targetPosition < minPosition) {
            targetPosition = minPosition;
        }

        currentPosition = potentiometer.getValue();
        rotationPower = -pidController.computeCorrection(targetPosition, currentPosition);
        rotation.setPower(rotationPower);
        rotation.update();
    }

    public void stop() {
        targetPosition = currentPosition;
    }
}
