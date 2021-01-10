package org.firstinspires.ftc.teamcode.GameChangersTester;

import ftc.electronvolts.util.PIDController;
import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.sensors.AnalogSensor;
import ftc.evlib.hardware.servos.ServoControl;

public class WobbleGoalCollector {

    private final Motor rotation;
    private final ServoControl pinch;
    private Enum close;
    private Enum open;
    private AnalogSensor potentiometer;
    private PIDController pidController;


    //positions
    private final double upPosition = 0;
    private final double downPosition = 0;
    private final double maxPosition = 0;
    private final double minPosition = 0;
    private double targetPosition = 0;
    private double currentPosition = 0;
    private double rotationPower = 0;



    public WobbleGoalCollector(Motor rotation, ServoControl pinch, Enum close, Enum open, AnalogSensor potentiometer) {
        this.rotation = rotation;
        this.pinch = pinch;
        this.close = close;
        this.open = open;
        this.potentiometer = potentiometer;
        // NEED TO TUNE IT!!!!!!!!!!!!!!!!!!!!!!!!!
        pidController = new PIDController(0.05,0.1,0.1,0.3);
    }

    public void close() {
        pinch.goToPreset(close);
    }
    public void open() {
        pinch.goToPreset(open);
    }

    public void moveArmUp() {
        targetPosition = upPosition;
    }

    public void moveArmDown(){
        targetPosition = downPosition;
    }


    public void act() {
        currentPosition = potentiometer.getValue();
        rotationPower = pidController.computeCorrection(targetPosition, currentPosition);
        rotation.setPower(rotationPower);
        rotation.update();

        if(targetPosition > maxPosition){
            targetPosition = maxPosition;
        } else if(targetPosition < minPosition) {
            targetPosition = minPosition;
        }
    }

    public void stop() {
        targetPosition = currentPosition;
    }
}
