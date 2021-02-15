package org.firstinspires.ftc.teamcode.GameChangersTester;

import ftc.electronvolts.util.PIDController;
import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.servos.ServoControl;

public class FlyWheelShooter {

    private final MotorEnc flywheelMotor;
    private final ServoControl elevationServo;
    private final ServoControl pusherServo;
    private final double shootingSpeed; //constant for how fast the shooter needs to be to function
    private double currentSpeed; //The latest measurement of flywheel speed
    private double finalTargetSpeed; //the speed that we set for the flywheel shooter to be
    private final double rampRate; //how fast it accelerates
    private double currentTargetSpeed; //the desired speed at this point in the acceleration ramp
    private double lastTimeStamp;
    private PIDController pidController;
    private double currentPower; //the power we want to set the flywheel shooter to
    private int lastEnc;
    private int currEnc;



    public FlyWheelShooter(MotorEnc flywheelMotor, ServoControl elevationServo, ServoControl pusherServo, double maxPower,
                           double rampRate) {
        this.flywheelMotor = flywheelMotor;
        this.elevationServo = elevationServo;
        this.pusherServo = pusherServo;
        this.shootingSpeed = maxPower;
        this.rampRate = rampRate;
        pidController = new PIDController(1,0,0,1);
        currentSpeed = 0;
        finalTargetSpeed = 0;
        currentTargetSpeed = 0;
        currentPower = 0;
        lastEnc = 0;
        currEnc = 0;
        lastTimeStamp = System.currentTimeMillis();
    }

    public void turnOnFlywheel() {
        finalTargetSpeed = shootingSpeed;
    }

    public void reverseFlywheel() {
        finalTargetSpeed = -shootingSpeed;
    }

    public void goToShootingAngle() {
        elevationServo.goToPreset(ServoPresets.Elevation.SHOOTING);
    }

    public void goToCollectionAngle() {
        elevationServo.goToPreset(ServoPresets.Elevation.COLLECTING);
    }

    /**
    this method pushes the ring into the shooter
     */
    public void engagePusher() {
        pusherServo.goToPreset(ServoPresets.Pusher.PUSH);
    }

    /**
     this method moves the servo out of the way to let the next ring fall into place
     */
    public void disengagePusher() {
        pusherServo.goToPreset(ServoPresets.Pusher.RELEASE);
    }

    public double getCurrentSpeed(){ return currentSpeed;}
    public double getCurrentTargetSpeed(){ return currentTargetSpeed;}
    public double getFinalTargetSpeed(){ return finalTargetSpeed;}
    public double getFlywheelEncoderValue(){ return currEnc;}

    public void stop() {
        flywheelMotor.setPower(0);
        currentTargetSpeed = 0;
        finalTargetSpeed = 0;
    }

    public void act() {
        double currentTimeStamp = System.currentTimeMillis();
        double cycleTime = currentTimeStamp - lastTimeStamp;
        if(currentTargetSpeed != finalTargetSpeed) {
            double speedDelta = rampRate*cycleTime;
            if(currentTargetSpeed < finalTargetSpeed) {
                if(currentTargetSpeed +speedDelta > finalTargetSpeed) {
                    currentTargetSpeed = finalTargetSpeed;
                } else {
                    currentTargetSpeed += (speedDelta);
                }
            } else if(currentTargetSpeed > finalTargetSpeed) {
                if(currentTargetSpeed - speedDelta < finalTargetSpeed) {
                    currentTargetSpeed = finalTargetSpeed;
                } else {
                    currentTargetSpeed -= (speedDelta);
                }
            }
        }

        currEnc = flywheelMotor.getEncoderPosition();
        int distance = currEnc - lastEnc;
        currentSpeed = distance/cycleTime;

        //normal operation code
        //currentPower = pidController.computeCorrection(currentTargetSpeed,currentSpeed);
        //flywheelMotor.setPower(currentPower);

        //Test 1
        if(finalTargetSpeed > 0) {
            currentPower = 1;
            flywheelMotor.setPower(currentPower);

            //Test 2
            //currentPower = 0.5;
            //flywheelMotor.setPower(currentPower);

            //Test 3
            //currentPower = 1;
            //flywheelMotor.setSpeed(currentPower);

            //Test 4
            //currentPower = 0.5;
            //flywheelMotor.setSpeed(currentPower);
        }

        lastTimeStamp = currentTimeStamp;
        lastEnc = currEnc;
        flywheelMotor.update();
    }
}
