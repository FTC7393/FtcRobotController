package org.firstinspires.ftc.teamcode.GameChangersTester;

import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.servos.ServoControl;

public class FlyWheelShooter {

    private final Motor flywheelMotor;
    private final ServoControl elevationServo;
    private final ServoControl pusherServo;
    private final double maxPower;
    private double targetPower;
    private final double rampRate;
    double currentPower;
    private double lastTimeStamp;


    public FlyWheelShooter(Motor flywheelMotor, ServoControl elevationServo, ServoControl pusherServo, double maxPower,
                           double rampRate) {
        this.flywheelMotor = flywheelMotor;
        this.elevationServo = elevationServo;
        this.pusherServo = pusherServo;
        this.maxPower = maxPower;
        this.rampRate = rampRate;
        targetPower = 0;
        currentPower = 0;
        lastTimeStamp = System.currentTimeMillis();
    }

    public void turnOnFlywheel() {
        targetPower = maxPower;
    }

    public void reverseFlywheel() {
        targetPower = -maxPower;
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

    public void stop() {
        flywheelMotor.setPower(0);
    }

    public void act() {
        double currentTimeStamp = System.currentTimeMillis();
        if(currentPower != targetPower) {
            double cycleTime = currentTimeStamp - lastTimeStamp;
            double powerDelta = rampRate*cycleTime;
            if(currentPower < targetPower) {
                if(currentPower+powerDelta > targetPower) {
                    currentPower = targetPower;
                } else {
                    currentPower += (powerDelta);
                }
            } else if(currentPower > targetPower) {
                if(currentPower - powerDelta < targetPower) {
                    currentPower = targetPower;
                } else {
                    currentPower -= (powerDelta);
                }
            }
            flywheelMotor.setPower(currentPower);
        }

        lastTimeStamp = currentTimeStamp;
    }

}
