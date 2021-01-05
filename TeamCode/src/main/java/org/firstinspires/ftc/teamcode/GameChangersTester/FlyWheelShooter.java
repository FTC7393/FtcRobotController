package org.firstinspires.ftc.teamcode.GameChangersTester;

import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.servos.ServoControl;

public class FlyWheelShooter {

    private final Motor flywheelMotor;
    private final ServoControl elevationServo;
    private final ServoControl pusherServo;
    private double targetPower;
    private final double rampRate;
    double currentPower;
    private double lastTimeStamp;


    public FlyWheelShooter(Motor flywheelMotor, ServoControl elevationServo, ServoControl pusherServo, double targetPower,
                           double rampRate) {
        this.flywheelMotor = flywheelMotor;
        this.elevationServo = elevationServo;
        this.pusherServo = pusherServo;
        this.targetPower = targetPower;
        this.rampRate = rampRate;
        lastTimeStamp = System.currentTimeMillis();
    }

    public void turnOnFlywheel() {
        flywheelMotor.setPower(targetPower);
    }

    public void reverseFlywheel() {
        flywheelMotor.setPower(-targetPower);
    }

    public void goToShootingAngle() {
        Enum shootingPreset = null;
        elevationServo.goToPreset(shootingPreset);
    }

    public void ejectRing() {
        Enum pushingPreset = null;
        pusherServo.goToPreset(pushingPreset);
    }

    public void stop() {
        flywheelMotor.setPower(0);
    }

    public void act() {
        double currentTimeStamp = System.currentTimeMillis();
        double cycleTime = currentTimeStamp - lastTimeStamp;
        if(currentPower < targetPower) {
            currentPower += (rampRate*cycleTime);
        }
        if(currentPower > targetPower) {
            currentPower -= (rampRate*cycleTime);
        }
        lastTimeStamp = currentTimeStamp;
    }

}
