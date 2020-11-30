package org.firstinspires.ftc.teamcode.GameChangersTester;

import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.servos.ServoControl;

public class FlyWheelShooter {

    private final Motor flywheelMotor;
    private final ServoControl elevationServo;
    private final ServoControl pusherServo;
    private double power;


    public FlyWheelShooter(Motor flywheelMotor, ServoControl elevationServo, ServoControl pusherServo, double power) {
        this.flywheelMotor = flywheelMotor;
        this.elevationServo = elevationServo;
        this.pusherServo = pusherServo;
        this.power = power;
    }

    public void turnOnFlywheel() {
        flywheelMotor.setPower(power);
    }

    public void reverseFlywheel() {
        flywheelMotor.setPower(-power);
    }

    public void stopFlywheel() {
        flywheelMotor.setPower(0);
    }

    public void goToShootingAngle() {
        Enum shootingPreset = null;
        elevationServo.goToPreset(shootingPreset);
    }

    public void ejectRing() {
        Enum pushingPreset = null;
        pusherServo.goToPreset(pushingPreset);
    }

}
