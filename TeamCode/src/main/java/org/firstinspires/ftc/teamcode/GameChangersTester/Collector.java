package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.servos.ServoControl;

public class Collector {
    private Motor collectorMotor;
    private ServoControl collectorServo;
    final double motorPower = 1;
    final double servoPower = 1;
    final double motorPowerBack = -1;
    final double servoPowerBack = 0;
    final double motorStop = 0;
    final double servoStop = 0.5;

    public Collector(Motor collectorMotor, ServoControl collectorServo)
    {
        this.collectorMotor = collectorMotor;
        this.collectorServo = collectorServo; // this is the collector servo
    }

    public void ingest()
    {
        collectorMotor.setPower(motorPower);
        collectorServo.goToPreset(ServoPresets.Collector.FORWARD);
    }
            //damages rings and possibly could break the windmill
    public void expel()
    {
        collectorMotor.setPower(motorPowerBack);
        collectorServo.setPosition(servoPowerBack);
    }

    public void stop()
    {
        collectorMotor.setPower(motorStop);
        collectorServo.goToPreset(ServoPresets.Collector.OFF);
        act();
    }

    public void act()
    {
        collectorMotor.update();
    }



}
