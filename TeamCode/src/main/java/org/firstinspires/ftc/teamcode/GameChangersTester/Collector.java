package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.robotcore.hardware.CRServo;

import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.motors.Motors;

public class Collector {
    private Motor collectorMotor;
    private Motor collectorServo;
    final double motorPower = 1;
    final double servoPower = 1;
    final double motorPowerBack = -1;
    final double servoPowerBack = -1;
    final double motorStop = 0;
    final double servoStop = 0;

    public Collector(Motor collectorMotor, CRServo collectorServo)
    {
        this.collectorMotor = collectorMotor;
        this.collectorServo = Motors.continuousServo(collectorServo, false);
    }

    public void ingest()
    {
        collectorMotor.setPower(motorPower);
        collectorServo.setPower(servoPower);
    }

    public void expel()
    {
        collectorMotor.setPower(motorPowerBack);
        collectorServo.setPower(servoPowerBack);
    }

    public void stop()
    {
        collectorMotor.setPower(motorStop);
        collectorServo.setPower(servoStop);
        act();
    }

    public void act()
    {
        collectorServo.update();
        collectorMotor.update();
    }

    //Cause this sucks!
    public void Suck()
    {
        ingest();
    }
    //and blows too!
    public void Spew()
    {
        expel();
    }

}
