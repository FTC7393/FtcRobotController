package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.motors.Motors;

public class Collector {
    private Motor collectorMotor;
    private Servo collectorServo;
    final double motorPower = 1;
    final double servoPower = 1;
    final double motorPowerBack = -1;
    final double servoPowerBack = 0;
    final double motorStop = 0;
    final double servoStop = 0.5;

    public Collector(Motor collectorMotor, Servo collectorServo)
    {
        this.collectorMotor = collectorMotor;
        this.collectorServo = collectorServo;
    }

    public void ingest()
    {
        collectorMotor.setPower(motorPower);
        collectorServo.setPosition(servoPower);
    }

    public void expel()
    {
        collectorMotor.setPower(motorPowerBack);
        collectorServo.setPosition(servoPowerBack);
    }

    public void stop()
    {
        collectorMotor.setPower(motorStop);
        collectorServo.setPosition(servoStop);
        act();
    }

    public void act()
    {
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
