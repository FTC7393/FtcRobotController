package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.robotcore.hardware.CRServo;

import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.servos.ServoControl;

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

    public void Ingest()
    {
        collectorMotor.setPower(motorPower);
        collectorServo.setPower(servoPower);
    }

    public void Expel()
    {
        collectorMotor.setPower(motorPowerBack);
        collectorServo.setPower(servoPowerBack);
    }

    public void Stop()
    {
        collectorMotor.setPower(motorStop);
        collectorServo.setPower(servoStop);
    }
    //Cause this sucks!
    public void Suck()
    {
        Ingest();
    }
    //and blows too!
    public void Spew()
    {
        Expel();
    }

}
