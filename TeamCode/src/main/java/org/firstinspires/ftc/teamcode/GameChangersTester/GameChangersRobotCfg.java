package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.motors.FourMotors;
import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.motors.TwoMotors;
import ftc.evlib.hardware.servos.ServoCfg;
import ftc.evlib.hardware.motors.Motors;

import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.motors.MecanumMotors;
import ftc.evlib.hardware.servos.ServoName;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

public class GameChangersRobotCfg extends RobotCfg {

    private final Velocity maxRobotSpeed = new Velocity(Distance.fromInches(12), Time.fromSeconds(1.0));
    private final Velocity maxRobotSpeedSideways = new Velocity(Distance.fromInches(12), Time.fromSeconds(1.0));
    private final MecanumControl mecanumControl;


    // testing out the webhook, this is a cool feature which will help monitor pushes and changes to code

    public GameChangersRobotCfg(HardwareMap hardwareMap) {
        super(hardwareMap);
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        Motor fr =  Motors.withEncoder(frontRight, true, true, stoppers);
        Motor fl =  Motors.withEncoder(frontLeft, false, true, stoppers);
        Motor br =  Motors.withEncoder(backRight, true, true, stoppers);
        Motor bl =  Motors.withEncoder(backLeft, false, true, stoppers);
        mecanumControl = new MecanumControl(new MecanumMotors(fl, fr, bl, br, true,maxRobotSpeed, maxRobotSpeedSideways ));
    }


    @Override
    public void start() {

    }

    @Override
    public void act() {
        mecanumControl.act();
    }

    @Override
    public void stop() {
        mecanumControl.stop();
    }


}
