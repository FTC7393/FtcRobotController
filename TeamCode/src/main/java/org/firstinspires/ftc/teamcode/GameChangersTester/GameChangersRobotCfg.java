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

    private final TwoMotors twoMotors;
    private final Velocity velocity = new Velocity(Distance.fromInches(12), Time.fromSeconds(1.0));

    public TwoMotors getTwoMotors() {
        return twoMotors;
    }

    public GameChangersRobotCfg(HardwareMap hardwareMap) {
        super(hardwareMap);
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        Motor lm =  Motors.withEncoder(leftMotor, false, true, stoppers);
        Motor rm =  Motors.withEncoder(rightMotor, true, true, stoppers);
        twoMotors = new TwoMotors(lm, rm, true, velocity);


    }


    @Override
    public void start() {

    }

    @Override
    public void act() {
        twoMotors.update();
    }

    @Override
    public void stop() {
        twoMotors.stop();
    }


}
