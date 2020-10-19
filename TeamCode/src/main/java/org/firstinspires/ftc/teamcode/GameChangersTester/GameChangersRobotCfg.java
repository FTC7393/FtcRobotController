package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.robotcore.hardware.DcMotorEx;
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

public class GameChangersRobotCfg {

    public GameChangersRobotCfg(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
//        super(hardwareMap);
        double scaleFactor = 1.0;
        mecanumControl = new MecanumControl(new MecanumMotors(
                Motors.withEncoder(hardwareMap.get(DcMotorEx.class, "backLeft"), true, true, stoppers), // 0
                Motors.withEncoder(hardwareMap.get(DcMotorEx.class, "frontLeft"), false, true, stoppers), // 1

                Motors.scale(Motors.withEncoder(hardwareMap.get(DcMotorEx.class, "frontRight"), false, true, stoppers), scaleFactor), // 2
                Motors.scale(Motors.withEncoder(hardwareMap.get(DcMotorEx.class, "backRight"), true, true, stoppers), scaleFactor), // 3
                true, MAX_ROBOT_SPEED, MAX_ROBOT_SPEED_SIDEWAYS));

//        servos = new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));
//
//        gyro0 = new MRGyro(hardwareMap.get(ModernRoboticsI2cGyro.class, "mr0"));

//        DcMotorEx collectorMotor = hardwareMap.get(DcMotorEx.class,"collectorMotor");
//
//        blockCollector = new BlockCollector(
//                Motors.withoutEncoder(collectorMotor, false, false, stoppers), getBlockDetector()
//        );

    }

    public GameChangersRobotCfg(HardwareMap hardwareMap) {
        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(SkystoneServoName.values()));
    }

    private final MecanumControl mecanumControl;

    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(57 * 4), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));


}
