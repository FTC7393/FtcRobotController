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
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.ServoName;
import ftc.evlib.hardware.servos.Servos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;
//Find max speed of robot.
public class GameChangersRobotCfg extends RobotCfg {

    //private final TwoMotors twoMotors;
    private final Motor collector;
    private final MecanumControl mecanumControl;
    private final Velocity velocity = new Velocity(Distance.fromInches(12), Time.fromSeconds(1.0));
    private final Servos servos;
    private final WobbleGoalCollector wobbleGoal;

    public GameChangersRobotCfg(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);

        servos = new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));

        //Drive Train.
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        Motor lm =  Motors.withEncoder(leftMotor, true, true, stoppers);
        Motor rm =  Motors.withEncoder(rightMotor, false, true, stoppers);
        Motor blm =  Motors.withEncoder(backLeftMotor, true, true, stoppers);
        Motor brm =  Motors.withEncoder(backRightMotor, false, true, stoppers);
        // twoMotors = new TwoMotors(lm, rm, true, velocity);
        mecanumControl = new MecanumControl(new MecanumMotors(lm,rm,blm,brm,true,velocity,velocity));

        //Collector Stoof (Peter is working on it)
        DcMotor leftCollector = hardwareMap.get(DcMotor.class, "leftCollector");
        DcMotor rightCollector = hardwareMap.get(DcMotor.class, "rightCollector");
        Motor lc =  Motors.withEncoder(leftCollector, true, true, stoppers);
        Motor rc =  Motors.withEncoder(rightCollector, false, true, stoppers);
        collector = Motors.combinedWithoutEncoder(lc,rc);

        //Wobble Goal Collector Stouf
        Motor rotator =  Motors.withEncoder(hardwareMap.get(DcMotor.class, "WobbleCollectorMotor"), false, true, stoppers);
        ServoControl pinchServo = getPincher();

        wobbleGoal = new WobbleGoalCollector(rotator, pinchServo, WobblePincherServoPresets.CLOSED,WobblePincherServoPresets.OPENED);
    }
    //Servo Stuof
    public GameChangersRobotCfg(HardwareMap hardwareMap) {
        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(GameChangersServoName.values()));
    }

    public enum WobblePincherServoPresets {
        CLOSED,
        OPENED
    }

    public ServoControl getPincher() {
        return getServo(GameChangersServoName.PINCH_SERVO);
    }

    public Servos getServos(){
        return servos;
    }

    public enum GameChangersServoName implements ServoName {
        //enum name("hardware name", preset enum.values()),
        PINCH_SERVO("pinchServo", WobblePincherServoPresets.values());

//        PUSH_SERVO("pushServo", RotateServoPresets.values());

        private final String hardwareName;
        private final Enum[] presets;

        GameChangersServoName(String hardwareName, Enum[] presets) {
            this.hardwareName = hardwareName;
            this.presets = presets;
        }

        @Override
        public String getHardwareName() {
            return hardwareName;
        }

        @Override
        public Enum[] getPresets() {
            return presets;
        }

        @Override
        public Class<GameChangersRobotCfg> getRobotCfg() {
            return GameChangersRobotCfg.class;
        }
    }

    @Override
    public void start() {

    }

    @Override
    public void act() {
        //twoMotors.update();
        collector.update();
        mecanumControl.act();
    }

    @Override
    public void stop() {
        //twoMotors.stop();
        mecanumControl.stop();
    }


}
