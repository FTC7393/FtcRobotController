package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.motors.FourMotors;
import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.motors.MotorEncEx;
import ftc.evlib.hardware.motors.TwoMotors;
import ftc.evlib.hardware.sensors.AnalogSensor;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.hardware.sensors.IMUGyro;
import ftc.evlib.hardware.sensors.Sensors;
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

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.Map;
//Find max speed of robot.
public class GameChangersRobotCfg extends RobotCfg {

    //private final TwoMotors twoMotors;
    private final Collector collector;
    private final MecanumControl mecanumControl;
    private static final Velocity velocity = new Velocity(Distance.fromInches(12), Time.fromSeconds(1.0));
    private final Servos servos;
    private final WobbleGoalCollector wobbleGoal;
    private final IMUGyro gyro;
    private final AnalogSensor potentiometer;
    private final FlyWheelShooter flyWheelShooter;
    private final double flyWheelMotorSpeed = 1;
//    private final BlinkyControl blinkin;



    private final WebcamName webcam;
    private final RevBlinkinLedDriver blinkin;

    public AnalogSensor getPotentiometer() {
        return potentiometer;
    }

    public WebcamName getWebcamName() {
        return webcam;
    }

    public GameChangersRobotCfg(HardwareMap hardwareMap, Velocity velocityX, Velocity velocityY) {
        super(hardwareMap);

        Map<ServoName, Enum> servoStartPresetMap = ServoCfg.defaultServoStartPresetMap(GameChangersServoName.values());

        servos = new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));

        //Drive Train.
        DcMotor leftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotor rightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        Motor lm =  Motors.withEncoder(leftMotor, true, true, stoppers);
        Motor rm =  Motors.withEncoder(rightMotor, false, true, stoppers);
        Motor blm =  Motors.withEncoder(backLeftMotor, true, true, stoppers);
        Motor brm =  Motors.withEncoder(backRightMotor, false, true, stoppers);
        // twoMotors = new TwoMotors(lm, rm, true, velocity);
        mecanumControl = new MecanumControl(new MecanumMotors(lm,rm,blm,brm,true,velocityX,velocityY));

        //Collector Stoof
        DcMotor leftCollector = hardwareMap.get(DcMotorEx.class, "collectorMotor");
        ServoControl windmill = getServo(GameChangersServoName.COLLECTOR);

        Motor lc =  Motors.withEncoder(leftCollector, false, false, stoppers);
        collector = new Collector(lc, windmill);

        // gyro
        gyro = new IMUGyro(hardwareMap.get(BNO055IMU.class, "imu0"));

        //Wobble Goal Collector Stouf
        Motor rotator =  Motors.withoutEncoder(hardwareMap.get(DcMotorEx.class, "wobbleGoalArmMotor"), false, true, stoppers);
        ServoControl pinchServo = getPincher();

        potentiometer = Sensors.analogLinear(hardwareMap, "potentiometer");
        wobbleGoal = new WobbleGoalCollector(rotator, pinchServo, ServoPresets.WobblePincher.CLOSED,ServoPresets.WobblePincher.OPENED, potentiometer);

        //shooter
        MotorEncEx flyWheelmotor = Motors.withEncoder(hardwareMap.get(DcMotorEx.class, "flyWheelShooter"), false, false, stoppers);
        flyWheelShooter = new FlyWheelShooter(flyWheelmotor, getElevation(), getPusher(),1,.1);
        //camera
        webcam = hardwareMap.get(WebcamName.class, "Webcam");

        //Blinkin thing.
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");


    }
    //Servo Stuof
    public GameChangersRobotCfg(HardwareMap hardwareMap) {
        this(hardwareMap, velocity, velocity);
    }

    public FlyWheelShooter getFlyWheelShooter() { return flyWheelShooter; }

    public ServoControl getPincher() {
        return getServo(GameChangersServoName.PINCH);
    }

    public Collector getCollector() {
        return collector;
    }

    public ServoControl getPusher() {
        return getServo(GameChangersServoName.PUSHER);
    }

    public ServoControl getElevation() {
        return getServo(GameChangersServoName.ELEVATION);
    }

    public ServoControl getCameraServo() {
        return getServo(GameChangersServoName.CAMERA);
    }

    public Servos getServos(){
        return servos;
    }

    public Gyro getGyro() {
        return gyro;
    }

    public MecanumControl getMecanumControl() {
        return mecanumControl;
    }

    public WobbleGoalCollector getWobbleGoalArm() {
        return wobbleGoal;
    }

    public RevBlinkinLedDriver getBlinkin() {
        return blinkin;
    }


    public enum GameChangersServoName implements ServoName {
        //enum name("hardware name", preset enum.values()),
        PINCH("pinchServo", ServoPresets.WobblePincher.values()),
        PUSHER("pusherServo", ServoPresets.Pusher.values()),
        ELEVATION("elevationServo", ServoPresets.Elevation.values()),
        COLLECTOR("collectorServo", ServoPresets.Collector.values()),
        CAMERA("cameraRotationServo", ServoPresets.Camera.values());

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
        collector.act();
        mecanumControl.act();
        wobbleGoal.act();
        flyWheelShooter.act();
    }

    @Override
    public void stop() {
        //twoMotors.stop();
        mecanumControl.stop();
        wobbleGoal.stop();
        collector.stop();
        flyWheelShooter.act();
    }


}
