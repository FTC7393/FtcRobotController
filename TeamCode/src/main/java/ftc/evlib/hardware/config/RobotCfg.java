package ftc.evlib.hardware.config;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.PowerManager;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

import ftc.evlib.hardware.motors.Stoppers;
import ftc.evlib.hardware.motors.TwoMotors;
import ftc.evlib.hardware.sensors.Accelerometer;
import ftc.evlib.hardware.sensors.Sensors;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.ServoName;
import ftc.evlib.hardware.servos.Servos;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/12/16
 *
 * Minimum hardware that every robot has.
 * This class can be extended to add more hardware devices for each specific robot.
 * Then each subclass can be used by multiple OpModes. {@see ftc.evlib.opmodes.AbstractOp}
 *
 * @see Servos
 */
public abstract class RobotCfg {
    public static final int DEVICE_MODULE_RED_LED = 1;
    public static final int DEVICE_MODULE_BLUE_LED = 0;

    private final Context phoneContext;
    private final SensorManager phoneSensorManager;
    private final Sensor phoneAccelerometer;
    private final PowerManager phonePowerManager;
    private final PowerManager.WakeLock phoneWakeLock;
    private final Accelerometer accelerometer;
    protected final Stoppers stoppers = new Stoppers();

//    public RobotCfg(){}
    public RobotCfg(HardwareMap hardwareMap) {
        //get the phone accelerometer and wakelock
        phoneContext = hardwareMap.appContext;
        phoneSensorManager = (SensorManager) phoneContext.getSystemService(Context.SENSOR_SERVICE);
        phoneAccelerometer = phoneSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        accelerometer = Sensors.accelerometer(phoneSensorManager, phoneAccelerometer);

        phonePowerManager = (PowerManager) phoneContext.getSystemService(Context.POWER_SERVICE);

        // This keeps the phone from automatically turning off the screen, which would
        // I think cause some things to go to sleep
        phoneWakeLock = phonePowerManager.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "evlib:FTC_APP_WAKELOCK");
    }

    public Context getPhoneContext() {
        return phoneContext;
    }

    public SensorManager getPhoneSensorManager() {
        return phoneSensorManager;
    }

    public Sensor getPhoneAccelerometer() {
        return phoneAccelerometer;
    }

    public PowerManager getPhonePowerManager() {
        return phonePowerManager;
    }

    public PowerManager.WakeLock getPhoneWakeLock() {
        return phoneWakeLock;
    }

    public Accelerometer getAccelerometer() {
        return accelerometer;
    }

    public Stoppers getStoppers() {
        return stoppers;
    }

    //this does not need to be overridden
    public ServoControl getServo(ServoName servoName) {
        return getServos().getServoMap().get(servoName);
    }

    //empty Servos object
    private static final Servos EMPTY_SERVOS = new Servos(new HashMap<ServoName, ServoControl>());

    //this should be overridden to return your robot's servos
    public Servos getServos() {
        return EMPTY_SERVOS;
    }

    //init(), act() and stop() will be called during the opmode

    //init can be used to initialize sensors, motors, etc.
    public abstract void start();

    //pre_act can be used to update sensors BEFORE teleop runs
    public void pre_act() {}

    //act can be used to update sensors, motors, display telemetry, etc. AFTER teleop runs
    public abstract void act();

    //stop can be used to stop motors, close files, etc.
    public abstract void stop();


    public static RobotCfg fake(HardwareMap hardwareMap) {
        return new RobotCfg(hardwareMap) {
            @Override
            public void start() {

            }

            @Override
            public void pre_act() {

            }

            @Override
            public void act() {

            }

            @Override
            public void stop() {

            }
        };
    }

}
