package ftc.evlib.statemachine;


import java.util.Map;

import ftc.electronvolts.util.InputExtractor;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.control.RotationControl;
import ftc.evlib.hardware.control.TranslationControl;
import ftc.evlib.hardware.control.XYRControl;
import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.ServoName;
import ftc.evlib.hardware.servos.Servos;
//import ftc.evlib.vision.framegrabber.FrameGrabber;
//import ftc.evlib.vision.processors.BeaconColorResult;
import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 5/10/16
 *
 * Builder that uses the EVStates Factory class to create states and build them into a StateMachine
 * extends StateMachineBuilder which has the basic builder methods as well as some useful addXYZ methods
 *
 * @see ftc.evlib.statemachine.EVStates
 * @see StateMachineBuilder
 */
public class EVStateMachineBuilder extends StateMachineBuilder {
    private final TeamColor teamColor;
    private final Angle tolerance;
    private final MecanumControl mecanumControl;
    private final Gyro gyro;
//    private final FrameGrabber frameGrabber;
    private final Servos servos;
    private final double gyroGain;
    private final double maxAngularSpeed;

    /**
     * any of the parameters can be null if the robot does not have it
     *  @param firstStateName the state to start with
     * @param teamColor      the alliance you are on
     * @param tolerance      the tolerance on gyro angles
     * @param gyro           the gyro sensor
     * @param servos         the servos
     * @param mecanumControl the mecanum wheel controller
     */
    public EVStateMachineBuilder(StateName firstStateName, TeamColor teamColor, Angle tolerance, Gyro gyro,
                                 double gyroGain, double maxAngularSpeed,
                                 //FrameGrabber frameGrabber,
                                 Servos servos, MecanumControl mecanumControl) {
        super(firstStateName);
        this.teamColor = teamColor;
        this.tolerance = tolerance;
        this.mecanumControl = mecanumControl;
        this.gyro = gyro;
//        this.frameGrabber = frameGrabber;
        this.servos = servos;
        this.gyroGain = gyroGain;
        this.maxAngularSpeed = maxAngularSpeed;
    }

    public EVStateMachineBuilder(StateName firstStateName, EVStateMachineBuilder b) {
        this(firstStateName, b.teamColor, b.tolerance, b.gyro,
                b.gyroGain, b.maxAngularSpeed,
                // b.frameGrabber,
                b.servos, b.mecanumControl);
    }

    //convenience methods for adding different types of States

    public void addServoInit(StateName stateName, StateName nextStateName) {
        add(stateName, ftc.evlib.statemachine.EVStates.servoInit(nextStateName, servos));
    }

    public void addCalibrateGyro(StateName stateName, StateName nextStateName) {
        add(stateName, ftc.evlib.statemachine.EVStates.calibrateGyro(nextStateName, gyro));
    }

    public void addStop(StateName stateName) {
        add(stateName, ftc.evlib.statemachine.EVStates.stop(mecanumControl));
    }

    ///// START DRIVE STATES /////
    public void addDrive(StateName stateName, Map<StateName, EndCondition> transitions, double velocity, double directionDegrees, double orientationDegrees, double maxAngularSpeed) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(transitions, mecanumControl, gyro, gyroGain, velocity, Angle.fromDegrees(directionDegrees), Angle.fromDegrees(orientationDegrees), tolerance, maxAngularSpeed));
    }

    public void addDrive(StateName stateName, Map<StateName, EndCondition> transitions, double velocity, double directionDegrees, double orientationDegrees) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(transitions, mecanumControl, gyro,gyroGain, velocity, Angle.fromDegrees(directionDegrees), Angle.fromDegrees(orientationDegrees), tolerance));
    }

    public void addDrive(StateName stateName, Map<StateName, EndCondition> transitions, double velocity, Angle direction, Angle orientation, double maxAngularSpeed) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(transitions, mecanumControl, gyro, gyroGain, velocity, direction, orientation, tolerance, maxAngularSpeed));
    }

    public void addDrive(StateName stateName, Map<StateName, EndCondition> transitions, double velocity, Angle direction, Angle orientation) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(transitions, mecanumControl, gyro, gyroGain, velocity, direction, orientation, tolerance));
    }


    public void addDrive(StateName stateName, StateName nextStateName, Distance distance, double velocity, double directionDegrees, double orientationDegrees, double maxAngularSpeed) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(nextStateName, distance, mecanumControl, gyro, gyroGain, velocity, Angle.fromDegrees(directionDegrees), Angle.fromDegrees(orientationDegrees), tolerance, maxAngularSpeed));
    }

    public void addDrive(StateName stateName, StateName nextStateName, Distance distance, double velocity, double directionDegrees, double orientationDegrees) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(nextStateName, distance, mecanumControl, gyro, gyroGain, velocity, Angle.fromDegrees(directionDegrees), Angle.fromDegrees(orientationDegrees), tolerance, maxAngularSpeed));
    }

    public void addDrive(StateName stateName, StateName nextStateName, Distance distance, double velocity, Angle direction, Angle orientation, double maxAngularSpeed) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(nextStateName, distance, mecanumControl, gyro, gyroGain, velocity, direction, orientation, tolerance, maxAngularSpeed));
    }

    public void addDrive(StateName stateName, StateName nextStateName, Distance distance, double velocity, Angle direction, Angle orientation) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(nextStateName, distance, mecanumControl, gyro, gyroGain, velocity, direction, orientation, tolerance, maxAngularSpeed));
    }

    public void addDrive(StateName stateName, StateName nextStateName, Distance distance, double velocity, Angle direction, Angle orientation, Angle tolerance, double maxAngularSpeed) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(nextStateName, distance, mecanumControl, gyro, gyroGain, velocity, direction, orientation, tolerance, maxAngularSpeed));
    }

    public void addDrive(StateName stateName, StateName nextStateName, Distance distance, double velocity, Angle direction, Angle orientation, Angle tolerance) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(nextStateName, distance, mecanumControl, gyro, gyroGain, velocity, direction, orientation, tolerance, maxAngularSpeed));
    }


    public void addDrive(StateName stateName, StateName nextStateName, Distance distance, RotationControl rotationControl, TranslationControl translationControl) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(nextStateName, distance, gyro, mecanumControl, rotationControl, translationControl));
    }

    public void addDrive(StateName stateName, Map<StateName, EndCondition> transitions, RotationControl rotationControl, TranslationControl translationControl) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(transitions, mecanumControl, rotationControl, translationControl));
    }

    public void addDrive(StateName stateName, StateName nextStateName, Distance distance, XYRControl xyrControl) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(nextStateName, distance, gyro,mecanumControl, xyrControl, xyrControl));
    }

    public void addDrive(StateName stateName, Map<StateName, EndCondition> transitions, XYRControl xyrControl) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(transitions, mecanumControl, xyrControl, xyrControl));
    }


    public void addDrive(StateName stateName, Map<StateName, EndCondition> transitions, Vector2D vector2D, double orientationDegrees, double maxAngularSpeed) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(transitions, mecanumControl, gyro, gyroGain, vector2D, Angle.fromDegrees(orientationDegrees), tolerance, maxAngularSpeed));
    }

    public void addDrive(StateName stateName, Map<StateName, EndCondition> transitions, Vector2D vector2D, double orientationDegrees) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(transitions, mecanumControl, gyro, gyroGain, vector2D, Angle.fromDegrees(orientationDegrees), tolerance));
    }

    public void addDrive(StateName stateName, Map<StateName, EndCondition> transitions, Vector2D vector2D, Angle orientation, double maxAngularSpeed) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(transitions, mecanumControl, gyro, gyroGain, vector2D, orientation, tolerance, maxAngularSpeed));
    }

    public void addDrive(StateName stateName, Map<StateName, EndCondition> transitions, Vector2D vector2D, Angle orientation) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(transitions, mecanumControl, gyro, gyroGain, vector2D, orientation, tolerance));
    }


    public void addDrive(StateName stateName, StateName nextStateName, Distance distance, Vector2D vector2D, double orientationDegrees, double maxAngularSpeed) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(nextStateName, distance, mecanumControl, gyro, gyroGain, vector2D, Angle.fromDegrees(orientationDegrees), tolerance, maxAngularSpeed));
    }

    public void addDriveWithSensor(StateName stateName, StateName nextStateName, Distance distance, Vector2D vector2D, double orientationDegrees, double maxAngularSpeed, InputExtractor<Double> sensorReading, double sensorTolerance, double gain, double minVelocity) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDriveWithSensor(nextStateName, distance,sensorReading, sensorTolerance,gain, gyro, gyroGain,mecanumControl,  vector2D, minVelocity, Angle.fromDegrees(orientationDegrees), tolerance, maxAngularSpeed));
    }

    public void addDrive(StateName stateName, StateName nextStateName, Distance distance, Vector2D vector2D, double orientationDegrees) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(nextStateName, distance, mecanumControl, gyro, gyroGain, vector2D, Angle.fromDegrees(orientationDegrees), tolerance, maxAngularSpeed));
    }

    public void addDrive(StateName stateName, StateName nextStateName, Distance distance, Vector2D vector2D, Angle orientation, double maxAngularSpeed) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(nextStateName, distance, mecanumControl, gyro, gyroGain, vector2D, orientation, tolerance, maxAngularSpeed));
    }

    public void addDrive(StateName stateName, StateName nextStateName, Distance distance, Vector2D vector2D, Angle orientation) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(nextStateName, distance, mecanumControl, gyro, gyroGain, vector2D, orientation, tolerance, maxAngularSpeed));
    }

    public void addDrive(StateName stateName, StateName nextStateName, Distance distance, Vector2D vector2D, Angle orientation, Angle tolerance, double maxAngularSpeed) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(nextStateName, distance, mecanumControl, gyro, gyroGain, vector2D, orientation, tolerance, maxAngularSpeed));
    }

    public void addDrive(StateName stateName, StateName nextStateName, Distance distance, Vector2D vector2D, Angle orientation, Angle tolerance) {
        add(stateName, ftc.evlib.statemachine.EVStates.mecanumDrive(nextStateName, distance, mecanumControl, gyro, gyroGain, vector2D, orientation, tolerance, maxAngularSpeed));
    }
    ///// END DRIVE STATES /////

    ///// START TURN STATES /////
    public void addGyroTurn(StateName stateName, StateName nextStateName, double orientationDegrees) {
        add(stateName, ftc.evlib.statemachine.EVStates.gyroTurn(nextStateName, mecanumControl, gyro, gyroGain, Angle.fromDegrees(orientationDegrees), tolerance));
//        add(stateName, EVStates.mecanumDrive(nextStateName, Distance.zero(), mecanumControl, gyro, 0, Angle.zero(), Angle.fromDegrees(orientationDegrees), Angle.fromDegrees(toleranceDegrees)));
    }

    public void addGyroTurn(StateName stateName, StateName nextStateName, double orientationDegrees, Angle tolerance) {
        add(stateName, ftc.evlib.statemachine.EVStates.gyroTurn(nextStateName, mecanumControl, gyro, gyroGain, Angle.fromDegrees(orientationDegrees), tolerance));
//        add(stateName, EVStates.mecanumDrive(nextStateName, Distance.zero(), mecanumControl, gyro, 0, Angle.zero(), Angle.fromDegrees(orientationDegrees), Angle.fromDegrees(toleranceDegrees)));
    }
    public void addGyroTurn(StateName stateName, StateName nextStateName, double orientationDegrees, Angle tolerance,double speed) {
        add(stateName, ftc.evlib.statemachine.EVStates.gyroTurn(nextStateName, mecanumControl, gyro, gyroGain, Angle.fromDegrees(orientationDegrees), tolerance,speed));
//        add(stateName, EVStates.mecanumDrive(nextStateName, Distance.zero(), mecanumControl, gyro, 0, Angle.zero(), Angle.fromDegrees(orientationDegrees), Angle.fromDegrees(toleranceDegrees)));
    }
    public void addGyroTurn(StateName stateName, StateName nextStateName, double orientationDegrees,double speed) {
        add(stateName, ftc.evlib.statemachine.EVStates.gyroTurn(nextStateName, mecanumControl, gyro, gyroGain, Angle.fromDegrees(orientationDegrees), tolerance,speed));
//        add(stateName, EVStates.mecanumDrive(nextStateName, Distance.zero(), mecanumControl, gyro, 0, Angle.zero(), Angle.fromDegrees(orientationDegrees), Angle.fromDegrees(toleranceDegrees)));
    }


    public void addGyroTurn(StateName stateName, StateName nextStateName, Angle orientation) {
        add(stateName, ftc.evlib.statemachine.EVStates.gyroTurn(nextStateName, mecanumControl, gyro, gyroGain, orientation, tolerance));
//        add(stateName, EVStates.mecanumDrive(nextStateName, Distance.zero(), mecanumControl, gyro, 0, Angle.zero(), orientation, tolerance));
    }

    public void addGyroTurn(StateName stateName, StateName nextStateName, Angle orientation, Angle tolerance) {
        add(stateName, ftc.evlib.statemachine.EVStates.gyroTurn(nextStateName, mecanumControl, gyro, gyroGain, orientation, tolerance));
//        add(stateName, EVStates.mecanumDrive(nextStateName, Distance.zero(), mecanumControl, gyro, 0, Angle.zero(), orientation, tolerance));
    }

    public void addGyroTurn(StateName stateName, StateName nextStateName, InputExtractor<Angle> orientation, Angle tolerance, double speed) {
        add(stateName, ftc.evlib.statemachine.EVStates.gyroTurn(nextStateName, mecanumControl, gyro, gyroGain, orientation, tolerance, speed));
//        add(stateName, EVStates.mecanumDrive(nextStateName, Distance.zero(), mecanumControl, gyro, 0, Angle.zero(), orientation, tolerance));
    }
    ///// END TURN STATES /////

    ///// START SERVO STATES /////
    private ServoControl getServo(ServoName servoName) {
        if (!servos.getServoMap().containsKey(servoName)) {
            throw new IllegalArgumentException("ServoName \"" + servoName + "\" was not found in the servoMap");
        }
        return servos.getServoMap().get(servoName);

    }

    public void addServo(StateName stateName, StateName nextStateName, ServoName servoName, double position, boolean waitForDone) {
        add(stateName, ftc.evlib.statemachine.EVStates.servoTurn(nextStateName, getServo(servoName), position, waitForDone));
    }

    public void addServo(StateName stateName, StateName nextStateName, ServoName servoName, double position, double speed, boolean waitForDone) {
        add(stateName, ftc.evlib.statemachine.EVStates.servoTurn(nextStateName, getServo(servoName), position, speed, waitForDone));
    }

    public void addServo(StateName stateName, StateName nextStateName, ServoName servoName, Enum preset, boolean waitForDone) {
        add(stateName, ftc.evlib.statemachine.EVStates.servoTurn(nextStateName, getServo(servoName), preset, waitForDone));
    }



    public void addServo(StateName stateName, StateName nextStateName, ServoName servoName, Enum preset, long milliseconds, boolean waitForDone) {
        add(stateName, ftc.evlib.statemachine.EVStates.servoTurn(nextStateName, getServo(servoName), preset,milliseconds, waitForDone));
    }

    public void addServo(StateName stateName, StateName nextStateName, ServoName servoName, Enum preset, double speed, boolean waitForDone) {
        add(stateName, ftc.evlib.statemachine.EVStates.servoTurn(nextStateName, getServo(servoName), preset, speed, waitForDone));
    }
    public void addServoAdd(StateName stateName, StateName nextStateName, ServoName servoName, double numToAdd, double speed, boolean waitForDone) {
        add(stateName, ftc.evlib.statemachine.EVStates.servoAdd(nextStateName, getServo(servoName), numToAdd, speed, waitForDone));
    }
    public void addServoAdd(StateName stateName, StateName nextStateName, ServoName servoName, double numToAdd,  boolean waitForDone) {
        add(stateName, ftc.evlib.statemachine.EVStates.servoAdd(nextStateName, getServo(servoName), numToAdd, waitForDone));
    }

    ///// END SERVO STATES /////

    public void addTelem(StateName stateName, String message, double value, Map<StateName, EndCondition> transitions) {
        add(stateName, ftc.evlib.statemachine.EVStates.telemetry(transitions, message, value));
    }

    public void addTelem(StateName stateName, String message, double value) {
        add(stateName, ftc.evlib.statemachine.EVStates.telemetry(message, value));
    }

    public void addMotorOn(StateName stateName, StateName nextStateName, Motor motor, double power) {
        add(stateName, ftc.evlib.statemachine.EVStates.motorOn(nextStateName, motor, power));
    }

    public void addMotorOff(StateName stateName, StateName nextStateName, Motor motor) {
        add(stateName, ftc.evlib.statemachine.EVStates.motorOff(nextStateName, motor));
    }

//    public void addDisplayBeaconColor(StateName stateName, ResultReceiver<BeaconColorResult.BeaconColor> beaconColor) {
//        add(stateName, ftc.evlib.statemachine.EVStates.displayBeaconColor(beaconColor));
//    }

//    public void addDisplayBeaconColorResult(StateName stateName, ResultReceiver<BeaconColorResult> beaconColorResult) {
//        add(stateName, ftc.evlib.statemachine.EVStates.displayBeaconColorResult(beaconColorResult));
//    }
//
//    public void addFindColorState(StateName stateName, StateName redState, StateName blueState, StateName unknownState, Time timeout, ResultReceiver<BeaconColorResult.BeaconColor> colorResult, boolean saveImages) {
//        add(stateName, ftc.evlib.statemachine.EVStates.findColorState(redState, blueState, unknownState, timeout, colorResult, saveImages));
//    }

    public void addMotorTurn(StateName stateName, Map<StateName, EndCondition> transitions, Motor motor, double power, boolean useSpeedMode) {
        add(stateName, ftc.evlib.statemachine.EVStates.motorTurn(transitions, motor, power, useSpeedMode));
    }

    public void addMotorTurn(StateName stateName, StateName nextStateName, Time time, Motor motor, double power, boolean useSpeedMode) {
        addMotorTurn(stateName, t(nextStateName, time), motor, power, useSpeedMode);
    }
    public void addResultReceiverReady(StateName stateName, StateName nextStateName, ResultReceiver resultReceiver) {
        add(stateName, EVStates.resultReceiverReady(nextStateName,resultReceiver));
    }

}
