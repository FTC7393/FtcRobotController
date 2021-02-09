package org.firstinspires.ftc.teamcode.GameChangersTester;


import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.List;

import ftc.electronvolts.statemachine.AbstractState;
import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.EndConditions;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMap;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Time;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.hardware.servos.Servos;
import ftc.evlib.statemachine.EVEndConditions;
import ftc.evlib.statemachine.EVStateMachineBuilder;

public class PowerShotStateMachineFactory {


    private GameChangersRobotCfg robotCfg;
    private final TeamColor teamColor;
    private final Angle tolerance;
    private final Gyro gyro;
    private final double gyroGain;
    private final double maxAngularSpeed;
    private final Servos servos;
    private final MecanumControl mecanumControl;
    private final Continuable button;
    private VuforiaRotationTranslationCntrl xyrControl;
    private VuforiaTrackables targetsUltimateGoal;
    private List<VuforiaTrackable> allTrackables;
    private VuforiaTrackable trackable;
    private double xDestIn;
    private double yDestIn;
    private double gyroHeading;

    public PowerShotStateMachineFactory(GameChangersRobotCfg robotCfg, TeamColor teamColor, Angle tolerance, Gyro gyro, double gyroGain, double maxAngularSpeed,
                                        Servos servos, MecanumControl mecanumControl, final Continuable button,
                                        VuforiaTrackables targetsUltimateGoal, List<VuforiaTrackable> allTrackables) {
        this.robotCfg = robotCfg;

        this.teamColor = teamColor;
        this.tolerance = tolerance;
        this.gyro = gyro;
        this.gyroGain = gyroGain;
        this.maxAngularSpeed = maxAngularSpeed;
        this.servos = servos;
        this.mecanumControl = mecanumControl;
        this.button = button;
        this.targetsUltimateGoal = targetsUltimateGoal;
        this.allTrackables = allTrackables;


        if(teamColor == TeamColor.BLUE) {
            trackable = allTrackables.get(3);
            xDestIn = 0; //need to tweak
            yDestIn = 20; //need to tweak
        } else {
            trackable = allTrackables.get(2);
            xDestIn = 0; //need to tweak
            yDestIn = -25.6; //need to tweak
        }

        double transGain = 0.03;
        double transDeadZone = 2.0;
        double transMinPower = .15;
        double transMaxPower = 1.0;
        double upperGainDistanceTreshold = 12;
        xyrControl = new VuforiaRotationTranslationCntrl(transGain, transDeadZone, transMinPower, transMaxPower,
                upperGainDistanceTreshold, teamColor);
        double rotationGain = 0.5;
        Angle targetHeading = Angle.fromDegrees(2); //need to tweak
        Angle angleTolerance = Angle.fromDegrees(.5);
        double maxAngularSpeedXYR = .5;
        double minAngularSpeed = 0.05;
        // heavy dependency on robot orientation, refer to vuCalc class at the end of it
        xyrControl.setVuCalc(trackable, xDestIn, yDestIn, rotationGain, targetHeading, angleTolerance, maxAngularSpeedXYR, minAngularSpeed);
    }

    public StateMachine create() {
        StateName firstState = S.IDLE;
        EVStateMachineBuilder b = new EVStateMachineBuilder(firstState, teamColor, tolerance, gyro, gyroGain, maxAngularSpeed,
                servos, mecanumControl);

        ServoPresets.Camera cameraServoPreset;
        if(teamColor == TeamColor.BLUE) {
            cameraServoPreset = ServoPresets.Camera.BLUE;
        } else {
            cameraServoPreset = ServoPresets.Camera.RED;
        }

        State idleState = new State() {
            @Override
            public StateName act() {
                if (button.doContinue()) {
                    return S.VUFORIA_TARGETS_ACTIVATE;
                }
                return null;
            }
        };

        EndCondition vuforiaSeek = createVuforiaSeekEC(1);

        StateMap vSeekSM = StateMap.of(S.VUFORIA_DRIVE, vuforiaSeek, S.TIMEOUT_DEACTIVATE, EndConditions.timed(3000));

        AbstractState vuforiaSeekState = new AbstractState(vSeekSM) {
            @Override public void init() {}

            @Override public void run() {}

            @Override public void dispose() {}
        };

        b.add(firstState, idleState);
        b.add(S.VUFORIA_TARGETS_ACTIVATE, makeTargetsActivateState(S.SET_CAMERA_SERVO));
        b.addServo(S.SET_CAMERA_SERVO, S.VUFORIA_SEEK, robotCfg.getCameraServo().getName(), cameraServoPreset, true);
        b.add(S.VUFORIA_SEEK, vuforiaSeekState);
        EndCondition vuforiaArrived = createXYREndCondition();
        // add other pairs of state name end conditions
        EndCondition driverHaltEC = createDriverHaltEC();
        b.addDrive(S.VUFORIA_DRIVE, StateMap.of(S.VUFORIA_TARGETS_DEACTIVATE, vuforiaArrived, S.TIMEOUT_DEACTIVATE,
                EVEndConditions.timed(Time.fromSeconds(3)), S.TIMEOUT_DEACTIVATE, driverHaltEC), xyrControl);
        b.add(S.VUFORIA_TARGETS_DEACTIVATE, makeTargetsDeactivateState(S.GET_GYRO_HEADING));
        b.add(S.GET_GYRO_HEADING, makeGyroHeadingState(S.SET_SHOOTER_SERVO));
        b.addServo(S.SET_SHOOTER_SERVO, S.START_FLYWHEEL, robotCfg.getElevation().getName(),
                ServoPresets.Elevation.POWERSHOOTING, true);
        b.add(S.START_FLYWHEEL, makeStartFlyWheelState(S.WAIT_FOR_FLYWHEEL));
        b.addWait(S.WAIT_FOR_FLYWHEEL, S.SHOOT_MIDDLE, 3000);
        b.add(S.SHOOT_MIDDLE, makeShootRingState(S.TURN_LEFT, 200, S.TIMEOUT_DEACTIVATE, button));
        b.addGyroTurn(S.TURN_LEFT, S.SHOOT_LEFT, () -> Angle.fromDegrees(gyroHeading - 2), tolerance, 1);
        b.add(S.SHOOT_LEFT, makeShootRingState(S.STOP, 200, S.TIMEOUT_DEACTIVATE, button));




        //timeout branch - deactivate targets, prep shooter for driver
        b.add(S.TIMEOUT_DEACTIVATE, makeTargetsDeactivateState(S.TIMEOUT_SET_SHOOTER_SERVO));
        b.addServo(S.TIMEOUT_SET_SHOOTER_SERVO, S.TIMEOUT_START_FLYWHEEL, robotCfg.getElevation().getName(),
                ServoPresets.Elevation.POWERSHOOTING, true);
        b.add(S.TIMEOUT_START_FLYWHEEL, makeStartFlyWheelState(S.IDLE));
        b.addStop(S.STOP);
        //TODO: make sure vuforia targets are deactivated
        return b.build();
    }

    private State makeGyroHeadingState(final StateName nextState) {
        return () -> {
            gyroHeading = robotCfg.getGyro().getHeading();
            return nextState;
        };
    }

    private State makeShootRingState(final StateName successState, final long delay, final StateName cancelState, final Continuable button) {
        return new BasicAbstractState() {
            long endTime;
            StateName nextState;

            @Override
            public void init() {
                endTime = System.currentTimeMillis() + delay;
                robotCfg.getPusher().goToPreset(ServoPresets.Pusher.PUSH);
            }

            @Override
            public boolean isDone() {
                if (!button.doContinue()) {
                    //Pull back servo
                    robotCfg.getPusher().goToPreset(ServoPresets.Pusher.RELEASE);
                    nextState = cancelState;
                    return true;
                }
                if (System.currentTimeMillis() > endTime) {
                    //Pull back servo
                    robotCfg.getPusher().goToPreset(ServoPresets.Pusher.RELEASE);
                    nextState = successState;
                    return true;
                }
                return false;
            }

            @Override
            public StateName getNextStateName() {
                return nextState;
            }
        };
    }


    private EndCondition createDriverHaltEC() {
        return new EndCondition() {
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return false;
            }
        };
    }

    private EndCondition createXYREndCondition() {

        return new EndCondition() {
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                xyrControl.act();
                return xyrControl.isDone();
            }
        };

    }

    private EndCondition createVuforiaSeekEC(final int repeatedVisibilityCount) {
        return new EndCondition() {

            private int counter;

            @Override
            public void init() {
                counter = 0;
            }

            @Override
            public boolean isDone() {

                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    counter++;
                } else {
                    counter = 0;
                }
                return counter >= repeatedVisibilityCount;
            }
        };
    }


    private State makeTargetsActivateState(final StateName nextState) {
        return () -> {
            targetsUltimateGoal.activate();
            return nextState;
        };
    }

    private State makeTargetsDeactivateState(final StateName nextState) {
        return () -> {
            targetsUltimateGoal.deactivate();
            return nextState;
        };
    }

    private State makeStartFlyWheelState(final StateName nextState) {
        return () -> {
            robotCfg.startFlyWheel();
            return nextState;
        };
    }


    public enum S implements StateName{
        VUFORIA_SEEK, VUFORIA_TARGETS_ACTIVATE, VUFORIA_DRIVE, VUFORIA_TARGETS_DEACTIVATE, START_FLYWHEEL, SET_CAMERA_SERVO, TIMEOUT_DEACTIVATE, SET_SHOOTER_SERVO, STOP, TIMEOUT_SET_SHOOTER_SERVO, TIMEOUT_START_FLYWHEEL, WAIT_FOR_FLYWHEEL, SHOOT_MIDDLE, SHOOT_LEFT, TURN_LEFT, GET_GYRO_HEADING, TURN_RIGHT, IDLE
    }

}

