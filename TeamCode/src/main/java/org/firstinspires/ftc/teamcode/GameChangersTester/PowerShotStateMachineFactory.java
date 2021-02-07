package org.firstinspires.ftc.teamcode.GameChangersTester;


import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.List;

import ftc.electronvolts.statemachine.AbstractState;
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

    public PowerShotStateMachineFactory(TeamColor teamColor, Angle tolerance, Gyro gyro, double gyroGain, double maxAngularSpeed,
                                        Servos servos, MecanumControl mecanumControl, final Continuable button,
                                        VuforiaTrackables targetsUltimateGoal, List<VuforiaTrackable> allTrackables) {

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
            yDestIn = -30; //need to tweak
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

        StateMap vSeekSM = StateMap.of(S.VUFORIA_DRIVE, vuforiaSeek, S.VUFORIA_TARGETS_DEACTIVATE, EndConditions.timed(3000));

        AbstractState vuforiaSeekState = new AbstractState(vSeekSM) {
            @Override public void init() {}

            @Override public void run() {}

            @Override public void dispose() {}
        };

        b.add(firstState, idleState);
        b.add(S.VUFORIA_TARGETS_ACTIVATE, makeTargetsActivateState(S.SET_CAMERA_SERVO));
        b.addServo(S.SET_CAMERA_SERVO, S.VUFORIA_SEEK, GameChangersRobotCfg.GameChangersServoName.CAMERA, cameraServoPreset, true);
        b.add(S.VUFORIA_SEEK, vuforiaSeekState);
        EndCondition vuforiaArrived = createXYREndCondition();
        // add other pairs of state name end conditions
        b.addDrive(S.VUFORIA_DRIVE, StateMap.of(S.START_FLYWHEEL, vuforiaArrived, S.START_FLYWHEEL, EVEndConditions.timed(Time.fromSeconds(3))), xyrControl);
        b.add(S.VUFORIA_TARGETS_DEACTIVATE, makeTargetsDeactivateState(S.START_FLYWHEEL));
        b.addStop(S.START_FLYWHEEL);
        //TODO: make sure vuforia targets are deactivated
        return b.build();
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
        return new State() {
            @Override
            public StateName act() {
                targetsUltimateGoal.activate();
                return nextState;
            }
        };
    }

    private State makeTargetsDeactivateState(final StateName nextState) {
        return new State() {
            @Override
            public StateName act() {
                targetsUltimateGoal.deactivate();
                return nextState;
            }
        };
    }

    public enum S implements StateName{
        VUFORIA_SEEK, VUFORIA_TARGETS_ACTIVATE, VUFORIA_DRIVE, VUFORIA_TARGETS_DEACTIVATE, START_FLYWHEEL, SET_CAMERA_SERVO, IDLE
    }

}

