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
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.InputExtractors;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.evlib.driverstation.GamepadManager;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
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
    private GamepadManager driver1;
    private VuforiaTrackable trackable;
    private double xDestIn;
    private double yDestIn;
    private double gyroHeadingAtPowershotTime;

    public PowerShotStateMachineFactory(GameChangersRobotCfg robotCfg, TeamColor teamColor, Gyro gyro, double gyroGain, double maxAngularSpeed,
                                        Servos servos, MecanumControl mecanumControl, final Continuable button,
                                        VuforiaTrackables targetsUltimateGoal, List<VuforiaTrackable> allTrackables, GamepadManager driver1) {
        this.robotCfg = robotCfg;

        this.teamColor = teamColor;
        this.tolerance = Angle.fromDegrees(.4);
        this.gyro = gyro;
        this.gyroGain = gyroGain;
        this.maxAngularSpeed = maxAngularSpeed;
        this.servos = servos;
        this.mecanumControl = mecanumControl;
        this.button = button;
        this.targetsUltimateGoal = targetsUltimateGoal;
        this.allTrackables = allTrackables;
        this.driver1 = driver1;


        if(teamColor == TeamColor.BLUE) {
            trackable = allTrackables.get(3);
            xDestIn = 0; //need to tweak
            yDestIn = 22; //need to tweak
        } else {
            trackable = allTrackables.get(2);
            xDestIn = -1; //need to tweak
            yDestIn = -27; //need to tweak
        }

        double transGain = 0.03;
        double transDeadZone = 0.6;
        double transMinPower = .15;
        double transMaxPower = 1.0;
        double upperGainDistanceTreshold = 12;
        xyrControl = new VuforiaRotationTranslationCntrl(transGain, transDeadZone, transMinPower, transMaxPower,
                upperGainDistanceTreshold, teamColor);
        double rotationGain = 0.5;
        Angle targetHeading = Angle.fromDegrees(2); //need to tweak
        Angle angleTolerance = Angle.fromDegrees(.2);
        double maxAngularSpeedXYR = .5;
        double minAngularSpeed = 0.05;
        // heavy dependency on robot orientation, refer to vuCalc class at the end of it
        xyrControl.setVuCalc(trackable, xDestIn, yDestIn, rotationGain, targetHeading, angleTolerance, maxAngularSpeedXYR, minAngularSpeed);
    }

    public StateMachine create() {
        StateName firstState = S.IDLE;
        EVStateMachineBuilder b = new EVStateMachineBuilder(firstState, teamColor, tolerance, gyro, gyroGain, maxAngularSpeed,
                servos, mecanumControl);

        State idleState = () -> {
            if (button.doContinue()) {
                return S.VUFORIA_TARGETS_ACTIVATE;
            }
            return null;
        };

        EndCondition vuforiaSeek = createVuforiaSeekEC(1);

        EndCondition driverHaltEC = createDriverHaltEC();

        StateMap gyroECMap = StateMap.of(S.TIMEOUT_DEACTIVATE, driverHaltEC);


        StateMap vSeekSM = StateMap.of(S.VUFORIA_DRIVE, vuforiaSeek, S.VUFORIA_FAIL_LIGHTS, EndConditions.timed(3000), S.TIMEOUT_DEACTIVATE, driverHaltEC);

        AbstractState vuforiaSeekState = new AbstractState(vSeekSM) {
            @Override public void init() {
                robotCfg.getCollector().stop();
            }

            @Override public void run() {}

            @Override public void dispose() {}
        };

        b.add(firstState, idleState);
        b.add(S.VUFORIA_TARGETS_ACTIVATE, makeTargetsActivateState(S.VUFORIA_SEEK));
        b.add(S.VUFORIA_SEEK, vuforiaSeekState);
        EndCondition vuforiaArrived = createXYREndCondition();
        // add other pairs of state name end conditions
        b.addDrive(S.VUFORIA_DRIVE, StateMap.of(S.VUFORIA_TARGETS_DEACTIVATE, vuforiaArrived, S.VUFORIA_FAIL_LIGHTS,
                EVEndConditions.timed(Time.fromSeconds(5)), S.TIMEOUT_DEACTIVATE, driverHaltEC), xyrControl);
        b.add(S.VUFORIA_TARGETS_DEACTIVATE, makeTargetsDeactivateState(S.GET_GYRO_HEADING));
        b.add(S.GET_GYRO_HEADING, makeGyroHeadingState(S.SET_SHOOTER_SERVO));
        b.addServo(S.SET_SHOOTER_SERVO, S.START_FLYWHEEL, robotCfg.getElevation().getName(),
                ServoPresets.Elevation.POWERSHOOTING, true);
        b.add(S.START_FLYWHEEL, makeStartFlyWheelState(S.TEAMCOLOR_DECIDE));



        b.addBranch(S.TEAMCOLOR_DECIDE, S.MECANUM_DRIVE, S.TURN_RIGHT, teamColor == TeamColor.BLUE);


        //blue branch
        b.addDrive(S.MECANUM_DRIVE, S.BLUE_TURN_LEFT, Distance.fromInches(4), 0.5, gyroHeadingAtPowershotTime+180, gyroHeadingAtPowershotTime);
        b.addGyroTurn(S.BLUE_TURN_LEFT, S.BLUE_WAIT_FOR_FLYWHEEL, () -> Angle.fromDegrees(gyroHeadingAtPowershotTime - 4.5), tolerance, 1, gyroECMap);
        b.add(S.BLUE_WAIT_FOR_FLYWHEEL, makeFlywheelWaitState(S.BLUE_SHOOT_LEFT, S.TIMEOUT_DEACTIVATE, 2500L, 3, 1020));
        b.add(S.BLUE_SHOOT_LEFT, makeShootRingState(S.BLUE_TURN_MIDDLE, 200, S.TIMEOUT_DEACTIVATE));
        b.addGyroTurn(S.BLUE_TURN_MIDDLE, S.BLUE_SHOOT_MIDDLE, () -> Angle.fromDegrees(gyroHeadingAtPowershotTime -9), tolerance, 1, gyroECMap);
        b.add(S.BLUE_SHOOT_MIDDLE, makeShootRingState(S.BLUE_TURN_RIGHT, 200, S.TIMEOUT_DEACTIVATE));
        b.addGyroTurn(S.BLUE_TURN_RIGHT, S.BLUE_SHOOT_RIGHT, () -> Angle.fromDegrees(gyroHeadingAtPowershotTime - 13.5), tolerance, 1, gyroECMap);
        b.add(S.BLUE_SHOOT_RIGHT, makeShootRingState(S.STOP_FLYWHEEL, 200, S.TIMEOUT_DEACTIVATE));



        //red branch
        b.addGyroTurn(S.TURN_RIGHT, S.WAIT_FOR_FLYWHEEL, () -> Angle.fromDegrees(gyroHeadingAtPowershotTime + 0), tolerance, 1, gyroECMap);
        b.add(S.WAIT_FOR_FLYWHEEL, makeFlywheelWaitState(S.SHOOT_RIGHT, S.TIMEOUT_DEACTIVATE, 2500L, 3, 1020));
        b.add(S.SHOOT_RIGHT, makeShootRingState(S.TURN_MIDDLE, 200, S.TIMEOUT_DEACTIVATE));
        b.addGyroTurn(S.TURN_MIDDLE, S.SHOOT_MIDDLE, () -> Angle.fromDegrees(gyroHeadingAtPowershotTime + 4.5), tolerance, 1, gyroECMap);
        b.add(S.SHOOT_MIDDLE, makeShootRingState(S.TURN_LEFT, 200, S.TIMEOUT_DEACTIVATE));
        b.addGyroTurn(S.TURN_LEFT, S.SHOOT_LEFT, () -> Angle.fromDegrees(gyroHeadingAtPowershotTime + 9), tolerance, 1, gyroECMap);
        b.add(S.SHOOT_LEFT, makeShootRingState(S.STOP_FLYWHEEL, 200, S.TIMEOUT_DEACTIVATE));

        //common
        b.add(S.STOP_FLYWHEEL, makeStopFlyWheelState(S.BUTTON_RESET));
        b.add(S.BUTTON_RESET, makeButtonResetState(S.IDLE));


//vuforia never locked, cue the disco
        b.add(S.VUFORIA_FAIL_LIGHTS, createBlinkState(S.TIMEOUT_DEACTIVATE));

        //timeout branch - deactivate targets, prep shooter for driver
        b.add(S.TIMEOUT_DEACTIVATE, makeTargetsDeactivateState(S.TIMEOUT_SET_SHOOTER_SERVO));
        b.addServo(S.TIMEOUT_SET_SHOOTER_SERVO, S.BUTTON_RESET, robotCfg.getElevation().getName(),
                ServoPresets.Elevation.POWERSHOOTING, true);
//        b.add(S.TIMEOUT_START_FLYWHEEL, makeStartFlyWheelState(S.BUTTON_RESET));
        b.add(S.BUTTON_RESET, makeButtonResetState(S.IDLE));
        //TODO: make sure vuforia targets are deactivated
        return b.build();
    }

    private State createBlinkState(StateName nextState) {
        return () -> {
            GameChangersTeleOP.blinkVuforiaFail=true;
            return nextState;
        };
    }

    private State makeFlywheelWaitState(final S continueState, S cancelState, long waitTime, int speedRepeatCount, double minVelocityValue) {
        return new BasicAbstractState() {
            long endTime;
            S nextStateName;
            int count = 0;
            @Override
            public void init() {
                endTime = waitTime + System.currentTimeMillis();
                count = 0;
            }

            @Override
            public boolean isDone() {
                if(!button.doContinue()) {
                    nextStateName = cancelState;
                    return true;
                }
                if(System.currentTimeMillis() >= endTime) {
                    nextStateName = continueState;
                    return true;
                }
                if(robotCfg.getFlyWheelShooter().getVelocity() >= minVelocityValue) {
                    count++;
                } else {
                    count = 0;
                }
                nextStateName = continueState;
                return count >= speedRepeatCount;
            }

            @Override
            public StateName getNextStateName() {
                return nextStateName;
            }
        };
    }

    private State makeButtonResetState(final StateName nextState) {
        return () -> {
            button.reset();
            InputExtractor<Double> invertedRightStick = () -> -driver1.right_stick_x.getValue();
            robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(InputExtractors.negative(driver1.left_stick_x), driver1.left_stick_y));
            robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(invertedRightStick));
            return nextState;
        };
    }

    private State makeGyroHeadingState(final StateName nextState) {
        return new BasicAbstractState() {
            private int runCount;

            @Override
            public void init() {
                runCount = 0;
                robotCfg.getGyro().setActive(true);
            }

            @Override
            public boolean isDone() {
                if(runCount > 3) {
                    gyroHeadingAtPowershotTime = robotCfg.getGyro().getHeading();
                    robotCfg.getGyro().setActive(false);
                    return true;
                }
                runCount++;
                return false;
                }

            @Override
            public StateName getNextStateName() {
                return nextState;
            }
        };
    }

    private State makeShootRingState(final StateName successState, final long delay, final StateName cancelState) {
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
                return !button.doContinue();
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
            robotCfg.getFlyWheelShooter().turnOnFlywheel();
            return nextState;
        };
    }

    private State makeStopFlyWheelState(final StateName nextState) {
        return () -> {
            robotCfg.getFlyWheelShooter().stop();
            return nextState;
        };
    }

    public VuforiaRotationTranslationCntrl getXyrControl() {
        return xyrControl;
    }



    public enum S implements StateName{
        VUFORIA_SEEK, VUFORIA_TARGETS_ACTIVATE, VUFORIA_DRIVE, VUFORIA_TARGETS_DEACTIVATE, START_FLYWHEEL, SET_CAMERA_SERVO,
        TIMEOUT_DEACTIVATE, SET_SHOOTER_SERVO, TIMEOUT_SET_SHOOTER_SERVO, TIMEOUT_START_FLYWHEEL, WAIT_FOR_FLYWHEEL,
        SHOOT_MIDDLE, SHOOT_LEFT, TURN_LEFT, GET_GYRO_HEADING, TURN_RIGHT, SHOOT_RIGHT, STOP_FLYWHEEL, BUTTON_RESET, TIMEOUT_BUTTON_RESET, TEAMCOLOR_DECIDE, MECANUM_DRIVE, TURN_MIDDLE, VUFORIA_FAIL_LIGHTS, BLUE_TURN_RIGHT, BLUE_SHOOT_LEFT, BLUE_TURN_LEFT, BLUE_SHOOT_MIDDLE, BLUE_TURN_MIDDLE, BLUE_SHOOT_RIGHT, BLUE_WAIT_FOR_FLYWHEEL, IDLE
    }

}

