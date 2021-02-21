package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMap;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.RepeatedResultReceiver;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVEndConditions;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.FileUtil;
import ftc.evlib.util.ImmutableList;

@Autonomous(name = "GameChangersAuto")


public class GameChangersAutonomous extends AbstractAutoOp<GameChangersRobotCfg> {
    private final String VUFORIA_KEY;
    private VuforiaTrackable sideWallTarget;
    private WebcamName webcamName;
    //options op values
    private TeamColor teamColor = null;
    private double initialDelay = 0.0;
    double xDestIn = 0.0;
    double yDestIn = 0.0;
    private VuforiaRotationTranslationCntrl xyrControl;
    private OpenCvWebcam webcam;
    private RingPipeline ringPipeline;
    ResultReceiver<RingPipeline.RING_NUMBERS> ringNumbersResultReceiver;
    ResultReceiver<VuforiaPositionHolder> vuforiaPosRR = new RepeatedResultReceiver<>(1);
    ResultReceiver<Boolean> waitForStartRR = new BasicResultReceiver<>();
    VuforiaTrackables targetsUltimateGoal;
    private List<VuforiaTrackable> allTrackables;
    private StartingPosition startingPosition;
    private ServoPresets.Camera cameraServoPreset;
    private ResultReceiver<Boolean> vuforiaInitRR = new BasicResultReceiver<>();
    private long wobbleGoalWaitTime;
    private long servoReleaseWaitTime;
    private BlinkEventListener listener = new BlinkEventListener();
    private StateMachine blinkinStateMachine;
    private BlinkEvent lastBlinkState = BlinkEvent.NONE;

    public GameChangersAutonomous() {
        super();
        VUFORIA_KEY = VuforiaKeyReader.readVuforiaKey();
    }

    @Override
    protected GameChangersRobotCfg createRobotCfg() {
        return new GameChangersRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return createLogger_unused();
    }

    protected Logger createLogger_unused() {
        return new Logger("auto_log", ".csv", ImmutableList.of(
                new Logger.Column("state", (InputExtractor<String>) () -> stateMachine.getCurrentStateName().name()),
                new Logger.Column("pot", (InputExtractor<Double>) () -> robotCfg.getPotentiometer().getValue()),
                new Logger.Column("velocityR", (InputExtractor<Double>) () -> robotCfg.getMecanumControl().getVelocityR()),
                new Logger.Column("velocityX", (InputExtractor<Double>) () -> robotCfg.getMecanumControl().getVelocityX()),
                new Logger.Column("velocityY", (InputExtractor<Double>) () -> robotCfg.getMecanumControl().getVelocityY()),
                new Logger.Column("currentXCoord", (InputExtractor<Double>) () -> xyrControl.getCurrentX()),
                new Logger.Column("currentYCoord", (InputExtractor<Double>) () -> xyrControl.getCurrentY()),
                new Logger.Column("xDestIn", (InputExtractor<Double>) () -> xDestIn),
                new Logger.Column("yDestIn", (InputExtractor<Double>) () -> yDestIn),
                new Logger.Column("driveAngle", (InputExtractor<Double>) () -> {
                    if (xyrControl.getTranslation() != null) {
                        return xyrControl.getTranslation().getDirection().degrees();
                    }
                    return Double.NaN;
                }),
                new Logger.Column("driveSpeed", (InputExtractor<Double>) () -> {
                    if (xyrControl.getTranslation() != null) {
                        return xyrControl.getTranslation().getLength();
                    }
                    return Double.NaN;
                }),
                new Logger.Column("robotHeading", (InputExtractor<Double>) () -> xyrControl.getHeading())
        ));
    }


    @Override
    public void setup() {
        OptionsFile optionsFile = new OptionsFile(GCConverters.getInstance(), FileUtil.getOptionsFile(GameChangersOptionsOp.FILENAME));
        teamColor = optionsFile.get(GameChangersOptionsOp.teamColorTag, GameChangersOptionsOp.teamColorDefault);
        initialDelay = optionsFile.get(GameChangersOptionsOp.initialAutoDelayTag, GameChangersOptionsOp.initialAutoDelayDefault);
        startingPosition = optionsFile.get(GameChangersOptionsOp.startingPositionTag, GameChangersOptionsOp.startingPositionDefault);
        ringNumbersResultReceiver = new RepeatedResultReceiver<>(5);
        ringPipeline = new RingPipeline(ringNumbersResultReceiver, waitForStartRR, startingPosition, listener);
        webcamName = robotCfg.getWebcamName();
        blinkinStateMachine = buildBlinkinStateMachine();
        //creating xyrcontrol object which will be used during the whole class
        double transGain = 0.03; // need to test
        double transDeadZone = 2.0; // need to test
        double transMinPower = .15; // need to test
        double transMaxPower = 1.0; // need to test
        //might not need (in inches)
        double upperGainDistanceTreshold = 12; // need to test
        xyrControl = new VuforiaRotationTranslationCntrl(transGain, transDeadZone, transMinPower, transMaxPower, upperGainDistanceTreshold, teamColor);

        super.setup();
    }

    @Override
    protected void setup_act() {
        stateMachine.act();
        robotCfg.getCameraServo().act();
        robotCfg.getPincher().act();
        telemetry.addData("state", stateMachine.getCurrentStateName());
        telemetry.addData("number of rings", ringPipeline.getRingNumber());
        if(ringPipeline.getRingNumber() == RingPipeline.RING_NUMBERS.ring_0 && lastBlinkState != BlinkEvent.ZERO_RINGS){
            listener.requestNewBlinkPattern(BlinkEvent.ZERO_RINGS);
            lastBlinkState = BlinkEvent.ZERO_RINGS;
        } else if (ringPipeline.getRingNumber() == RingPipeline.RING_NUMBERS.ring_1 && lastBlinkState != BlinkEvent.ONE_RING){
            listener.requestNewBlinkPattern(BlinkEvent.ONE_RING);
            lastBlinkState = BlinkEvent.ONE_RING;
        } else if (ringPipeline.getRingNumber() == RingPipeline.RING_NUMBERS.ring_4 && lastBlinkState != BlinkEvent.FOUR_RINGS){
            listener.requestNewBlinkPattern(BlinkEvent.FOUR_RINGS);
            lastBlinkState = BlinkEvent.FOUR_RINGS;
        }
        blinkinStateMachine.act();
        telemetry.update();
    }



    @Override
    public StateMachine buildStates() {
        EVStateMachineBuilder b = new EVStateMachineBuilder(S.OPENCV_INIT, teamColor, Angle.fromDegrees(2), robotCfg.getGyro(),
                0.6, 0.6, servos, robotCfg.getMecanumControl());
        if (teamColor == TeamColor.BLUE) {
            cameraServoPreset = ServoPresets.Camera.BLUE;
        } else {
            cameraServoPreset = ServoPresets.Camera.RED;
        }
        wobbleGoalWaitTime = 500L;
        servoReleaseWaitTime = 600L;
        b.add(S.OPENCV_INIT, makeOpenCvInit(S.WAIT_FOR_START));
        b.addResultReceiverReady(S.WAIT_FOR_START, S.OPENCV_RESULT, waitForStartRR);
        b.addResultReceiverReady(S.OPENCV_RESULT, S.OPENCV_STOP, ringNumbersResultReceiver);
        b.add(S.OPENCV_STOP, makeOpenCVStopper(S.SET_CAMERA_SERVO));
        b.addServo(S.SET_CAMERA_SERVO, S.VUFORIA_INIT, robotCfg.getCameraServo().getName(), cameraServoPreset, false);
        b.add(S.VUFORIA_INIT, makeVuforiaInit(S.WAIT_FOR_OTHER_TEAM));
        b.addWait(S.WAIT_FOR_OTHER_TEAM, S.ELEVATE_SHOOTER, Time.fromSeconds(initialDelay));
        b.addServo(S.ELEVATE_SHOOTER,teamColor == TeamColor.RED?S.DRIVE_1:S.BLUE_DRIVE_1,robotCfg.getElevation().getName(),ServoPresets.Elevation.SHOOTING,false);
        if (teamColor == TeamColor.RED) {
            if (startingPosition == StartingPosition.LEFT) {
                b.addDrive(S.DRIVE_1, S.DRIVE_1B, Distance.fromFeet(1.5), 1.0, 275, 0);
                b.addDrive(S.DRIVE_1B, S.SET_VUCALC, Distance.fromFeet(.32), 0.5, 180, 0);
            } else {
                b.addDrive(S.DRIVE_1, S.DRIVE_1B, Distance.fromFeet(.2), .7, 180, 0); // might go out of field
                b.addDrive(S.DRIVE_1B, S.DRIVE_1C, Distance.fromFeet(1.5), 1.0, 270, 0);
                b.addDrive(S.DRIVE_1C, S.SET_VUCALC, Distance.fromFeet(.3), 1.0, 0, 0);
//                b.addDrive(S.DRIVE_1C, S.SET_VUCALC, Distance.fromFeet(.3), 1.0, 0, 0);
            }
            b.add(S.SET_VUCALC, makeVuCalcState(S.WAIT_FOR_VUFORIA_INIT));
            b.addResultReceiverReady(S.WAIT_FOR_VUFORIA_INIT, S.ACTIVATE_TARGETS, vuforiaInitRR);
            b.add(S.ACTIVATE_TARGETS, makeTargetsActivateState(S.DRIVE_VUFORIA_TO_POWERSHOT));
            b.add(S.VUFORIA_EXPLORE, getVuforiaPosition());
            EndCondition vuforiaArrived = createXYREndCondition();
            // add other pairs of state name end conditions
            b.addDrive(S.DRIVE_VUFORIA_TO_POWERSHOT, StateMap.of(S.START_FLYWHEEL, vuforiaArrived, S.START_FLYWHEEL, EVEndConditions.timed(Time.fromSeconds(3))), xyrControl);

            //START timeout drive
            b.addDrive(S.TIMEOUT_DRIVE, S.TIMEOUT_PARK,Distance.fromFeet(.3),.7,270,0);
            b.addDrive(S.TIMEOUT_PARK,S.TIMEOUT_LINE ,Distance.fromFeet(1.6),1,0,0);
            //END timeout drive

            b.add(S.START_FLYWHEEL,makeStartFlyWheelState(S.TURN_AIM_SHOOT));
            b.addGyroTurn(S.TURN_AIM_SHOOT, S.WAIT_ELEVATION_STABILIZE, -1);
            b.addWait(S.WAIT_ELEVATION_STABILIZE, S.SHOOT_RINGS, 2000L);
            b.add(S.SHOOT_RINGS, new ShooterState(robotCfg, 150L, 500L, S.TURN_OFF_SHOOTER));
            b.add(S.TURN_OFF_SHOOTER, makeFlyWheelStopState(S.DEACTIVATE_TARGETS));
            b.add(S.DEACTIVATE_TARGETS, makeTargetsDeactivateState(S.DETERMINE_RING_STACK));
            b.add(S.DETERMINE_RING_STACK, () -> {
                if (ringNumbersResultReceiver.getValue() == RingPipeline.RING_NUMBERS.ring_0) {
                    return S.DRIVE_RING_0;
                } else if (ringNumbersResultReceiver.getValue() == RingPipeline.RING_NUMBERS.ring_1) {
                    return S.DRIVE_RING_1;
                } else {
                    return S.DRIVE_RING_4;
                }
            });
            //-------------------------------------------------------------------------------------------------------------------------------
            //0 rings
            //-------------------------------------------------------------------------------------------------------------------------------
            b.addDrive(S.DRIVE_RING_0, S.MOVE_ARM_DOWN_0, Distance.fromFeet(0.62), 0.7, 225, 0);
            b.add(S.MOVE_ARM_DOWN_0, () -> {
                robotCfg.getWobbleGoalArm().moveArmDown();
                return S.WAIT_FOR_DROP_0;
            });
            b.addWait(S.WAIT_FOR_DROP_0, S.DROP_WOBBLE_GOAL_0, wobbleGoalWaitTime);
            b.addServo(S.DROP_WOBBLE_GOAL_0, S.MOVE_ARM_UP_0, robotCfg.getPincher().getName(), ServoPresets.WobblePincher.OPENED,servoReleaseWaitTime, true);
            b.add(S.MOVE_ARM_UP_0, () -> {
                robotCfg.getWobbleGoalArm().moveArmUp();
                return S.PARK_0;
            });
//            b.addDrive(S.PARK_0_A, S.PARK_0, Distance.fromFeet(1), 1, 0, 0);
            b.addDrive(S.PARK_0, S.STOP, Distance.fromFeet(1.8), 1, 3, 0);
            //-------------------------------------------------------------------------------------------------------------------------------
            //1 ring
            //-------------------------------------------------------------------------------------------------------------------------------
            b.addDrive(S.DRIVE_RING_1, S.MOVE_ARM_DOWN_1, Distance.fromFeet(.95), 0.7, 275, 0);
            b.add(S.MOVE_ARM_DOWN_1, () -> {
                robotCfg.getWobbleGoalArm().moveArmDown();
                return S.WAIT_FOR_DROP_1;
            });
            b.addWait(S.WAIT_FOR_DROP_1, S.DROP_WOBBLE_GOAL_1, wobbleGoalWaitTime);
            b.addServo(S.DROP_WOBBLE_GOAL_1, S.MOVE_ARM_UP_1, robotCfg.getPincher().getName(), ServoPresets.WobblePincher.OPENED,servoReleaseWaitTime, true);
            b.add(S.MOVE_ARM_UP_1, () -> {
                robotCfg.getWobbleGoalArm().moveArmUp();
                return S.PARK_1_A;
            });
            b.addDrive(S.PARK_1_A, S.PARK_1, Distance.fromFeet(1), 1, 0, 0);
            b.addDrive(S.PARK_1, S.STOP, Distance.fromFeet(0.8), 1, 65, 0);
            //-------------------------------------------------------------------------------------------------------------------------------
            //4 rings
            //-------------------------------------------------------------------------------------------------------------------------------
            b.addDrive(S.DRIVE_RING_4, S.MOVE_ARM_DOWN_4, Distance.fromFeet(1.55), 0.7, 255, 0);
            b.add(S.MOVE_ARM_DOWN_4, () -> {
                robotCfg.getWobbleGoalArm().moveArmDown();
                return S.WAIT_FOR_DROP_4;
            });
            b.addWait(S.WAIT_FOR_DROP_4, S.DROP_WOBBLE_GOAL_4, wobbleGoalWaitTime);
            b.addServo(S.DROP_WOBBLE_GOAL_4, S.MOVE_ARM_UP_4, robotCfg.getPincher().getName(), ServoPresets.WobblePincher.OPENED, servoReleaseWaitTime,true); // need to be condensed using new method
            b.add(S.MOVE_ARM_UP_4, () -> {
                robotCfg.getWobbleGoalArm().moveArmUp();
                return S.PARK_4_A;
            });
            b.addDrive(S.PARK_4_A, S.PARK_4, Distance.fromFeet(1), 1, 5, 0);
            b.addDrive(S.PARK_4, S.STOP, Distance.fromFeet(1.35), 1, 55, 0);


            b.addStop(S.TIMEOUT_LINE);
            b.addStop(S.STOP);
        } else {
            if (startingPosition == StartingPosition.LEFT) {
                b.addDrive(S.BLUE_DRIVE_1, S.BLUE_DRIVE_1B, Distance.fromFeet(.2), .7, 0, 0);
                b.addDrive(S.BLUE_DRIVE_1B, S.BLUE_DRIVE_1C, Distance.fromFeet(1.5), 1.0, 270, 0);
                b.addDrive(S.BLUE_DRIVE_1C, S.BLUE_SET_VUCALC, Distance.fromFeet(.3), 1.0, 180, 0);
            } else {
                b.addDrive(S.BLUE_DRIVE_1, S.BLUE_DRIVE_1B, Distance.fromFeet(1.4), 1.0, 265, 0);
                b.addDrive(S.BLUE_DRIVE_1B, S.BLUE_SET_VUCALC, Distance.fromFeet(.43), 0.5, 0, 0);
            }
            b.add(S.BLUE_SET_VUCALC, makeVuCalcState(S.BLUE_WAIT_FOR_VUFORIA_INIT));
            b.addResultReceiverReady(S.BLUE_WAIT_FOR_VUFORIA_INIT, S.BLUE_TARGETS_ACTIVATE, vuforiaInitRR);
            b.add(S.BLUE_TARGETS_ACTIVATE, makeTargetsActivateState(S.BLUE_DRIVE_VUFORIA_TO_POWERSHOT));
//            b.addWait(S.BLUE_WAIT, S.BLUE_DRIVE_VUFORIA_TO_POWERSHOT, 1000);
//        b.addWait(S.WAIT, S.VUFORIA_EXPLORE, 3000L);

            b.add(S.BLUE_VUFORIA_EXPLORE, getVuforiaPosition());
            EndCondition vuforiaArrived = createXYREndCondition();
            // add other pairs of state name end conditions
            b.addDrive(S.BLUE_DRIVE_VUFORIA_TO_POWERSHOT, StateMap.of(S.BLUE_START_FLYWHEEL, vuforiaArrived, S.BLUE_START_FLYWHEEL, EVEndConditions.timed(Time.fromSeconds(3))), xyrControl);

            //START timeout drive
            b.addDrive(S.BLUE_TIMEOUT_DRIVE, S.BLUE_TIMEOUT_PARK,Distance.fromFeet(.3),.7,270,0);
            b.addDrive(S.BLUE_TIMEOUT_PARK,S.BLUE_TIMEOUT_LINE ,Distance.fromFeet(1.6),1,180,0);
            //END timeout drive

            b.add(S.BLUE_START_FLYWHEEL,makeStartFlyWheelState(S.BLUE_TURN_AIM_SHOOT));
            b.addGyroTurn(S.BLUE_TURN_AIM_SHOOT, S.BLUE_WAIT_ELEVATION_STABILIZE, 0);
            b.addWait(S.BLUE_WAIT_ELEVATION_STABILIZE, S.BLUE_SHOOT_RINGS, 2500L);
            b.add(S.BLUE_SHOOT_RINGS, new ShooterState(robotCfg, 200L, 650L, S.BLUE_TURN_OFF_SHOOTER));
            b.add(S.BLUE_TURN_OFF_SHOOTER, makeFlyWheelStopState(S.BLUE_DEACTIVATE_TARGETS));
            b.add(S.BLUE_DEACTIVATE_TARGETS, makeTargetsDeactivateState(S.BLUE_DETERMINE_RING_STACK));
            b.add(S.BLUE_DETERMINE_RING_STACK, () -> {
                if (ringNumbersResultReceiver.getValue() == RingPipeline.RING_NUMBERS.ring_0) {
                    return S.BLUE_DRIVE_RING_0;
                } else if (ringNumbersResultReceiver.getValue() == RingPipeline.RING_NUMBERS.ring_1) {
                    return S.BLUE_DRIVE_RING_1;
                } else {
                    return S.BLUE_DRIVE_RING_4;
                }
            });
            //-------------------------------------------------------------------------------------------------------------------------------
            //0 rings
            //-------------------------------------------------------------------------------------------------------------------------------
            b.addDrive(S.BLUE_DRIVE_RING_0, S.BLUE_WOBBLE_TURN, Distance.fromFeet(0.55), 0.7, 321, -5);
            b.addGyroTurn(S.BLUE_WOBBLE_TURN, S.BLUE_MOVE_ARM_DOWN_0, 180);
            b.add(S.BLUE_MOVE_ARM_DOWN_0, makeArmDownState(S.BLUE_WAIT_FOR_DROP_0));
            b.addWait(S.BLUE_WAIT_FOR_DROP_0, S.BLUE_DROP_WOBBLE_GOAL_0, wobbleGoalWaitTime);
            b.addServo(S.BLUE_DROP_WOBBLE_GOAL_0, S.BLUE_MOVE_ARM_UP_0, robotCfg.getPincher().getName(), ServoPresets.WobblePincher.OPENED, servoReleaseWaitTime,true);
            b.add(S.BLUE_MOVE_ARM_UP_0, makeArmUpState(S.BLUE_PARK_0));
//            b.addDrive(S.BLUE_PARK_0_A, S.BLUE_PARK_0, Distance.fromFeet(1), 1, 180, 180);
            b.addDrive(S.BLUE_PARK_0, S.BLUE_STOP, Distance.fromFeet(1.8), 1, 183, 180);
            //-------------------------------------------------------------------------------------------------------------------------------
            //1 ring (untested changes)
            //-------------------------------------------------------------------------------------------------------------------------------
            b.addDrive(S.BLUE_DRIVE_RING_1, S.BLUE_WOBBLE_TURN_1, Distance.fromFeet(.9), 0.7, 265, -5);
            b.addGyroTurn(S.BLUE_WOBBLE_TURN_1, S.BLUE_MOVE_ARM_DOWN_1, 180);
            b.add(S.BLUE_MOVE_ARM_DOWN_1, makeArmDownState(S.BLUE_WAIT_FOR_DROP_1));
            b.addWait(S.BLUE_WAIT_FOR_DROP_1, S.BLUE_DROP_WOBBLE_GOAL_1, wobbleGoalWaitTime);
            b.addServo(S.BLUE_DROP_WOBBLE_GOAL_1, S.BLUE_MOVE_ARM_UP_1, robotCfg.getPincher().getName(), ServoPresets.WobblePincher.OPENED,servoReleaseWaitTime, true);
            b.add(S.BLUE_MOVE_ARM_UP_1, makeArmUpState(S.BLUE_PARK_1_A));
            b.addDrive(S.BLUE_PARK_1_A, S.BLUE_PARK_1, Distance.fromFeet(1), 1, 180, 180);
            b.addDrive(S.BLUE_PARK_1, S.BLUE_STOP, Distance.fromFeet(0.6), 1, 115, 180);
            //-------------------------------------------------------------------------------------------------------------------------------
            //4 rings
            //-------------------------------------------------------------------------------------------------------------------------------
            b.addDrive(S.BLUE_DRIVE_RING_4, S.BLUE_WOBBLE_TURN_4, Distance.fromFeet(1.45), 0.7, 285, -5);
            b.addGyroTurn(S.BLUE_WOBBLE_TURN_4, S.BLUE_MOVE_ARM_DOWN_4, 180);
            b.add(S.BLUE_MOVE_ARM_DOWN_4, makeArmDownState(S.BLUE_WAIT_FOR_DROP_4));
            b.addWait(S.BLUE_WAIT_FOR_DROP_4, S.BLUE_DROP_WOBBLE_GOAL_4, wobbleGoalWaitTime);
            b.addServo(S.BLUE_DROP_WOBBLE_GOAL_4, S.BLUE_MOVE_ARM_UP_4, robotCfg.getPincher().getName(), ServoPresets.WobblePincher.OPENED, servoReleaseWaitTime,true);
            b.add(S.BLUE_MOVE_ARM_UP_4, makeArmUpState(S.BLUE_PARK_4_A));
            b.addDrive(S.BLUE_PARK_4_A, S.BLUE_PARK_4, Distance.fromFeet(.5), 1, 180, 180);
            b.addDrive(S.BLUE_PARK_4, S.BLUE_STOP, Distance.fromFeet(1.5), 1, 135, 180);

            b.addStop(S.BLUE_TIMEOUT_LINE);
            b.addStop(S.BLUE_STOP);
        }
        return b.build();
    }

    private EndCondition createXYREndCondition() {
         return new EndCondition() {
            @Override
            public void init() {}

            @Override
            public boolean isDone() {
                xyrControl.act();
                return xyrControl.isDone();
            }
        };
    }

    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
        allTrackables = VuLocalizer.setVuLocalizer(teamColor, targetsUltimateGoal, parameters);
        if (teamColor == TeamColor.BLUE) {
            sideWallTarget = allTrackables.get(3);
            xDestIn = 0;
            yDestIn = 33;
        } else {
            sideWallTarget = allTrackables.get(2);
            xDestIn = 0;
            yDestIn = -40;
        }
    }
    private State makeVuCalcState(final StateName nextState) {
        return () -> {
            double rotationGain = 0.5;
            Angle targetHeading = Angle.fromDegrees(2);
            Angle angleTolerance = Angle.fromDegrees(.5);
            double maxAngularSpeed = .5;
            double minAngularSpeed = 0.05;
            // heavy dependency on robot orientation, refer to vuCalc class at the end of it
            xyrControl.setVuCalc(sideWallTarget, xDestIn, yDestIn, rotationGain, targetHeading, angleTolerance, maxAngularSpeed, minAngularSpeed);
            return nextState;
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
        return new State() {
            @Override
            public StateName act() {
                robotCfg.getFlyWheelShooter().turnOnFlywheel();
                return nextState;
            }
        };
    }



    private State makeFlyWheelStopState(final StateName nextState) {
        return new State() {
            @Override
            public StateName act() {
                robotCfg.getFlyWheelShooter().stop();
                robotCfg.getElevation().goToPreset(ServoPresets.Elevation.COLLECTING);
                return nextState;
            }
        };
    }

    private State makeOpenCvInit(final StateName nextState) {
        return new BasicAbstractState() {
            boolean isDone = false;

            @Override
            public void init() {
                Runnable r = () -> {
                    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                    webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
                    webcam.setPipeline(ringPipeline);
                    webcam.openCameraDeviceAsync(() -> {
                        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                        isDone = true;
                    });

                };
                listener.requestNewBlinkPattern(BlinkEvent.BLUE);
                new Thread(r).start();
            }

            @Override
            public boolean isDone() {
                return isDone;
            }

            @Override
            public StateName getNextStateName() {
                return nextState;
            }
        };

    }

    private State makeOpenCVStopper(final StateName nextState) {
        return new BasicAbstractState() {
            private boolean isDone = false;

            @Override
            public void init() {
                Runnable r = () -> {
                    webcam.stopStreaming();
                    isDone = true;
                };
                new Thread(r).start();
            }

            @Override
            public boolean isDone() {
                return isDone;
            }

            @Override
            public StateName getNextStateName() {
                return nextState;
            }
        };
    }

    private State makeVuforiaInit(final StateName nextState) {
        return () -> {
            Runnable r = () -> {
                initVuforia();
                vuforiaInitRR.setValue(true);
            };
            new Thread(r).start();
            return nextState;
        };
    }
    private State getVuforiaPosition() {
        return () -> {
            xyrControl.act();
            Vector2D vector = new Vector2D(xyrControl.getCurrentX(), xyrControl.getCurrentY());
            VuforiaPositionHolder vuforiaPositionHolder = new VuforiaPositionHolder(vector);
            vuforiaPosRR.setValue(vuforiaPositionHolder);
            return null;
        };
    }

    private State makeArmUpState(final StateName nextState) {
        return () -> {
            robotCfg.getWobbleGoalArm().moveArmUp();
            return nextState;
        };
    }

    private State makeArmDownState(final StateName nextState) {
        return () -> {
            robotCfg.getWobbleGoalArm().moveArmDown();
            return nextState;
        };
    }

    private StateMachine buildBlinkinStateMachine() {
        BlinkStateBuilder b = new BlinkStateBuilder(robotCfg.getBlinkin(), listener, BlinkEvent.NONE);
        RevBlinkinLedDriver.BlinkinPattern blue = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        Long t = 200L;
        RevBlinkinLedDriver.BlinkinPattern orange = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
        RevBlinkinLedDriver.BlinkinPattern black = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        b.addSingleColor(BlinkEvent.NONE, black);
        b.addList(BlinkEvent.ZERO_RINGS, ImmutableList.of(orange, black), ImmutableList.of(t, 900L) );
        b.addList(BlinkEvent.ONE_RING, ImmutableList.of(orange, black, orange, black), ImmutableList.of(t, 300L, t, 900L));
        b.addList(BlinkEvent.FOUR_RINGS, ImmutableList.of(orange, black, orange, black, orange, black), ImmutableList.of(t, 300L, t, 300L, t, 900L));

        return b.build();
    }

    public enum S implements StateName {
        DRIVE_1,
        DRIVE_1B,
        DRIVE_1C,
        WAIT,
        DRIVE_VUFORIA_TO_POWERSHOT,
        TIMEOUT_LINE,
        STOP,
        OPENCV_STOP,
        OPENCV_RESULT,
        VUFORIA_INIT,
        VUFORIA_EXPLORE,
        WAIT_FOR_START, WAIT_FOR_OTHER_TEAM, SET_CAMERA_SERVO, START_FLYWHEEL, SHOOT_RINGS, WAIT_ELEVATION_STABILIZE, TURN_OFF_SHOOTER, DETERMINE_RING_STACK, DRIVE_RING_0, DRIVE_RING_1, DRIVE_RING_4, PARK_0, PARK_1, PARK_4, DROP_WOBBLE_GOAL, MOVE_ARM_DOWN, WAIT_FOR_DROP, WAIT_FOR_DROP_0, DROP_WOBBLE_GOAL_0, DROP_WOBBLE_GOAL_1, DROP_WOBBLE_GOAL_4, MOVE_ARM_DOWN_0, MOVE_ARM_DOWN_1, WAIT_FOR_DROP_4, WAIT_FOR_DROP_1, MOVE_ARM_UP_4, MOVE_ARM_UP_1, MOVE_ARM_UP_0, TURN_AIM_SHOOT, MOVE_ARM_DOWN_4, BLUE_STOP, BLUE_TIMEOUT_LINE, BLUE_PARK_4, BLUE_MOVE_ARM_UP_4, BLUE_DROP_WOBBLE_GOAL_4, BLUE_WAIT_FOR_DROP_4, BLUE_MOVE_ARM_DOWN_4, BLUE_DRIVE_RING_4, BLUE_PARK_1, BLUE_MOVE_ARM_UP_1, BLUE_DROP_WOBBLE_GOAL_1, BLUE_WAIT_FOR_DROP_1, BLUE_MOVE_ARM_DOWN_1, BLUE_DRIVE_RING_1, BLUE_PARK_0, BLUE_MOVE_ARM_UP_0, BLUE_DROP_WOBBLE_GOAL_0, BLUE_WAIT_FOR_DROP_0, BLUE_MOVE_ARM_DOWN_0, BLUE_DRIVE_RING_0, BLUE_DETERMINE_RING_STACK, BLUE_TURN_OFF_SHOOTER, BLUE_SHOOT_RINGS, BLUE_WAIT_ELEVATION_STABILIZE, BLUE_TURN_AIM_SHOOT, BLUE_DRIVE_VUFORIA_TO_POWERSHOT, BLUE_VUFORIA_EXPLORE, BLUE_WAIT, BLUE_DRIVE_1C, BLUE_DRIVE_1B, BLUE_DRIVE_1, SET_VUCALC, WAIT_FOR_VUFORIA_INIT, ACTIVATE_TARGETS, DEACTIVATE_TARGETS, BLUE_SET_VUCALC, BLUE_TARGETS_ACTIVATE, BLUE_WAIT_FOR_VUFORIA_INIT, BLUE_DEACTIVATE_TARGETS, ELEVATE_SHOOTER, BLUE_START_FLYWHEEL, BLUE_WOBBLE_TURN, BLUE_WOBBLE_TURN_1, BLUE_WOBBLE_TURN_4, AFTER_PARK, PARK_0_A, PARK_1_A, PARK_4_A, BLUE_PARK_0_A, BLUE_PARK_1_A, BLUE_PARK_4_A, BLUE_PARK_4C, BLUE_TIMEOUT_DRIVE, BLUE_TIMEOUT_PARK, TIMEOUT_DRIVE, TIMEOUT_PARK, OPENCV_INIT
    }



    @Override
    protected void go() {
        waitForStartRR.setValue(true);
    }

    @Override
    protected void act() {
        telemetry.addData("state", stateMachine.getCurrentStateName());
        telemetry.addData("number of rings", ringNumbersResultReceiver.isReady() ? ringNumbersResultReceiver.getValue() : "null");
        telemetry.addData("vuforia position", vuforiaPosRR.isReady() ? vuforiaPosRR.getValue() : "null");
        telemetry.addData("x", xyrControl.getCurrentX());
        telemetry.addData("y", xyrControl.getCurrentY());
        telemetry.addData("robot heading", xyrControl.getHeading());
    }



    @Override
    protected void end() {

    }
}
/*
    auto colors :
        1. initialize opencv - solid color - blue
        2. find rings - flashing orange - 1 blink for zero, 2 blinks for one, and 3 blinks for 4 rings
        3. vuforia init - flashing green
        4. vuforia lock outcome - solid green or solid red depending
        5. go to black at the very end
*/
