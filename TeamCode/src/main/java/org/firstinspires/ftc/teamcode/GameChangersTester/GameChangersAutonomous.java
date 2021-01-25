package org.firstinspires.ftc.teamcode.GameChangersTester;

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

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
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
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVEndConditions;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.FileUtil;
import ftc.evlib.util.ImmutableList;

@Autonomous(name = "GameChangersAuto")


public class GameChangersAutonomous extends AbstractAutoOp<GameChangersRobotCfg> {
    private final String VUFORIA_KEY;
    private VuforiaTrackable towerGoalTarget;
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
    private VuforiaToGyro vuforiaGyro;

    public GameChangersAutonomous() throws IOException {
        super();
        // now read Vuforia Key from file in FTC directory on ControlHub:
        File keyFile = FileUtil.getAppFile("vuforiakey.txt");
        BufferedReader breader = new BufferedReader(new FileReader(keyFile));
        String line = breader.readLine();
        breader.close();
        VUFORIA_KEY = line + " \n";
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
                new Logger.Column("state", new InputExtractor<String>() {
                    @Override
                    public String getValue() {
                        return stateMachine.getCurrentStateName().name();
                    }
                }),
                new Logger.Column("pot", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getPotentiometer().getValue();
                    }
                }),
                new Logger.Column("velocityR", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getMecanumControl().getVelocityR();
                    }
                }),
                new Logger.Column("velocityX", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getMecanumControl().getVelocityX();
                    }
                }),
                new Logger.Column("velocityY", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return robotCfg.getMecanumControl().getVelocityY();
                    }
                }),
                new Logger.Column("currentXCoord", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return xyrControl.getCurrentX();
                    }
                }),
                new Logger.Column("currentYCoord", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return xyrControl.getCurrentY();
                    }
                }),
                new Logger.Column("xDestIn", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return xDestIn;
                    }
                }),
                new Logger.Column("yDestIn", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return yDestIn;
                    }
                }),
                new Logger.Column("driveAngle", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        if (xyrControl.getTranslation() != null) {
                            return xyrControl.getTranslation().getDirection().degrees();
                        }
                        return Double.NaN;
                    }
                }),
                new Logger.Column("driveSpeed", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        if (xyrControl.getTranslation() != null) {
                            return xyrControl.getTranslation().getLength();
                        }
                        return Double.NaN;
                    }
                }),
                new Logger.Column("robotHeading", new InputExtractor<Double>() {

                    @Override
                    public Double getValue() {
                        return xyrControl.getHeading();
                    }
                })
        ));
    }


    @Override
    public void setup() {
        OptionsFile optionsFile = new OptionsFile(GCConverters.getInstance(), FileUtil.getOptionsFile(GameChangersOptionsOp.FILENAME));
        teamColor = optionsFile.get(GameChangersOptionsOp.teamColorTag, GameChangersOptionsOp.teamColorDefault);
        initialDelay = optionsFile.get(GameChangersOptionsOp.initialAutoDelayTag, GameChangersOptionsOp.initialAutoDelayDefault);
        startingPosition = optionsFile.get(GameChangersOptionsOp.startingPositionTag, GameChangersOptionsOp.startingPositionDefault);
        ringNumbersResultReceiver = new RepeatedResultReceiver<>(5);
        ringPipeline = new RingPipeline(ringNumbersResultReceiver, waitForStartRR, startingPosition);
        webcamName = robotCfg.getWebcamName();
//        robotCfg.getCameraServo().goToPreset(ServoPresets.Camera.MIDDLE);
        super.setup();
    }

    @Override
    protected void setup_act() {
        stateMachine.act();
        robotCfg.getCameraServo().act();
        robotCfg.getPincher().act();
        telemetry.addData("state", stateMachine.getCurrentStateName());
        telemetry.addData("number of rings", ringPipeline.getRingNumber());
        telemetry.update();
    }



    @Override
    public StateMachine buildStates() {
        EVStateMachineBuilder b = new EVStateMachineBuilder(S.OPENCV_INIT, teamColor, Angle.fromDegrees(2), robotCfg.getGyro(), 0.6, 0.6, servos, robotCfg.getMecanumControl());
        if(teamColor == TeamColor.BLUE) {
            cameraServoPreset = ServoPresets.Camera.BLUE;
        } else {
            cameraServoPreset = ServoPresets.Camera.RED;
        }
        b.add(S.OPENCV_INIT, makeOpenCvInit(S.WAIT_FOR_START));
        b.addResultReceiverReady(S.WAIT_FOR_START, S.OPENCV_RESULT, waitForStartRR);
        b.addResultReceiverReady(S.OPENCV_RESULT, S.OPENCV_STOP, ringNumbersResultReceiver);
        b.add(S.OPENCV_STOP, makeOpenCVStopper(S.SET_CAMERA_SERVO));
        b.addServo(S.SET_CAMERA_SERVO, S.VUFORIA_INIT, robotCfg.getCameraServo().getName(), cameraServoPreset, false);
        b.add(S.VUFORIA_INIT, makeVuforiaInit(S.WAIT_FOR_OTHER_TEAM));
        b.addWait(S.WAIT_FOR_OTHER_TEAM, S.START_FLYWHEEL, Time.fromSeconds(initialDelay));
        b.add(S.START_FLYWHEEL, new State() {
            @Override
            public StateName act() {
                robotCfg.startFlyWheel();
                robotCfg.getElevation().goToPreset(ServoPresets.Elevation.SHOOTING);
                return S.DRIVE_1;
            }
        });
        if(startingPosition == StartingPosition.LEFT ){
            b.addDrive(S.DRIVE_1, S.DRIVE_1B, Distance.fromFeet(1.5), 1.0, 275, 0);
            b.addDrive(S.DRIVE_1B, S.WAIT, Distance.fromFeet(.3), 0.5, 180, 0);
        }else{
            b.addDrive(S.DRIVE_1, S.DRIVE_1B, Distance.fromFeet(.2), .7, 180, 0);
            b.addDrive(S.DRIVE_1B, S.DRIVE_1C, Distance.fromFeet(1.5), 1.0, 270, 0);
            b.addDrive(S.DRIVE_1C, S.WAIT, Distance.fromFeet(.3), 1.0, 0, 0);
        }

        b.addWait(S.WAIT, S.DRIVE_VUFORIA_TO_POWERSHOT, 3000);
//        b.addWait(S.WAIT, S.VUFORIA_EXPLORE, 3000);

        double transGain = 0.03; // need to test
        double transDeadZone = 2.0; // need to test
        double transMinPower = .15; // need to test
        double transMaxPower = 1.0; // need to test
        //might not need (in inches)
        double upperGainDistanceTreshold = 12; // need to test
        xyrControl = new VuforiaRotationTranslationCntrl(transGain, transDeadZone, transMinPower, transMaxPower, upperGainDistanceTreshold, teamColor);
        b.add(S.VUFORIA_EXPLORE, getVuforiaPosition());
        EndCondition vuforiaArrived = new EndCondition() {
            // making inline class
            @Override
            public void init() {

            }

            @Override
            public boolean isDone() {
                return xyrControl.isDone();
            }
        };
        // add other pairs of state name end conditions
        b.addDrive(S.DRIVE_VUFORIA_TO_POWERSHOT, StateMap.of(S.WAIT_ELEVATION_STABILIZE, vuforiaArrived, S.TIMEOUT_LINE, EVEndConditions.timed(Time.fromSeconds(5))), xyrControl);
        b.add(S.INIT_VUFORIA_GYRO, new State() {
            @Override
            public StateName act() {
                VuCalc vuCalcParam = new VuCalc(0,0,towerGoalTarget,teamColor);
                vuforiaGyro = new VuforiaToGyro(targetsUltimateGoal, vuCalcParam);
                return S.LINEUP_SHOOTING_ROTATION;
            }
        });
        b.addGyroTurn(S.LINEUP_SHOOTING_ROTATION, S.WAIT_ELEVATION_STABILIZE, vuforiaGyro, 0);
        b.addWait(S.WAIT_ELEVATION_STABILIZE, S.SHOOT_RINGS, 700L);
        b.add(S.SHOOT_RINGS, new ShooterState(robotCfg, 200L, 550L, S.TURN_OFF_SHOOTER));
        b.add(S.TURN_OFF_SHOOTER, new State() {
            @Override
            public StateName act() {
                robotCfg.stopFlyWheel();
                robotCfg.getElevation().goToPreset(ServoPresets.Elevation.COLLECTING);
                return S.DETERMINE_RING_STACK;
            }
        });
        b.add(S.DETERMINE_RING_STACK, new State() {
            @Override
            public StateName act() {
                if(ringNumbersResultReceiver.getValue() == RingPipeline.RING_NUMBERS.ring_0) {
                    return S.DRIVE_RING_0;
                } else if (ringNumbersResultReceiver.getValue() == RingPipeline.RING_NUMBERS.ring_1) {
                    return S.DRIVE_RING_1;
                } else {
                    return S.DRIVE_RING_4;
                }
            }
        });
        //-------------------------------------------------------------------------------------------------------------------------------
        //0 rings
        //-------------------------------------------------------------------------------------------------------------------------------
        b.addDrive(S.DRIVE_RING_0, S.MOVE_ARM_DOWN_0, Distance.fromFeet(0.5), 0.7, 225, 0);
        b.add(S.MOVE_ARM_DOWN_0, new State() {
            @Override
            public StateName act() {
                robotCfg.getWobbleGoalArm().moveArmDown();
                return S.WAIT_FOR_DROP_0;
            }
        });
        b.addWait(S.WAIT_FOR_DROP_0, S.DROP_WOBBLE_GOAL_0, 500L);
        b.addServo(S.DROP_WOBBLE_GOAL_0, S.MOVE_ARM_UP_0, robotCfg.getPincher().getName(), ServoPresets.WobblePincher.OPENED, true);
        b.add(S.MOVE_ARM_UP_0, new State() {
            @Override
            public StateName act() {
                robotCfg.getWobbleGoalArm().moveArmUp();
                return S.PARK_0;
            }
        });
        b.addDrive(S.PARK_0, S.STOP, Distance.fromFeet(.7), 1, 0, 0);
        //-------------------------------------------------------------------------------------------------------------------------------
        //1 ring
        //-------------------------------------------------------------------------------------------------------------------------------
        b.addDrive(S.DRIVE_RING_1, S.MOVE_ARM_DOWN_1, Distance.fromFeet(1), 0.7, 280, 0);
        b.add(S.MOVE_ARM_DOWN_1, new State() {
            @Override
            public StateName act() {
                robotCfg.getWobbleGoalArm().moveArmDown();
                return S.WAIT_FOR_DROP_1;
            }
        });
        b.addWait(S.WAIT_FOR_DROP_1, S.DROP_WOBBLE_GOAL_1, 500L);
        b.addServo(S.DROP_WOBBLE_GOAL_1, S.MOVE_ARM_UP_1, robotCfg.getPincher().getName(), ServoPresets.WobblePincher.OPENED, true);
        b.add(S.MOVE_ARM_UP_1, new State() {
            @Override
            public StateName act() {
                robotCfg.getWobbleGoalArm().moveArmUp();
                return S.PARK_4;
            }
        });
        b.addDrive(S.PARK_1, S.STOP, Distance.fromFeet(0.7), 1, 55, 0);
        //-------------------------------------------------------------------------------------------------------------------------------
        //4 rings
        //-------------------------------------------------------------------------------------------------------------------------------
        b.addDrive(S.DRIVE_RING_4, S.MOVE_ARM_DOWN_4, Distance.fromFeet(1.3), 0.7, 265, 0);
        b.add(S.MOVE_ARM_DOWN_4, new State() {
            @Override
            public StateName act() {
                robotCfg.getWobbleGoalArm().moveArmDown();
                return S.WAIT_FOR_DROP_4;
            }
        });
        b.addWait(S.WAIT_FOR_DROP_4, S.DROP_WOBBLE_GOAL_4, 500L);
        b.addServo(S.DROP_WOBBLE_GOAL_4, S.MOVE_ARM_UP_4, robotCfg.getPincher().getName(), ServoPresets.WobblePincher.OPENED, true);
        b.add(S.MOVE_ARM_UP_4, new State() {
            @Override
            public StateName act() {
                robotCfg.getWobbleGoalArm().moveArmUp();
                return S.PARK_4;
            }
        });
        b.addDrive(S.PARK_4, S.STOP, Distance.fromFeet(1.3), 1, 60, 0);

        b.addStop(S.TIMEOUT_LINE);
        b.addStop(S.STOP);
        return b.build();
    }

    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
        allTrackables = VuLocalizer.setVuLocalizer(teamColor,targetsUltimateGoal, parameters);
        if (teamColor == TeamColor.BLUE) {
            towerGoalTarget = allTrackables.get(3);
            xDestIn = -4;
            yDestIn = 40;
        } else {
            towerGoalTarget = allTrackables.get(2);
            xDestIn = 0;
            yDestIn = -40;
        }
        targetsUltimateGoal.activate();
        double rotationGain = 0.5; // need to test
        Angle targetHeading = Angle.fromDegrees(2); // need to test
        Angle angleTolerance = Angle.fromDegrees(.5); // need to test
        double maxAngularSpeed = .5; // need to test
        double minAngularSpeed = 0.05; // need to test
        // heavy dependency on robot orientation, refer to vuCalc class at the end of it
        xyrControl.setVuCalc(towerGoalTarget, xDestIn, yDestIn, rotationGain, targetHeading, angleTolerance, maxAngularSpeed, minAngularSpeed);
    }

    private State makeOpenCvInit(final StateName nextState) {
        return new BasicAbstractState() {
            boolean isDone = false;

            @Override
            public void init() {
                Runnable r = new Runnable() {
                    @Override
                    public void run() {
                        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
                        webcam.setPipeline(ringPipeline);
                        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                            @Override
                            public void onOpened() {
                                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                                isDone = true;
                            }
                        });

                    }
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

    private State makeOpenCVStopper(final StateName nextState) {
        return new BasicAbstractState() {
            private boolean isDone = false;

            @Override
            public void init() {
                Runnable r = new Runnable() {
                    @Override
                    public void run() {
                        webcam.stopStreaming();
                        isDone = true;
                    }
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
        return new BasicAbstractState() {
            private boolean isDone = false;
            @Override
            public void init() {
                Runnable r = new Runnable() {
                    @Override
                    public void run() {
                        initVuforia();
                        isDone = true;
                    }
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
    private State getVuforiaPosition() {
        return new State() {
            @Override
            public StateName act() {
                xyrControl.act();
                Vector2D vector = new Vector2D(xyrControl.getCurrentX(), xyrControl.getCurrentY());
                VuforiaPositionHolder vuforiaPositionHolder = new VuforiaPositionHolder(vector);
                vuforiaPosRR.setValue(vuforiaPositionHolder);
                return null;
            }
        };
    }

//    private State getVuforiaPosition() {
//        return new BasicAbstractState() {
//            @Override
//            public void init() {
//            }
//
//            @Override
//            public boolean isDone() {
//                xyrControl.act();
//                Vector2D vector = new Vector2D(xyrControl.getCurrentX(), xyrControl.getCurrentY());
//                VuforiaPositionHolder vuforiaPositionHolder = new VuforiaPositionHolder(vector);
//                vuforiaPosRR.setValue(vuforiaPositionHolder);
//                return false;
//            }
//
//            @Override
//            public StateName getNextStateName() {
//                return null;
//            }
//        };
//    }

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
        VUFORIA_EXPLORE, WAIT_FOR_START, WAIT_FOR_OTHER_TEAM, SET_CAMERA_SERVO, START_FLYWHEEL, SHOOT_RINGS, WAIT_ELEVATION_STABILIZE, TURN_OFF_SHOOTER, DETERMINE_RING_STACK, DRIVE_RING_0, DRIVE_RING_1, DRIVE_RING_4, PARK_0, PARK_1, PARK_4, DROP_WOBBLE_GOAL, MOVE_ARM_DOWN, WAIT_FOR_DROP, WAIT_FOR_DROP_0, DROP_WOBBLE_GOAL_0, DROP_WOBBLE_GOAL_1, DROP_WOBBLE_GOAL_4, MOVE_ARM_DOWN_0, MOVE_ARM_DOWN_1, WAIT_FOR_DROP_4, WAIT_FOR_DROP_1, MOVE_ARM_UP_4, MOVE_ARM_UP_1, MOVE_ARM_UP_0, MOVE_ARM_DOWN_4, LINEUP_SHOOTING_ROTATION, INIT_VUFORIA_GYRO, OPENCV_INIT
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
        robotCfg.getFlyWheelShooter().update();
        robotCfg.getWobbleGoalArm().act();
    }



    @Override
    protected void end() {

    }
}
