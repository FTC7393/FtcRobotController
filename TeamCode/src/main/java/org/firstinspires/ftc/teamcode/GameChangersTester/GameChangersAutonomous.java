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
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.statemachine.EVEndConditions;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.EVConverters;
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
    ResultReceiver<VuforiaPositionHolder> vuforiaPosRR = new RepeatedResultReceiver<>(5);
    ResultReceiver<Boolean> waitForStartRR = new BasicResultReceiver<>();
    VuforiaTrackables targetsUltimateGoal;
    private List<VuforiaTrackable> allTrackables;
    private StartingPosition startingPosition;
    private ServoPresets.Camera cameraServoPreset;

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
        return null;
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
        robotCfg.getCameraServo().goToPreset(ServoPresets.Camera.MIDDLE);
        super.setup();
    }

    @Override
    protected void setup_act() {
        stateMachine.act();
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
        b.addWait(S.WAIT_FOR_OTHER_TEAM, S.DRIVE_1, Time.fromSeconds(initialDelay));
        b.addDrive(S.DRIVE_1, S.WAIT, Distance.fromFeet(2), 1.0, 270, 0);
        b.addWait(S.WAIT, S.VUFORIA_EXPLORE, 3000);

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
        b.addDrive(S.RUN_VUFORIA, StateMap.of(S.STOP, vuforiaArrived, S.TIMEOUT_LINE, EVEndConditions.timed(Time.fromSeconds(5))), xyrControl);
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
            towerGoalTarget = allTrackables.get(0);
            xDestIn = -4;
            yDestIn = 40;
        } else {
            towerGoalTarget = allTrackables.get(1);
            xDestIn = -4;
            yDestIn = -40;
        }
        targetsUltimateGoal.activate();
        double rotationGain = 0.5; // need to test
        Angle targetHeading = Angle.fromDegrees(0); // need to test
        Angle angleTolerance = Angle.fromDegrees(2); // need to test
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

    public enum S implements StateName {
        DRIVE_1,
        WAIT,
        RUN_VUFORIA,
        TIMEOUT_LINE,
        STOP,
        OPENCV_STOP,
        OPENCV_RESULT,
        VUFORIA_INIT,
        VUFORIA_EXPLORE, WAIT_FOR_START, WAIT_FOR_OTHER_TEAM, SET_CAMERA_SERVO, OPENCV_INIT
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
        telemetry.addData("vuCalc heading", xyrControl.getVuCalcHeading());
    }



    @Override
    protected void end() {

    }
}
