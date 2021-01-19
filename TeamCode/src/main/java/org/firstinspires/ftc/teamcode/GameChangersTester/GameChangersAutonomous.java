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
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

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
    //options op values
    private TeamColor teamColor = null;
    private double initialDelay = 0.0;
    double xDestIn = 0.0;
    double yDestIn = 0.0;

    private VuforiaRotationTranslationCntrl xyrControl;
    private OpenCvWebcam webcam;
    private SamplePipeline samplePipeline;
    ResultReceiver<SamplePipeline.RING_NUMBERS> ringNumbersResultReceiver;

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
    protected Logger createLogger() {
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
        OptionsFile optionsFile = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(GameChangersOptionsOp.FILENAME));
        teamColor = optionsFile.get(GameChangersOptionsOp.teamColorTag, GameChangersOptionsOp.teamColorDefault);
        initialDelay = optionsFile.get(GameChangersOptionsOp.initialAutoDelayTag, GameChangersOptionsOp.initialAutoDelayDefault);
//        initVuforia();
        super.setup();
        ringNumbersResultReceiver = new RepeatedResultReceiver<>(5);
        samplePipeline = new SamplePipeline(ringNumbersResultReceiver);
    }

    @Override
    protected void setup_act() {
        stateMachine.act();
        telemetry.addData("number of rings", ringNumbersResultReceiver.isReady() ? ringNumbersResultReceiver.getValue() : "null");
        telemetry.update();
    }

    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        VuforiaTrackables targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
        if (teamColor == TeamColor.BLUE) {
            towerGoalTarget = targetsUltimateGoal.get(0);
            towerGoalTarget.setName("Blue Tower Goal Target");
            xDestIn = -4;
            yDestIn = 40;
        } else {
            towerGoalTarget = targetsUltimateGoal.get(1);
            towerGoalTarget.setName("Red Tower Goal Target");
            xDestIn = -4;
            yDestIn = -40;
        }
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
                        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
                        webcam.setPipeline(samplePipeline);
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
        return new BasicAbstractState(){
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

    @Override
    public StateMachine buildStates() {
        EVStateMachineBuilder b = new EVStateMachineBuilder(S.OPENCV_INIT, teamColor, Angle.fromDegrees(2), robotCfg.getGyro(), 0.6, 0.6, servos, robotCfg.getMecanumControl());
        b.add(S.OPENCV_INIT, makeOpenCvInit(S.OPENCV_RESULT));
        b.addResultReceiverReady(S.OPENCV_RESULT, S.OPENCV_STOP, ringNumbersResultReceiver);
        b.add(S.OPENCV_STOP, makeOpenCVStopper(S.STOP));
        b.addDrive(S.DRIVE_1, S.WAIT, Distance.fromFeet(4), 0.08, 270, 0);
        b.addWait(S.WAIT, S.RUN_VUFORIA, 3000);
        double rotationGain = 0.7; // need to test
        Angle targetHeading = Angle.fromDegrees(90); // need to test
        Angle angleTolerance = Angle.fromDegrees(5); // need to test
        double maxAngularSpeed = 0.7; // need to test
        double minAngularSpeed = 0.05; // need to test
        double transGain = 0.01; // need to test
        double transDeadZone = 2.0; // need to test
        double transMinPower = .15; // need to test
        double transMaxPower = 1.0; // need to test
        //might not need (in inches)
        double upperGainDistanceTreshold = 12; // need to test
        xyrControl = new VuforiaRotationTranslationCntrl(towerGoalTarget,
                xDestIn, yDestIn, rotationGain, targetHeading, angleTolerance, maxAngularSpeed, minAngularSpeed,
                transGain, transDeadZone, transMinPower, transMaxPower, upperGainDistanceTreshold);
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


    public enum S implements StateName {
        DRIVE_1,
        WAIT,
        RUN_VUFORIA,
        TIMEOUT_LINE,
        STOP,
        OPENCV_STOP, OPENCV_RESULT, OPENCV_INIT
    }

    @Override
    protected GameChangersRobotCfg createRobotCfg() {
        return new GameChangersRobotCfg(hardwareMap);
    }

    @Override
    protected void act() {
        telemetry.addData("x", xyrControl.getCurrentX());
        telemetry.addData("y", xyrControl.getCurrentY());
        telemetry.addData("state", stateMachine.getCurrentStateName());
    }

    @Override
    protected void go() {

    }

    @Override
    protected void end() {

    }
}



