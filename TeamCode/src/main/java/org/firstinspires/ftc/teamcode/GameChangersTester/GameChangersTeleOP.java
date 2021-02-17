package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.IOException;
import java.util.List;

import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.electronvolts.statemachine.StateMap;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.AnalogInputEdgeDetector;
import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.InputExtractors;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.opmodes.AbstractTeleOp;
import ftc.evlib.util.FileUtil;
import ftc.evlib.util.ImmutableList;

@TeleOp(name = "GameChangersTeleOP")
public class GameChangersTeleOP extends AbstractTeleOp<GameChangersRobotCfg>  {

    private boolean wobbleGoalGrabberIsUp = true; //this requires the wobble goal collector to initialize by being up
    private boolean pincherIsClosed = true;
    private boolean collectorIsSucking = false;
    private StateMachine shooterStateMachine;

    private AnalogInputEdgeDetector collectorIntakeButton;
    private AnalogInputEdgeDetector collectorShooterButton;
    private TeamColor teamColor;
    private VuforiaTrackables targetsUltimateGoal;
    private ResultReceiver<Boolean> vuforiaInitRR = new BasicResultReceiver<>();
    private StateMachine autoPowerShotSM;
    private List<VuforiaTrackable> allTrackables;
    private PowerShotStateMachineFactory factory;
    private StateMachine blinkinStateMachine;
    private BlinkEventListener listener;

    public GameChangersTeleOP() {

    }

    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.eBased(5);
    }

    @Override
    protected GameChangersRobotCfg createRobotCfg() {
        Velocity velocityX = new Velocity(Distance.fromInches(30), Time.fromSeconds(1.0));
        Velocity velocityY = new Velocity(Distance.fromInches(18), Time.fromSeconds(1.0));
        return new GameChangersRobotCfg(hardwareMap, velocityX, velocityY);
    }

    @Override
    protected Logger createLogger() {
        return new Logger("log_", ".csv", ImmutableList.of(
                new Logger.Column("pshot sm states", (InputExtractor<String>) () ->
                        autoPowerShotSM == null ? "none" : autoPowerShotSM.getCurrentStateName().name()),
                new Logger.Column("gyro values", (InputExtractor<Double>) () ->
                        robotCfg.getGyro().getHeading()),
                new Logger.Column("vuforia pos x", (InputExtractor<Double>) () ->
                        factory == null ? Double.NaN : factory.getXyrControl().getCurrentX()),
                new Logger.Column("vuforia pos y", (InputExtractor<Double>) () ->
                        factory == null ? Double.NaN : factory.getXyrControl().getCurrentY())
        ));
    }


    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VuforiaKeyReader.readVuforiaKey();
        parameters.cameraName = robotCfg.getWebcamName();
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
        allTrackables = VuLocalizer.setVuLocalizer(teamColor, targetsUltimateGoal, parameters);
    }


    @Override
    protected void setup() {
        collectorIntakeButton = new AnalogInputEdgeDetector(driver1.left_trigger, 0.3, 0.7, false);
        collectorShooterButton = new AnalogInputEdgeDetector(driver1.right_trigger, 0.3, 0.7, false);
        OptionsFile optionsFile = new OptionsFile(GCConverters.getInstance(), FileUtil.getOptionsFile(GameChangersOptionsOp.FILENAME));
        teamColor = optionsFile.get(GameChangersOptionsOp.teamColorTag, GameChangersOptionsOp.teamColorDefault);
        shooterStateMachine = buildShooterStateMachine();
        listener = new BlinkEventListener();
        blinkinStateMachine = buildBlinkinStateMachine();

        Runnable r = () -> {
            try {
                initVuforia();
                Continuable button = new Continuable() {
                    boolean isRunning = false;
                    @Override
                    public boolean doContinue() {
                        if(driver1.a.justPressed()) {
                            isRunning = !isRunning;
                        }
                        return isRunning;
                    }
                };
                factory = new PowerShotStateMachineFactory(robotCfg, teamColor, Angle.fromDegrees(0.2),
                        robotCfg.getGyro(), 0.6, 0.6, robotCfg.getServos(), robotCfg.getMecanumControl(),
                        button, targetsUltimateGoal, allTrackables);
                autoPowerShotSM = factory.create();
            } catch(RuntimeException r1) {

            }
        };

        Thread t = new Thread(r);
        t.start();
    }



    @Override
    protected void setup_act() {
        robotCfg.getFlyWheelShooter().update();
    }

    @Override
    protected void go() {
        InputExtractor<Double> invertedRightStick = () -> -driver1.right_stick_x.getValue();
        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(InputExtractors.negative(driver1.left_stick_x), driver1.left_stick_y));
        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(invertedRightStick));

        if(teamColor == TeamColor.RED) {
            robotCfg.getCameraServo().goToPreset(ServoPresets.Camera.RED);
        } else {
            robotCfg.getCameraServo().goToPreset(ServoPresets.Camera.BLUE);
        }

    }

    @Override
    protected void act() {
        robotCfg.getFlyWheelShooter().update();
        collectorIntakeButton.update();
        collectorShooterButton.update();
        shooterStateMachine.act();
        blinkinStateMachine.act();

//        telemetry.addData("potentiometer values", robotCfg.getPotentiometer().getValue());
        telemetry.addData("Motor Encoder Valies", robotCfg.getFlywheelEncoderValue());

        if (driver1.left_bumper.justPressed()) {
            if (wobbleGoalGrabberIsUp) {
                robotCfg.getWobbleGoalArm().moveArmDown();
            } else {
                robotCfg.getWobbleGoalArm().moveArmUp();
            }
            wobbleGoalGrabberIsUp = !wobbleGoalGrabberIsUp;
        }

        if (driver1.right_bumper.justPressed()) {
            if (pincherIsClosed) {
                robotCfg.getPincher().goToPreset(ServoPresets.WobblePincher.OPENED);
            } else {
                robotCfg.getPincher().goToPreset(ServoPresets.WobblePincher.CLOSED);
            }
            pincherIsClosed = !pincherIsClosed;
        }


        if (collectorIntakeButton.justPressed()) {
            if (collectorIsSucking) {
                robotCfg.getCollector().stop();
                robotCfg.getElevation().goToPreset(ServoPresets.Elevation.SHOOTING);
                robotCfg.startFlyWheel();
            } else {
                robotCfg.getCollector().ingest();
                robotCfg.getElevation().goToPreset(ServoPresets.Elevation.COLLECTING);
                robotCfg.stopFlyWheel();
            }
            collectorIsSucking = !collectorIsSucking;
        }

        if (driver1.x.justPressed()) {
            robotCfg.getCollector().stop();
            robotCfg.getElevation().goToPreset(ServoPresets.Elevation.POWERSHOOTING);
            robotCfg.startFlyWheel();
            collectorIsSucking = false;

        }

        if (driver1.y.justPressed()) {
            robotCfg.getCollector().stop();
            robotCfg.getElevation().goToPreset(ServoPresets.Elevation.COLLECTING);
            robotCfg.stopFlyWheel();
            collectorIsSucking = false;

        }

        if (driver1.b.justPressed()) {
            robotCfg.getWobbleGoalArm().moveArmMoreDown();
            wobbleGoalGrabberIsUp = false;
        }
        if (driver1.dpad_down.justPressed() && collectorIsSucking) {
            robotCfg.getCollector().expel();
        }
        if (driver1.dpad_down.justReleased() && collectorIsSucking) {
            robotCfg.getCollector().ingest();
        }

        if(autoPowerShotSM != null) {
            autoPowerShotSM.act();
        }

        telemetry.addData("powershot statemachine state",autoPowerShotSM == null ? "none" : autoPowerShotSM.getCurrentStateName().name());

    }

    @Override
    protected void end() {

    }

    private StateMachine buildShooterStateMachine() {
        State idleState = () -> {
            if (collectorShooterButton.isPressed()) {
                return S.SHOOTING;
            }
            return null;
        };

        State shootingState = new ShooterState(collectorShooterButton,robotCfg,150L,800L, S.IDLE);

        StateMachineBuilder b = new StateMachineBuilder(S.IDLE);
        b.add(S.IDLE, idleState);
        b.add(S.SHOOTING, shootingState);



        return b.build();
    }
    enum S implements StateName {
        IDLE,
        SHOOTING
    }

    private StateMachine buildBlinkinStateMachine() {
        BlinkStateBuilder b = new BlinkStateBuilder(robotCfg.getBlinkin(), listener, BlinkEvent.NONE);
        b.addSingleColor(BlinkEvent.NONE, RevBlinkinLedDriver.BlinkinPattern.BLACK);
        b.addSingleColor(BlinkEvent.RED, RevBlinkinLedDriver.BlinkinPattern.RED);
        b.addTwoColors(BlinkEvent.RED_AND_BLUE, RevBlinkinLedDriver.BlinkinPattern.RED, 300L,
                RevBlinkinLedDriver.BlinkinPattern.BLUE, 300L);
        b.addOnAndOff(BlinkEvent.BLINKING_GREEN, RevBlinkinLedDriver.BlinkinPattern.GREEN, 500L);
        return b.build();
    }

}

/*
teleop colors :
1. ten seconds till endgame - neutral slow blink, maybe yellow ??
2. five seconds till endgame - faster blink same color
3. during powershot - flash green while attempting to get vuforia
4. if get vuforia lock - go to solid green, otherwise go to solid red
auto colors :
1. initialize opencv - solid color - blue
2. find rings - flashing orange - 1 blink for zero, 2 blinks for one, and 3 blinks for 4 rings
3. vuforia init - flashing green
4. vuforia lock outcome - solid green or solid red depending
5. go to black at the very end
 */