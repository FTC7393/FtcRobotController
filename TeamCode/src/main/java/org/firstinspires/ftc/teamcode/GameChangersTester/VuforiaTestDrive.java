package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMap;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.evlib.hardware.control.XYRControl;
import ftc.evlib.opmodes.AbstractAutoOp;
import ftc.evlib.opmodes.AbstractOp;
import ftc.evlib.statemachine.EVEndConditions;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.FileUtil;

@Autonomous(name = "VuforiaTestDrive")

public class VuforiaTestDrive extends AbstractAutoOp<GameChangersRobotCfg> {

    private final String VUFORIA_KEY;

    public VuforiaTestDrive() throws IOException {
        super();
        // now read Vuforia Key from file in FTC directory on ControlHub:
        File keyFile = FileUtil.getAppFile("vuforiakey.txt");
        BufferedReader breader = new BufferedReader(new FileReader(keyFile));
        String line = breader.readLine();
        breader.close();
        VUFORIA_KEY = line + " \n";
    }



    @Override
    public StateMachine buildStates() {
        EVStateMachineBuilder b = new EVStateMachineBuilder(S.DRIVE_1,TeamColor.BLUE,Angle.fromDegrees(2), robotCfg.getGyro(), 0.6, 0.6, servos, robotCfg.getMecanumControl() );
        b.addDrive(S.DRIVE_1, S.WAIT, Distance.fromFeet(4), 0.08, 270, 0);
        b.addWait(S.WAIT,S.RUN_VUFORIA,3000);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        VuforiaTrackables targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        // still need to figure out the rest of the parameters
        double xDestIn = 36; // need to test (function of which color we are, we need to have an option to choose what team we are in)
        double yDestIn = 0; // need to test
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
        final VuforiaRotationTranslationCntrl xyrControl = new VuforiaRotationTranslationCntrl(blueTowerGoalTarget, xDestIn, yDestIn, rotationGain, targetHeading, angleTolerance, maxAngularSpeed, minAngularSpeed, transGain, transDeadZone, transMinPower, transMaxPower, upperGainDistanceTreshold);
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
        b.addDrive(S.RUN_VUFORIA, StateMap.of(S.STOP,vuforiaArrived,S.TIMEOUT_LINE, EVEndConditions.timed(Time.fromSeconds(5))), xyrControl);
        b.addStop(S.TIMEOUT_LINE);
        b.addStop(S.STOP);
        return b.build();
    }


    public enum S implements StateName {
        DRIVE_1,
        WAIT,
        RUN_VUFORIA,
        TIMEOUT_LINE,
        STOP;
    }

    @Override
    protected GameChangersRobotCfg createRobotCfg(){
        return new GameChangersRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return null;
    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }

    @Override
    protected void act() {

    }

    @Override
    protected void end() {

    }
}
