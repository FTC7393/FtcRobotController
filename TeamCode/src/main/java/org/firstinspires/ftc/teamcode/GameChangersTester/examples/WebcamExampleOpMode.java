package org.firstinspires.ftc.teamcode.GameChangersTester.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.GameChangersTester.BlinkEventListener;
import org.firstinspires.ftc.teamcode.GameChangersTester.GameChangersRobotCfg;
import org.firstinspires.ftc.teamcode.GameChangersTester.RingPipeline;
import org.firstinspires.ftc.teamcode.GameChangersTester.StartingPosition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.ResultReceiver;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.opmodes.AbstractTeleOp;
@Disabled
@TeleOp(name = "webcamTestOp")
public class WebcamExampleOpMode extends AbstractTeleOp<GameChangersRobotCfg> {

    OpenCvCamera webcam;
    ResultReceiver resultReceiver = new BasicResultReceiver();
    private RingPipeline samplePipeline = new RingPipeline(resultReceiver, new BasicResultReceiver<Boolean>(), StartingPosition.LEFT, new BlinkEventListener());


    @Override
    protected Function getJoystickScalingFunction() {
        return null;
    }

    @Override
    protected GameChangersRobotCfg createRobotCfg() {
        return new GameChangersRobotCfg(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        return null;
    }

    @Override
    protected void setup() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webcam.setPipeline(samplePipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
        });


    }

    @Override
    protected void setup_act() {
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.addData("avg color value", samplePipeline.getAvgSaturation());
        RingPipeline.RING_NUMBERS rv = samplePipeline.getRingNumber();
        telemetry.addData("number of rings", rv == null ? "null" : rv.name());
        telemetry.update();
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
