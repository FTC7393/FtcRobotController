package org.firstinspires.ftc.teamcode.GameChangersTester.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.GameChangersTester.GameChangersRobotCfg;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.opmodes.AbstractTeleOp;

@TeleOp(name = "webcamTestOp")
public class WebcamExampleOpMode extends AbstractTeleOp<GameChangersRobotCfg> {

    OpenCvCamera webcam;
    private OpenCvPipeline SamplePipeline = new SamplePipeline();


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
        webcam.setPipeline(new SamplePipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });


    }

    @Override
    protected void setup_act() {

    }

    @Override
    protected void go() {

    }

    @Override
    protected void act() {

        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.update();

        if(driver1.dpad_up.isPressed()) {
            webcam.pauseViewport();
        }

    }

    @Override
    protected void end() {

    }
}
