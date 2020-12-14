package org.firstinspires.ftc.teamcode.jonv;


/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Map;

import ftc.evlib.util.FileUtil;

/*
 * This version of the internal camera example uses EasyOpenCV's interface to the
 * original Android camera API
 */
@TeleOp
public class RingStackRuler extends LinearOpMode
{
    OpenCvCamera camera;

    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        SamplePipeline pipeline = new SamplePipeline();
        camera.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", camera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", camera.getFps()));
            telemetry.addData("Total frame time ms", camera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", camera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", camera.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", camera.getCurrentPipelineMaxFps());
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                camera.stopStreaming();
                //phoneCam.closeCameraDevice();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        Mat horizontalBoxCb, verticalBoxCb;
        Mat imgYCrCb = new Mat(320,240, CvType.CV_8UC3);
        Mat imgCb = new Mat(320,240, CvType.CV_8UC3);
        int x0 = 90, y0 = 120;
        int nx = 60, ny = 30;
        Mat rowCb = new Mat(1, nx, CvType.CV_8UC3);
        Mat colCb = new Mat(ny, 1, CvType.CV_8UC3);
        Rect insetBox = new Rect(60,80, 200, 100);
//        Rect vertBox = new Rect(60,80, 200, 100);
        int loopCount = 0; // increment every loop (every frame processed)
        private final int SAVE_LOOP_INTERVAL = 30; // save every interval
        boolean viewportPaused = false;
        private double[] horizYellow = new double[nx];
        private double[] vertYellow = new double[ny];
        SimpleDateFormat formatter = new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss");
        Date currentTime = new Date();
        String dateStamp = formatter.format(currentTime);
        File dataFile = FileUtil.getLogsFile("image_data_"+dateStamp+".csv");
        private PrintWriter writer;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */


        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, imgYCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(imgYCrCb, imgCb, 2);

        }


        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'imgCb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            horizontalBoxCb = imgCb.submat(insetBox);
            verticalBoxCb = imgCb.submat(insetBox);
        }

        @Override
        public Mat processFrame(Mat input) {

            inputToCb(input);

            System.out.println("loop count: " + loopCount); // debug
            if (loopCount++ > SAVE_LOOP_INTERVAL) {

                // resize the horiz box to 1 pixel tall row:
                Imgproc.resize(horizontalBoxCb, rowCb, rowCb.size(), 0, 0, Imgproc.INTER_LINEAR);
                Imgproc.resize(verticalBoxCb, colCb, colCb.size(), 0, 0, Imgproc.INTER_LINEAR);

                setHorizYellow(rowCb);
                setVertYellow(colCb);
                openFile();
                append(horizYellow);
                append(",XYZ,");
                append(vertYellow);
                append("\n");
                closeFile();
                loopCount = 0;
            }
            Scalar color = new Scalar(255,0,0);
            int nx = input.cols();
            int ny = input.rows();

            RotatedRect rotatedRectBox = new RotatedRect(new Point(nx/2,ny/2), new Size(nx/8, ny/6), 0);
            Imgproc.ellipse(input, rotatedRectBox, color, 2);

            Imgproc.rectangle(input,insetBox, color,4);

            return input;
        }

        private void setHorizYellow(Mat row) {
            int n = row.cols();
            for (int i=0; i<n; i++) {
                horizYellow[i] = row.get(0, i)[0];
            }
        }
        private void setVertYellow(Mat col) {
            int n = col.rows();
            for (int i=0; i<n; i++) {
                vertYellow[i] = col.get(i, 0)[0];
            }
        }

        private void openFile() {
            try {
                writer = new PrintWriter(new OutputStreamWriter(new FileOutputStream(dataFile, true)));
                System.out.println("open.");
           } catch (IOException e) {
                System.out.println("error on open: " + e.getMessage());
                e.printStackTrace();
            }
        }
        private void closeFile() {
            writer.close();
        }

        private void append(String s) {
            writer.print(s);
        }

        private void append(double [] nums) {
            String SEP = ",";
            for (int i = 0; i < nums.length; i++) {
                writer.print(String.format("%5.1f%s", nums[i], SEP));
            }
        }

        public Mat processFrame2(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            // This was my addition - change the angle of the ellipse based on tapping the screen
//            angle = (angle + 2) % 360;

//            viewportPaused = !viewportPaused;
//
//            if(viewportPaused)
//            {
//                phoneCam.pauseViewport();
//            }
//            else
//            {
//                phoneCam.resumeViewport();
//            }
        }
    }
}


//class CalcHist {
//    public Mat getHistogramImg(Mat src) {
//        List<Mat> bgrPlanes = new ArrayList<>();
//        Core.split(src, bgrPlanes);
//        int histSize = 256;
//        float[] range = {0, 256}; //the upper boundary is exclusive
//        MatOfFloat histRange = new MatOfFloat(range);
//        boolean accumulate = false;
//        Mat horizHist = new Mat(), vertHist = new Mat();
//        Imgproc.calcHist(bgrPlanes, new MatOfInt(0), new Mat(), bHist, new MatOfInt(histSize), histRange, accumulate);
//        Imgproc.calcHist(bgrPlanes, new MatOfInt(1), new Mat(), gHist, new MatOfInt(histSize), histRange, accumulate);
//        Imgproc.calcHist(bgrPlanes, new MatOfInt(2), new Mat(), rHist, new MatOfInt(histSize), histRange, accumulate);
//        int histW = 512, histH = 400;
//        int binW = (int) Math.round((double) histW / histSize);
//        Mat histImage = new Mat( histH, histW, CvType.CV_8UC3, new Scalar( 0,0,0) );
//        Core.normalize(bHist, bHist, 0, histImage.rows(), Core.NORM_MINMAX);
//        Core.normalize(gHist, gHist, 0, histImage.rows(), Core.NORM_MINMAX);
//        Core.normalize(rHist, rHist, 0, histImage.rows(), Core.NORM_MINMAX);
//        float[] bHistData = new float[(int) (bHist.total() * bHist.channels())];
//        bHist.get(0, 0, bHistData);
//        float[] gHistData = new float[(int) (gHist.total() * gHist.channels())];
//        gHist.get(0, 0, gHistData);
//        float[] rHistData = new float[(int) (rHist.total() * rHist.channels())];
//        rHist.get(0, 0, rHistData);
//        for( int i = 1; i < histSize; i++ ) {
//            Imgproc.line(histImage, new Point(binW * (i - 1), histH - Math.round(bHistData[i - 1])),
//                    new Point(binW * (i), histH - Math.round(bHistData[i])), new Scalar(255, 0, 0), 2);
//            Imgproc.line(histImage, new Point(binW * (i - 1), histH - Math.round(gHistData[i - 1])),
//                    new Point(binW * (i), histH - Math.round(gHistData[i])), new Scalar(0, 255, 0), 2);
//            Imgproc.line(histImage, new Point(binW * (i - 1), histH - Math.round(rHistData[i - 1])),
//                    new Point(binW * (i), histH - Math.round(rHistData[i])), new Scalar(0, 0, 255), 2);
//        }
//        HighGui.imshow( "Source image", src );
//        HighGui.imshow( "calcHist Demo", histImage );
//        HighGui.waitKey(0);
//        System.exit(0);
//    }
//}
