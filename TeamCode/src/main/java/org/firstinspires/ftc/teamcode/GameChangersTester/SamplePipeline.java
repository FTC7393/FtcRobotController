package org.firstinspires.ftc.teamcode.GameChangersTester;

import org.firstinspires.ftc.teamcode.GameChangersTester.RingIdentifierError;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

import ftc.evlib.util.FileUtil;

public class SamplePipeline extends OpenCvPipeline {

    enum RING_NUMBERS {
        ring_4,
        ring_1,
        ring_0;
    }

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(325,315);
    static final int REGION_WIDTH = 15;
    static final int REGION_HEIGHT = 75;
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);




    SimpleDateFormat formatter = new SimpleDateFormat("yyyy_MM_dd_HH_mm");
    Date currentTime = new Date();
    String datestamp = formatter.format(currentTime);
    File datafile = FileUtil.getLogsFile("logs_" + datestamp + ".csv");

    private int avg1;
    Mat region1;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();





    private int yellowDiff;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2HSV_FULL);
        Core.extractChannel(YCrCb, Cb, 1); //this channel results in the most consistent
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);
        region1 = Cb.submat(new Rect(region1_pointA, region1_pointB));
    }



    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
        avg1 = (int) Core.mean(region1).val[0];



        Imgproc.rectangle( input, region1_pointA, region1_pointB, new Scalar(0, 255, 0), 4);

        try {
            ringDecider();
        } catch (RingIdentifierError r) {
            r.printStackTrace();
        }

        return input;

    }

    public RING_NUMBERS ringDecider() throws RingIdentifierError {
        if(avg1 > 165) {
            return RING_NUMBERS.ring_4;
        } else if (avg1 < 165 && avg1 > 80) {
            return RING_NUMBERS.ring_1;
        } else if (avg1 < 80 && avg1 > 0) {
            return RING_NUMBERS.ring_0;
        } else {
            throw new RingIdentifierError();
        }
    }

    public int getAvg1() {
        return avg1;
    }
}
