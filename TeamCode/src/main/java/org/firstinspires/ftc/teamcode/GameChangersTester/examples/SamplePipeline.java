package org.firstinspires.ftc.teamcode.GameChangersTester.examples;

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

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(250,225);
    static final int REGION_WIDTH = 150;
    static final int REGION_HEIGHT = 175;
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(450,225);
    static final int REGION2_WIDTH = 150;
    static final int REGION2_HEIGHT = 175;
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


    SimpleDateFormat formatter = new SimpleDateFormat("yyyy_MM_dd_HH_mm");
    Date currentTime = new Date();
    String datestamp = formatter.format(currentTime);
    File datafile = FileUtil.getLogsFile("logs_" + datestamp + ".csv");

    private int avg1;
    Mat region1;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();

    public int getAvg2() {
        return avg2;
    }

    private int avg2;
    Mat region2;

    public int getYellowDiff() {
        return yellowDiff;
    }

    private int yellowDiff;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);
        region1 = Cb.submat(new Rect(region1_pointA, region1_pointB));
        region2 = Cb.submat(new Rect(region2_pointA, region2_pointB));
    }



    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
        avg1 = (int) Core.mean(region1).val[0];
        avg2 = (int) Core.mean(region2).val[0];


        yellowDiff = Math.abs((avg2 - avg1));

        Imgproc.rectangle( input, region1_pointA, region1_pointB, new Scalar(0, 255, 0), 4);

        try {
            PrintWriter writer = new PrintWriter(new FileOutputStream(datafile, true));
            writer.print(avg1);
            writer.print("\n");
            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return input;

    }

    public int getAvg1() {
        return avg1;
    }
}
