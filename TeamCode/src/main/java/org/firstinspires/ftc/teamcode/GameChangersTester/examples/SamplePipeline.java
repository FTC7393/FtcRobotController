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

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(125,300);
    static final int REGION_WIDTH = 100;
    static final int REGION_HEIGHT = 100;
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


    SimpleDateFormat formatter = new SimpleDateFormat("yyyy_MM_dd_HH_mm");
    Date currentTime = new Date();
    String datestamp = formatter.format(currentTime);
    File dataFile = FileUtil.getLogsFile("image_data_" + datestamp + ".csv");

    private int avg1;
    private String datafile = "/FTC";
    Mat region1;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);
        region1 = Cb.submat(new Rect(region1_pointA, region1_pointB));
    }



    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
        Core.extractChannel(input, region1, 2);
                Imgproc.rectangle(
                input,
                region1_pointA,
                region1_pointB,
                new Scalar(0, 255, 0), 4);
        avg1 = (int) Core.mean(region1).val[0];

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
