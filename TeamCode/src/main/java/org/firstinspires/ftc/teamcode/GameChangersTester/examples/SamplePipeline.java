package org.firstinspires.ftc.teamcode.GameChangersTester.examples;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.FileOutputStream;
import java.io.PrintWriter;

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

    private int avg1;
    private String datafile;

    @Override
    public Mat processFrame(Mat input) {
        Mat region1 = input.submat(new Rect(region1_pointA, region1_pointB));
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
