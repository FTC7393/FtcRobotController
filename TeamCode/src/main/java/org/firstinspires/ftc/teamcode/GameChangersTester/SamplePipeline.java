package org.firstinspires.ftc.teamcode.GameChangersTester;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SamplePipeline extends OpenCvPipeline {

    private RING_NUMBERS ringValue;

    public enum RING_NUMBERS {
        ring_4,
        ring_1,
        ring_0,
        ring_error
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




    private int avgSaturation;
    Mat region1;
    Mat hsv_image = new Mat();
    Mat saturation_channel = new Mat();






    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, hsv_image, Imgproc.COLOR_RGB2HSV_FULL);
        Core.extractChannel(hsv_image, saturation_channel, 1); //this channel results in the most consistent
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);
        region1 = saturation_channel.submat(new Rect(region1_pointA, region1_pointB));
    }



    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
        avgSaturation = (int) Core.mean(region1).val[0];

        if(avgSaturation >= 165) {
            ringValue = RING_NUMBERS.ring_4;
        } else if (avgSaturation <= 165 && avgSaturation >= 80) {
            ringValue = RING_NUMBERS.ring_1;
        } else if (avgSaturation <= 80 && avgSaturation >= 0) {
            ringValue = RING_NUMBERS.ring_0;
        } else {
            ringValue = RING_NUMBERS.ring_error;
        }

        Imgproc.rectangle( input, region1_pointA, region1_pointB, new Scalar(0, 255, 0), 4);



        return input;

    }

    public RING_NUMBERS getRingValue()  {
        return ringValue;
    }

    public int getAvgSaturation() {
        return avgSaturation;
    }
}
