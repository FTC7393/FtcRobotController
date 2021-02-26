package org.firstinspires.ftc.teamcode.GameChangersTester;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import ftc.electronvolts.util.BasicResultReceiver;
import ftc.electronvolts.util.ResultReceiver;

public class RingPipeline extends OpenCvPipeline {

    private final ResultReceiver<RING_NUMBERS> resultReceiver;
    private final ResultReceiver<Boolean> waitForStartRR;
    private final BlinkEventListener listener;

    private RING_NUMBERS ringValue;

    public enum RING_NUMBERS {
        ring_4,
        ring_1,
        ring_0,
        ring_error
    }

    private final Point REGION1_TOPLEFT_ANCHOR_POINT; //inner edge of the right wheels are on left tape
//    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(1,255); //inner edge of the right wheels are on right tape
    private final int REGION_WIDTH = 15;
    private final int REGION_HEIGHT = 75;
    private final Point region1_pointA;
    private final Point region1_pointB;

    private int avgSaturation;
    Mat region1;
    Mat hsv_image = new Mat();
    Mat saturation_channel = new Mat();

    public RingPipeline(ResultReceiver<RING_NUMBERS> resultReceiver, ResultReceiver<Boolean> waitForStartRR, StartingPosition startingPosition, BlinkEventListener listener) {
        this.resultReceiver = resultReceiver;
        this.waitForStartRR = waitForStartRR;
        this.listener = listener;
        if(startingPosition == StartingPosition.LEFT) {
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(535, 294);
        } else {
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(6, 280);
        }

        region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    }

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
            listener.requestNewBlinkPattern(BlinkEvent.FOUR_RINGS);
        } else if (avgSaturation <= 165 && avgSaturation >= 80) {
            ringValue = RING_NUMBERS.ring_1;
            listener.requestNewBlinkPattern(BlinkEvent.ONE_RING);
        } else if (avgSaturation <= 80 && avgSaturation >= 0) {
            ringValue = RING_NUMBERS.ring_0;
            listener.requestNewBlinkPattern(BlinkEvent.ZERO_RINGS);
        } else {
            ringValue = RING_NUMBERS.ring_error;
            listener.requestNewBlinkPattern(BlinkEvent.ERROR_RINGS);
        }

        Imgproc.rectangle( input, region1_pointA, region1_pointB, new Scalar(0, 255, 0), 4);

        if(waitForStartRR.isReady()) {
            resultReceiver.setValue(ringValue);
        }
        return input;

    }

    public RING_NUMBERS getRingNumber()  {
        return ringValue;
    }

    public int getAvgSaturation() {
        return avgSaturation;
    }
}
