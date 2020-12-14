package org.firstinspires.ftc.teamcode.GameChangersTester.examples;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SamplePipeline extends OpenCvPipeline {
    private boolean hasSavedImage = false;

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.rectangle(
                input,
                new Point(
                        input.cols()/4,
                        input.rows()/4),
                new Point(
                        input.cols()*(3f/4f),
                        input.rows()*(3f/4f)),
                new Scalar(0, 255, 0), 4);
        if(!hasSavedImage) {
            saveMatToDisk(input, "/images/imgae_1");
            hasSavedImage = true;
        }
        return input;

    }
}
