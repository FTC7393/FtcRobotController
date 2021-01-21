package org.firstinspires.ftc.teamcode.GameChangersTester;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.units.Angle;
import ftc.evlib.hardware.sensors.HeadingSource;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VuCalc implements HeadingSource {

    private final double xDestIn;
    private final double yDestIn;
    private final VuforiaTrackable trackable;
    private static final double mmPerInch = 25.4f;

    private Vector2D translation;
    private double heading;

    private VectorF currentPos;

    public VuCalc(double xDestIn, double yDestIn, VuforiaTrackable trackable) {
        this.xDestIn = xDestIn;
        this.yDestIn = yDestIn;
        this.trackable = trackable;
    }


    public void update() {
        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
            // getUpdatedRobotLocation() will return null if no new information is available since
            // the last time that call was made, or if the trackable is not currently visible.
            OpenGLMatrix robotLocation = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocation != null) {

                // calculates the vector translation in inches
                currentPos = robotLocation.getTranslation();
//                translation = (xDestIn - currentPos.get(0))/(yDestIn - currentPos.get(1)));
                double deltaX = xDestIn - (currentPos.get(0)/mmPerInch);
                double deltaY = yDestIn - (currentPos.get(1)/mmPerInch);
                // translation distance is in inches
                Vector2D vuVec = new Vector2D(deltaX, deltaY);
                translation = new Vector2D(vuVec.getLength(), Angle.fromDegrees(vuVec.getDirection().degrees()-90));
                Orientation rotation = Orientation.getOrientation(robotLocation, EXTRINSIC, XYZ, DEGREES);
                heading = rotation.thirdAngle - 90;
            }
        }

    }

    @Override
    // robot frame heading
    public double getHeading() {
        return heading;
    }
    // robot frame translation
    public Vector2D getTranslation(){
        return  translation;
    }
    //vuforia frame pos
    public VectorF getVuCurrentPos(){
        return currentPos;
    }
}
