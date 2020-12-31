package org.firstinspires.ftc.teamcode.GameChangersTester;


import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import ftc.electronvolts.util.ControlLoop;
import ftc.electronvolts.util.ProportionalController;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.units.Angle;
import ftc.evlib.hardware.control.RotationControl;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.XYRControl;
import ftc.evlib.hardware.sensors.Gyro;

public class VuforiaRoTnCtrl extends XYRControl {

    private double velocityR;
    private Angle polarDirectionCorrection;
    private Vector2D translation;
    private final VuforiaTrackable trackable;
    private final double xDestIn;
    private final double yDestIn;
    private double minTransSize;
    private final RotationControl roTnCtnrl;

    /** pass in vuforia trackable(current location) as well as the location to go to in terms of the x cord and y cord
     * @param trackable - (current location)
     * @param xDestIn - x cord of destination
     * @param yDestIn - y cord of destination
     */
    public VuforiaRoTnCtrl(VuforiaTrackable trackable, double xDestIn, double yDestIn) {
        this.trackable = trackable;
        this.xDestIn = xDestIn;
        this.yDestIn = yDestIn;
        roTnCtnrl = vuforiaRoCntrl();
    }

    @Override
    public double getVelocityR() {
        return velocityR;
    }

    @Override
    public Angle getPolarDirectionCorrection() {
        return polarDirectionCorrection;
    }

    @Override
    public Vector2D getTranslation() {
        return translation;
    }

    @Override
    public boolean act() {
        if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
            // getUpdatedRobotLocation() will return null if no new information is available since
            // the last time that call was made, or if the trackable is not currently visible.
            OpenGLMatrix robotLocation = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocation != null) {

                // calculates the vector translation in inches
                VectorF currentPos = robotLocation.getTranslation();
//                translation = (xDestIn - currentPos.get(0))/(yDestIn - currentPos.get(1)));
                double deltaX = xDestIn - currentPos.get(0);
                double deltaY = yDestIn - currentPos.get(1);
                Vector2D tempTrans = new Vector2D(deltaX, deltaY).normalized();
                if(tempTrans.getLength() < minTransSize) {
                    translation = new Vector2D(minTransSize, tempTrans.getAngle());
                } else
                {
                    translation = tempTrans;
                }
            }


        }


        return false;
    }
}
