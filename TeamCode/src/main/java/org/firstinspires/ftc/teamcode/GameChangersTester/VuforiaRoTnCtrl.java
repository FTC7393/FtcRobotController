package org.firstinspires.ftc.teamcode.GameChangersTester;


import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VuforiaRoTnCtrl extends XYRControl {

    private double velocityR;
    private Angle polarDirectionCorrection;
    private Vector2D translation;
    private double minTransSize;
    private final RotationControl roTnCtnrl;
    private final VuCalc vuCalc;
    


    /**
     * pass in vuforia trackable(current location) as well as the location to go to in terms of the x cord and y cord
     *
     * @param trackable - (current location)
     * @param xDestIn   - x cord of destination
     * @param yDestIn   - y cord of destination
     */
    public VuforiaRoTnCtrl(VuforiaTrackable trackable, double xDestIn, double yDestIn, double rotationGain, Angle targetHeading, Angle angleTolerance, double maxAngularSpeed, double minAngularSpeed) {
        vuCalc = new VuCalc(xDestIn, yDestIn, minTransSize, trackable);
        roTnCtnrl = RotationControls.headingSource(vuCalc, rotationGain, targetHeading, angleTolerance, maxAngularSpeed, minAngularSpeed);
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
        vuCalc.update();
        this.translation = vuCalc.getTranslation();
        roTnCtnrl.act();

        this.velocityR = roTnCtnrl.getVelocityR();
        this.polarDirectionCorrection = roTnCtnrl.getPolarDirectionCorrection();
        return true;
    }
}


