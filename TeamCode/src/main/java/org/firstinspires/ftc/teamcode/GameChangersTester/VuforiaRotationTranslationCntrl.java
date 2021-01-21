package org.firstinspires.ftc.teamcode.GameChangersTester;


import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import ftc.electronvolts.util.ProportionalController;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.units.Angle;
import ftc.evlib.hardware.control.RotationControl;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.XYRControl;

public class VuforiaRotationTranslationCntrl extends XYRControl {

    private double velocityR;
    private Angle polarDirectionCorrection;
    private Vector2D translation;
    private double minTransSize;
    private RotationControl roTnCtnrl;
    private final ProportionalController transPropCntrl;
    private VuCalc vuCalc;
    private final double upperGainDistanceThreshold;
    // vector from current position to target position
    private Vector2D rawTrans;
    private final double transDeadZone;


    /**
     * pass in vuforia trackable(current location) as well as the location to go to in terms of the x cord and y cord
     * @param transGain - gain for translation
     * @param transDeadZone - the close enough distance, or you would stop if you are within this zone (Inches)
     * @param transMinPower -  the minimum motor power
     * @param transMaxPower -  the maximum motor power
     */
    public VuforiaRotationTranslationCntrl(double transGain, double transDeadZone, double transMinPower, double transMaxPower, double upperGainDistanceThreshold) {
        this.upperGainDistanceThreshold = upperGainDistanceThreshold;
        transPropCntrl = new ProportionalController(transGain, transDeadZone, transMinPower, transMaxPower);
        this.transDeadZone = transDeadZone;

    }

    public void setVuCalc(VuforiaTrackable trackable, double xDestIn, double yDestIn,  double rotationGain, Angle targetHeading, Angle angleTolerance, double maxAngularSpeed, double minAngularSpeed) {
        vuCalc = new VuCalc(xDestIn, yDestIn, trackable);
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

    public boolean isDone() {
        if(rawTrans != null){
            return rawTrans.getLength() <= transDeadZone;
        }
        return false;
    }

    private double getPos(int index){
        if(vuCalc == null) {
            return Double.NaN;
        }
        VectorF pos = vuCalc.getVuCurrentPos();
        if(pos != null){
            return vuCalc.getVuCurrentPos().get(index)/25.4f;
        }
        return Double.NaN;
    }

    public double getCurrentX(){
        return getPos(0);
    }

    public double getCurrentY(){
        return getPos(1);
    }

    @Override
    public boolean act() {
        vuCalc.update();
        rawTrans = vuCalc.getTranslation();
        if(rawTrans != null) {
            double newMag = rawTrans.getLength();
            // have to find the upperGainDistanceThreshold
            if (newMag > upperGainDistanceThreshold) {
                newMag = upperGainDistanceThreshold;
            }
            double power = Math.abs(transPropCntrl.computeCorrection(0, newMag));
            this.translation = new Vector2D(power, rawTrans.getDirection());

            roTnCtnrl.act();
            this.velocityR = roTnCtnrl.getVelocityR();
            this.polarDirectionCorrection = roTnCtnrl.getPolarDirectionCorrection();

        }
        return true;
    }
}


