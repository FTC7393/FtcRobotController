package org.firstinspires.ftc.teamcode.GameChangersTester;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import ftc.evlib.hardware.sensors.Gyro;

public class VuforiaToGyro implements Gyro{

    private final VuforiaTrackables targetsUltimateGoal;
    private final VuCalc vuCalc;

    public VuforiaToGyro(VuforiaTrackables targetsUltimateGoal, VuCalc vuCalc) {
        this.targetsUltimateGoal = targetsUltimateGoal;
        this.vuCalc = vuCalc;
    }

    @Override
    public boolean isCalibrating() {
        return false;
    }

    @Override
    public void stop() {
        // vuforia stopped elsewhere
    }

    @Override
    public void setActive(boolean isActive) {
        if(isActive){
         targetsUltimateGoal.activate();
        }
        else{
            targetsUltimateGoal.deactivate();
        }
    }

    @Override
    public double getHeading() {
        return vuCalc.getHeading();
    }


}

