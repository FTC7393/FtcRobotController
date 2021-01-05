package org.firstinspires.ftc.teamcode.GameChangersTester;

import ftc.evlib.hardware.motors.Motor;
import ftc.evlib.hardware.servos.ServoControl;

public class wobbleGoalCollecter {

    private final Motor rotation;
    private final ServoControl pinch;

    public wobbleGoalCollecter(Motor rotation, ServoControl pinch) {
        this.rotation = rotation;
        this.pinch = pinch;
    }

    public void close() {
    }



}
