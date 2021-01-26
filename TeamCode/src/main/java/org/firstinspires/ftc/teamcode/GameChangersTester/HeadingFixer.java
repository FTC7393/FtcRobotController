package org.firstinspires.ftc.teamcode.GameChangersTester;

public class HeadingFixer {

    public static double fix(double heading, String targetName, double roll) {
        if (!targetName.equals("Blue Alliance Target")) {
            return heading;
        }

        if (heading < -100) {

            heading += 180;
        } else if (heading > 100) {
            heading -= 180;
        }

        return heading;
    }
}
