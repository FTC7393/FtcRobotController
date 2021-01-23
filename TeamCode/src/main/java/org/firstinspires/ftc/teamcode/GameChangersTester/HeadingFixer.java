package org.firstinspires.ftc.teamcode.GameChangersTester;

public class HeadingFixer {

    public static double fix(double heading, String targetName, double roll){
        if(roll > 0 || targetName.equals("Blue Alliance Target")){
            return heading;
        }
        heading += 180;
        if (heading > 180){
            heading -= 360;
        }
        return heading;
    }
}
