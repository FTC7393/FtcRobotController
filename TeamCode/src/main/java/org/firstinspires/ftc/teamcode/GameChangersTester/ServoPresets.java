package org.firstinspires.ftc.teamcode.GameChangersTester;

import java.util.HashMap;

public class ServoPresets {

    public enum Pusher implements Preset{
        RELEASE, PUSH
    }

    public enum Elevation implements Preset{
        SHOOTING, COLLECTING
    }

    public enum WobblePincher implements Preset {
        CLOSED,
        OPENED
    }

    public enum Collector implements Preset {
        OFF,
        FORWARD
    }

    public enum Camera implements Preset {
        BLUE,
        MIDDLE,
        RED
    }

}
