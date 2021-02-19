package org.firstinspires.ftc.teamcode.GameChangersTester.examples;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import ftc.electronvolts.statemachine.State;

public class BlinkStates {

    enum Patterns {
        RED,
        REDFLASH,
        BLUE,
        BLUEFLASH,
        GREEN,
        GREENFLASH
    }

    public static State makeRed(RevBlinkinLedDriver blinkin){
        return new Blinker(blinkin, RevBlinkinLedDriver.BlinkinPattern.RED,RevBlinkinLedDriver.BlinkinPattern.BLACK,200,200);
    }




}


