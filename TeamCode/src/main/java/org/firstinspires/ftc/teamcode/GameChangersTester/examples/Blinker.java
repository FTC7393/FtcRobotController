package org.firstinspires.ftc.teamcode.GameChangersTester.examples;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;

public class Blinker extends BasicAbstractState {

    private final RevBlinkinLedDriver blinkin;
    private final RevBlinkinLedDriver.BlinkinPattern patt1;
    private final RevBlinkinLedDriver.BlinkinPattern patt2;
    private final long patt1Time;
    private final long patt2Time;

    public Blinker(RevBlinkinLedDriver blinkin, RevBlinkinLedDriver.BlinkinPattern patt1,
                   RevBlinkinLedDriver.BlinkinPattern patt2, long patt1Time, long patt2Time) {
        this.blinkin = blinkin;
        this.patt1 = patt1;
        this.patt2 = patt2;
        this.patt1Time = patt1Time;
        this.patt2Time = patt2Time;
    }

    public Blinker(RevBlinkinLedDriver blinkin, RevBlinkinLedDriver.BlinkinPattern patt) {
        this(blinkin,patt,patt,100,100);
    }

    @Override
    public void init() {
        blinkin.setPattern(patt1);
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public StateName getNextStateName() {
        return null;
    }
}
