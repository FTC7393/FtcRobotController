package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.evlib.util.ImmutableList;

public class BlinkStateBuilder extends StateMachineBuilder {

    private final RevBlinkinLedDriver blinkin;
    private final BlinkEventListener listener;

    public BlinkStateBuilder(RevBlinkinLedDriver blinkin, BlinkEventListener listener, BlinkEvent firstState) {
        super(firstState);
        this.blinkin = blinkin;
        this.listener = listener;
    }

    public void addSingleColor(BlinkEvent stateName, RevBlinkinLedDriver.BlinkinPattern pattern) {
        add(stateName, new BlinkState(listener, blinkin, ImmutableList.of(pattern), ImmutableList.of(200L)));
    }

    public void addTwoColors(BlinkEvent stateName, RevBlinkinLedDriver.BlinkinPattern pattern1, Long time, RevBlinkinLedDriver.BlinkinPattern pattern2, Long time2) {
        add(stateName, new BlinkState(listener, blinkin, ImmutableList.of(pattern1, pattern2), ImmutableList.of(time, time2)));
    }

    public void addOnAndOff(BlinkEvent stateName, RevBlinkinLedDriver.BlinkinPattern pattern, Long time) {
        addTwoColors(stateName, pattern, time, RevBlinkinLedDriver.BlinkinPattern.BLACK, time);
    }

}
