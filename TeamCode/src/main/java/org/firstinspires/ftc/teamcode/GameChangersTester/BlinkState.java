package org.firstinspires.ftc.teamcode.GameChangersTester;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import java.util.List;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;

public class BlinkState extends BasicAbstractState {

    private final BlinkEventListener listener;
    private final List<RevBlinkinLedDriver.BlinkinPattern> blinkinPatternList;
    private final List<Long> timeList;
    private int currentPatternIndex;
    private final RevBlinkinLedDriver blinkin;
    long [] endTimes;
    private StateName nextState;

    public BlinkState(BlinkEventListener listener, RevBlinkinLedDriver blinkin, List<RevBlinkinLedDriver.BlinkinPattern> blinkinPatternList, List<Long> timeList) {
        this.listener = listener;
        this.blinkinPatternList = blinkinPatternList;
        this.timeList = timeList;
        this.blinkin = blinkin;
        endTimes = new long[timeList.size()];
    }

    @Override
    public void init() {
        long cycleTime = System.currentTimeMillis();
        blinkin.setPattern(blinkinPatternList.get(0));
        currentPatternIndex = 0;
        long delta = 0;
        for (int i = 0; i < timeList.size(); i++) {
            delta = delta + timeList.get(i);
            endTimes[i] = cycleTime + delta;
        }
    }

    @Override
    public boolean isDone() {
        BlinkEvent currentEvent = listener.extractEvents();
        if(currentEvent != null) {
            nextState = currentEvent;
            return true;
        }
        long currentTime = System.currentTimeMillis();
        int newPatternIndex = currentPatternIndex;
        for (int i = endTimes.length - 1; i >= 0; i--) {
            if(currentTime > endTimes[i]) {
                newPatternIndex = i + 1;
                break;
            }
        }
        if(newPatternIndex != currentPatternIndex) {
            if(newPatternIndex > blinkinPatternList.size()) {
                init();
            } else {
                currentPatternIndex = newPatternIndex;
                blinkin.setPattern(blinkinPatternList.get(currentPatternIndex));
            }
        }
        return false;
    }

    @Override
    public StateName getNextStateName() {
        return nextState;
    }
}
