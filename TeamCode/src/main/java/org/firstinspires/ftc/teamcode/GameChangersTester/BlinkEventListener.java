package org.firstinspires.ftc.teamcode.GameChangersTester;

import java.util.ArrayList;
import java.util.List;

public class BlinkEventListener {

    private final List<BlinkEvent> internalList = new ArrayList<>();

    public synchronized void requestNewBlinkPattern(BlinkEvent event) {
        internalList.add(event);
    }

    public synchronized BlinkEvent extractEvents() {
        if(internalList.isEmpty()) {
            return null;
        } else {
            BlinkEvent event = internalList.get(internalList.size() - 1);
            internalList.clear();
            return event;
        }
    }

}
