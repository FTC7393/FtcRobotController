package org.firstinspires.ftc.teamcode.GameChangersTester;

import java.util.List;

public class BlinkEventListener {

    private List<BlinkEvent> internalList;

    public synchronized void requestNewBlinkPattern(BlinkEvent event) {
        internalList.add(event);
    }

    public synchronized List<BlinkEvent> extractEvents() {
        return
    }

}
