package org.firstinspires.ftc.teamcode.GameChangersTester;

public class RingIdentifierError extends Exception {

    public RingIdentifierError() {
        super.initCause(new Throwable("Could not identify rings"));
    }
}
