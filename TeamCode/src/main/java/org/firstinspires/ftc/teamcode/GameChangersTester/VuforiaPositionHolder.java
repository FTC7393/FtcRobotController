package org.firstinspires.ftc.teamcode.GameChangersTester;

import java.util.Objects;

import ftc.electronvolts.util.Vector2D;

public class VuforiaPositionHolder {

    private final Vector2D vuforiaPosition;

    public static double maxDistance = 2; //also in inches

    public VuforiaPositionHolder(Vector2D vuforiaPositionInInches) {
        this.vuforiaPosition = vuforiaPositionInInches;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        VuforiaPositionHolder that = (VuforiaPositionHolder) o;
        double deltaX = this.vuforiaPosition.getX() - that.vuforiaPosition.getX();
        double deltaY = this.vuforiaPosition.getY() - that.vuforiaPosition.getY();
        if(Math.sqrt(deltaX*deltaX + deltaY*deltaY) <= maxDistance) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public int hashCode() {
        return Objects.hash(vuforiaPosition);
    }
}
