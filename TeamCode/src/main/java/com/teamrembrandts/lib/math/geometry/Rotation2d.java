package com.teamrembrandts.lib.math.geometry;

public class Rotation2d {
    private double rad;

    public Rotation2d(double radians) {
        this.rad = radians;
    }

    public double getRadians() {
        return rad;
    }

    public double getDegrees() {
        return Math.toDegrees(rad);
    }
}
