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

    public Rotation2d minus(Rotation2d otherVector) {
        double newRad = rad - otherVector.getRadians();
        return new Rotation2d(newRad);
    }

    public double getAngle(Rotation2d otherVector){
        Rotation2d deltaAngle = this.minus(otherVector);
        return getAngle(deltaAngle);
    }
}
