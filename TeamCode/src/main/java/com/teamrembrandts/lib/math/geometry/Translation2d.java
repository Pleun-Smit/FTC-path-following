package com.teamrembrandts.lib.math.geometry;

public class Translation2d {
    private double x;
    private double y;

    public Translation2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Translation2d add(Translation2d otherVector) {
        double newX = x + otherVector.getX();
        double newY = y + otherVector.getY();

        return new Translation2d(newX, newY);
    }

    public Translation2d minus(Translation2d otherVector) {
        double newX = x - otherVector.getX();
        double newY = y - otherVector.getY();

        return new Translation2d(newX, newY);
    }

    public double getDistance(Translation2d otherVector) {
        Translation2d deltaVector = this.minus(otherVector);

        return Math.sqrt(Math.pow(deltaVector.x, 2) + Math.pow(deltaVector.y, 2));
    }
}
