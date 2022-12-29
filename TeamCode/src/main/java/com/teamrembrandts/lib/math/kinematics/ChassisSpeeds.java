package com.teamrembrandts.lib.math.kinematics;


public class ChassisSpeeds {
    private double vx;
    private double vy;
    private double vr;

    public ChassisSpeeds(double vx, double vy, double vr) {
        this.vx = vx;
        this.vy = vy;
        this.vr = vr;
    }

    public double getVX() {
        return vx;
    }

    public double getVY() {
        return vy;
    }

    public double getVR() {
        return vr;
    }
}
