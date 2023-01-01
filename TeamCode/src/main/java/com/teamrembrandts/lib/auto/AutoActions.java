package com.teamrembrandts.lib.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.teamrembrandts.lib.math.geometry.Pose2d;
import com.teamrembrandts.lib.math.geometry.Rotation2d;
import com.teamrembrandts.lib.math.geometry.Translation2d;
import com.teamrembrandts.lib.math.kinematics.ChassisSpeeds;
import com.teamrembrandts.lib.math.path.DeadWheelOdometry;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;

public class AutoActions {
    DeadWheelOdometry odometry;
    Drivetrain drivetrain;

    public AutoActions(HardwareMap hardwareMap) {
        odometry = new DeadWheelOdometry(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
    }

    public void driveToPoint(Translation2d target, double endDistance, double maxSpeed) {
        try {
            while (odometry.updatePose().getTranslation().getDistance(target) > endDistance) {
                Translation2d movementVector = target.minus(odometry.getCurrentPose().getTranslation());

                System.out.println("dx: " + movementVector.getX());
                System.out.println("dy: " + movementVector.getY());

                double movementVectorLength = target.getDistance(odometry.getCurrentPose().getTranslation());

                double vx = movementVector.getX() / movementVectorLength * maxSpeed;
                double vy = movementVector.getY() / movementVectorLength * maxSpeed;

                System.out.println("vx: " + vx);
                System.out.println("vy: " + vy);

                drivetrain.drive(new ChassisSpeeds(vx, vy, 0));
            }
        } catch (Exception e) {

        }
        drivetrain.drive(new ChassisSpeeds(0,0,0));
    }

    public void turnToPoint(Rotation2d target, double endDistance, double maxSpeed) {
        try {
            while (odometry.updatePose().getRotation().getAngle(target) > endDistance) {
                Rotation2d movementVector = target.minus(odometry.getCurrentPose().getRotation());

               // System.out.println("dx: " + movementVector.getX());
                System.out.println("dr: " + movementVector.getRadians());

                double movementVectorLength = target.getAngle(odometry.getCurrentPose().getRotation());

               // double vx = movementVector.getX() / movementVectorLength * maxSpeed;
                //double vy = movementVector.getY() / movementVectorLength * maxSpeed;
                double vr = movementVector.getRadians() / movementVectorLength * maxSpeed;

                //System.out.println("vx: " + vx);
                System.out.println("vr: " + vr);

                drivetrain.drive(new ChassisSpeeds(0, 0, vr));
            }
        } catch (Exception e) {

        }
        drivetrain.drive(new ChassisSpeeds(0,0,0));
    }




}
