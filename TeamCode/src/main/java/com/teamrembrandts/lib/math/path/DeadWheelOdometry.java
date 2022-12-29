package com.teamrembrandts.lib.math.path;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.teamrembrandts.lib.math.geometry.Pose2d;
import com.teamrembrandts.lib.math.geometry.Rotation2d;
import com.teamrembrandts.lib.math.geometry.Translation2d;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class DeadWheelOdometry {
    DcMotor leftEncoder;
    DcMotor rightEncoder;
    DcMotor perpendicularEncoder;

    double previousLeftEncoderPos = 0;
    double previousRightEncoderPos = 0;
    double previousPerpendicularEncoderPos = 0;

    Pose2d currentPose = new Pose2d(new Translation2d(0,0), new Rotation2d(0));

    public DeadWheelOdometry(HardwareMap hardwareMap) {
        leftEncoder = hardwareMap.get(DcMotorEx.class, "left-encoder");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "right-encoder");
        perpendicularEncoder = hardwareMap.get(DcMotorEx.class, "perp-encoder");

        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Pose2d updatePose() {
        double leftEncoderPos = leftEncoder.getCurrentPosition() * TICKS_TO_CENTIMETERS;
        double rightEncoderPos = rightEncoder.getCurrentPosition() * TICKS_TO_CENTIMETERS;
        double perpEncoderPos = perpendicularEncoder.getCurrentPosition() * TICKS_TO_CENTIMETERS;

        double deltaLeftEncoderPos =  leftEncoderPos - previousLeftEncoderPos;
        double deltaRightEncoderPos = rightEncoderPos - previousRightEncoderPos;
        double deltaPerpEncoderPos = perpEncoderPos - previousPerpendicularEncoderPos;

        double phi = (deltaLeftEncoderPos - deltaRightEncoderPos) / TRACK_WIDTH;
        double deltaMiddlePos = (deltaLeftEncoderPos + deltaRightEncoderPos) / 2;
        double deltaPerpPos = deltaPerpEncoderPos - FORWARD_OFFSET * phi;

        double heading = currentPose.getRotation().getRadians();

        double deltaX = deltaMiddlePos * Math.cos(heading) - deltaPerpPos * Math.sin(heading);
        double deltaY = deltaMiddlePos * Math.sin(heading) + deltaPerpPos * Math.cos(heading);

        currentPose = currentPose.add(
                new Pose2d(
                        new Translation2d(deltaX, deltaY),
                        new Rotation2d(phi)
                )
        );

        previousLeftEncoderPos = leftEncoderPos;
        previousRightEncoderPos = rightEncoderPos;
        previousPerpendicularEncoderPos = perpEncoderPos;

        return currentPose;
    }

    public Pose2d getCurrentPose() {
        return currentPose;
    }
}
