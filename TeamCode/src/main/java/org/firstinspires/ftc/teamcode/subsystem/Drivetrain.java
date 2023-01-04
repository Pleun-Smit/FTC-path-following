package org.firstinspires.ftc.teamcode.subsystem;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.teamrembrandts.lib.auto.AutoActions;
import com.teamrembrandts.lib.math.geometry.Pose2d;
import com.teamrembrandts.lib.math.geometry.Rotation2d;
import com.teamrembrandts.lib.math.kinematics.ChassisSpeeds;

public class Drivetrain {
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;

    public Drivetrain(HardwareMap hardwareMap) {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "front-left");
        motorFrontRight = hardwareMap.get(DcMotor.class, "front-right");
        motorBackLeft = hardwareMap.get(DcMotor.class, "back-left");
        motorBackRight = hardwareMap.get(DcMotor.class, "back-right");

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(ChassisSpeeds targetSpeeds, Rotation2d robotAngle, double turnPower) {
        //dit fiksen dan werkt het als het goed is
        //even met de + en - kutten todat het werkt
        // x is positie


        double robotrelativeVX  =  targetSpeeds.getVX() * Math.sin(robotAngle.getRadians()) + targetSpeeds.getVY() * Math.cos(robotAngle.getRadians());
        double robotrelativeVY  =  targetSpeeds.getVY() * Math.cos(robotAngle.getRadians()) + targetSpeeds.getVY() * Math.sin(robotAngle.getRadians());

        motorFrontLeft.setPower(-robotrelativeVX - robotrelativeVY - robotAngle.getRadians());
        motorFrontRight.setPower(-robotrelativeVX + robotrelativeVY + robotAngle.getRadians());
        motorBackLeft.setPower(-robotrelativeVX + robotrelativeVY - robotAngle.getRadians());
        motorBackRight.setPower(-robotrelativeVX - robotrelativeVY + robotAngle.getRadians());
    }
}