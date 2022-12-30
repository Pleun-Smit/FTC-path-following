package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    }

    public void drive(ChassisSpeeds targetSpeeds) {
        motorFrontLeft.setPower(-targetSpeeds.getVX() + targetSpeeds.getVY() - targetSpeeds.getVR());
        motorFrontRight.setPower(targetSpeeds.getVX() + targetSpeeds.getVY() + targetSpeeds.getVR());
        motorBackLeft.setPower(targetSpeeds.getVX() + targetSpeeds.getVY() - targetSpeeds.getVR());
        motorBackRight.setPower(-targetSpeeds.getVX() + targetSpeeds.getVY() + targetSpeeds.getVR());
    }
}
