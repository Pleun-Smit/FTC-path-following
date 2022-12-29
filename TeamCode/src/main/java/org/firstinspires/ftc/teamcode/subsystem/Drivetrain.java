package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.teamrembrandts.lib.math.kinematics.ChassisSpeeds;

public class Drivetrain {
    DcMotorEx motorFrontLeft;
    DcMotorEx motorFrontRight;
    DcMotorEx motorBackLeft;
    DcMotorEx motorBackRight;

    public Drivetrain(HardwareMap hardwareMap) {
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "front-left");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "front-right");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "back-left");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "back-right");

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(ChassisSpeeds targetSpeeds) {
        motorFrontLeft.setPower(-targetSpeeds.getVX() + targetSpeeds.getVY() - targetSpeeds.getVR());
        motorFrontRight.setPower(targetSpeeds.getVX() + targetSpeeds.getVY() + targetSpeeds.getVR());
        motorBackLeft.setPower(targetSpeeds.getVX() + targetSpeeds.getVY() - targetSpeeds.getVR());
        motorBackRight.setPower(-targetSpeeds.getVX() + targetSpeeds.getVY() + targetSpeeds.getVR());
    }
}
