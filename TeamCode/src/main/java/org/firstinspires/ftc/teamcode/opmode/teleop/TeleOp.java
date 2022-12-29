/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.teamrembrandts.lib.math.geometry.Pose2d;
import com.teamrembrandts.lib.math.geometry.Rotation2d;
import com.teamrembrandts.lib.math.geometry.Translation2d;
import com.teamrembrandts.lib.math.kinematics.ChassisSpeeds;
import com.teamrembrandts.lib.math.path.DeadWheelOdometry;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.*;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Weekend2", group="Iterative Opmode")
public class TeleOp extends OpMode
{
    Drivetrain drivetrain;
    DeadWheelOdometry odometry;

    Pose2d currentPose;

    //TODO zut weghalen
    DcMotor leftEncoder;
    DcMotor rightEncoder;
    DcMotor perpendicularEncoder;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        odometry = new DeadWheelOdometry(hardwareMap);
        currentPose = new Pose2d(new Translation2d(0,0), new Rotation2d(0));

        // TODO nog meer zut om weg te halen
        leftEncoder = hardwareMap.get(DcMotor.class, "left-encoder");
        rightEncoder = hardwareMap.get(DcMotor.class, "right-encoder");
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "perp-encoder");

        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        currentPose = odometry.updatePose();

        drivetrain.drive(
                new ChassisSpeeds(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x
                )
        );

        telemetry.addData("encoder left", leftEncoder.getCurrentPosition() * Constants.TICKS_TO_CENTIMETERS);
        telemetry.addData("encoder right", rightEncoder.getCurrentPosition() * Constants.TICKS_TO_CENTIMETERS);
        telemetry.addData("encoder perp", perpendicularEncoder.getCurrentPosition() * Constants.TICKS_TO_CENTIMETERS);
        telemetry.addData("x", currentPose.getTranslation().getX());
        telemetry.addData("y", currentPose.getTranslation().getY());
        telemetry.addData("r", currentPose.getRotation().getDegrees());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
