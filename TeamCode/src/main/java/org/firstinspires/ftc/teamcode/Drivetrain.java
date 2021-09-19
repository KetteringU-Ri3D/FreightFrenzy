package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Drive base with mecanum wheels
 */
public class Drivetrain {
    /**
     * Allows for mecanum drive control using the following scheme:
     *      Left Y - Forward/Backward
     *      Left X - Left/Right Strafe
     *      Right X - Left/Right Rotation
     * @param frontLeft     front left motor
     * @param frontRight    front right motor
     * @param backLeft      back left motor
     * @param backRight     back right motor
     * @param leftY         current Y value of the left joystick
     * @param leftX         current X value of the left joystick
     * @param rightX        current X value of the right joystick
     */
    public void drive(DcMotor frontLeft, DcMotor frontRight,
                      DcMotor backLeft, DcMotor backRight,
                      double leftY, double leftX, double rightX) {

        // ensure all motors get the same power
        double denominator = Math.max(Math.abs(leftY) + Math.abs(leftX) + Math.abs(rightX), 1);

        // calculate the power of each motor
        double powerFrontLeft = (leftY + leftX + rightX) / denominator;
        double powerFrontRight = (leftY - leftX - rightX) / denominator;
        double powerBackLeft = (leftY - leftX + rightX) / denominator;
        double powerBackRight = (leftY + leftX - rightX) / denominator;

        // set the power of each motor
        frontLeft.setPower(powerFrontLeft);
        frontRight.setPower(powerFrontRight);
        backLeft.setPower(powerBackLeft);
        backRight.setPower(powerBackRight);
    }
}
