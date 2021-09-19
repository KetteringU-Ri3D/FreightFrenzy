package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Class designed for the Freight manipulation mechanism
 */
public class FreightManipulator {
    /**
     * Spins motors inwards to collect freight
     * @param left  motor on the left side of the manipulator
     * @param right motor on the left side of the manipulator
     * @param power voltage to apply to the motors
     */
    public void intake(DcMotor left, DcMotor right, double power) {
        // spin both motors inward
        left.setPower(power);
        right.setPower(-power);
    }

    /**
     * Spins motors outwards to eject freight
     * @param left  motor on the left side of the manipulator
     * @param right motor on the left side of the manipulator
     * @param power voltage to apply to the motors
     */
    public void outtake(DcMotor left, DcMotor right, double power) {
        // spin both motors outward
        left.setPower(-power);
        right.setPower(power);
    }

    /**
     * Raises the four-bar system
     * @param motor motor that controls the lift
     * @param power voltage to apply to the motor
     */
    public void raise(DcMotor motor, double power) {
        // spin the motor to lift the four-bar
        motor.setPower(power);
    }

    /**
     * Lowers the four-bar system
     * @param motor motor that controls the lift
     * @param power voltage to apply to the motor
     */
    public void lower(DcMotor motor, double power) {
        // spin the motor to lift the four-bar
        motor.setPower(-power);
    }

    /**
     * Stop the motors
     * @param left  motor on the left side of the manipulator
     * @param right motor on the left side of the manipulator
     */
    public void stopCollector(DcMotor left, DcMotor right) {
        // spin both motors inward
        left.setPower(0);
        right.setPower(0);
    }

    /**
     * Stop the motors
     * @param motor motor that controls the lift
     */
    public void stopFourBar(DcMotor motor) {
        // spin both motors inward
        motor.setPower(0);
    }
}
