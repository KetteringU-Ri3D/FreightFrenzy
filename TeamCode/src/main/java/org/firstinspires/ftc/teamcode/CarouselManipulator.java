package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Class designed for the Carousel manipulation mechanism
 */
public class CarouselManipulator {
    /**
     * Spins motors inwards to collect Carousel
     * @param motor  motor on the motor side of the manipulator
     * @param power voltage to apply to the motors
     */
    public void forward(DcMotor motor, double power) {
        // spin the motor clockwise
        motor.setPower(power);
    }

    /**
     * Spins motors outwards to eject Carousel
     * @param motor  motor on the motor side of the manipulator
     * @param power voltage to apply to the motors
     */
    public void reverse(DcMotor motor, double power) {
        // spin the motor counter-clockwise
        motor.setPower(-power);
    }

    /**
     * Stop the motor
     * @param motor  motor on the motor side of the manipulator
     */
    public void stop(DcMotor motor) {
        // stop the motor
        motor.setPower(0);
    }
}
