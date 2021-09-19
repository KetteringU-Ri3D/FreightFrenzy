package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class BaseOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        // create class objects
        Drivetrain drivetrain = new Drivetrain();
        FreightManipulator freightManipulator = new FreightManipulator();
        CarouselManipulator carouselManipulator = new CarouselManipulator();

        // define drive motors on the control hub using the following map
        // driveFrontLeft = port 1
        // driveFrontRight = port 0
        // driveBackLeft = port 3
        // driveBackRight = port 2
        DcMotor driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        DcMotor driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        DcMotor driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        DcMotor driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        // define manipulator motors on the expansion hub using the following map
        // freightManipulatorLeft = 0
        // freightManipulatorRight = 1
        // freightManipulatorFourBar = 2
        // carouselManipulatorMotor = 3
        DcMotor freightManipulatorLeft = hardwareMap.dcMotor.get("freightManipulatorLeft");
        DcMotor freightManipulatorRight = hardwareMap.dcMotor.get("freightManipulatorRight");
        DcMotor freightManipulatorFourBar = hardwareMap.dcMotor.get("freightManipulatorFourBar");
        DcMotor carouselManipulatorMotor = hardwareMap.dcMotor.get("carouselManipulatorMotor");

        // reverse the motors on the right side of the drivetrain
        driveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // wait for the match to start
        waitForStart();

        // run while the OpMode is active
        while(opModeIsActive()) {
            // get the current value of each joystick axis
            double leftY = gamepad1.left_stick_y;
            double leftX = -gamepad1.left_stick_x;
            double rightX = -gamepad1.right_stick_x;

            // use mecanum drive control
            drivetrain.drive(driveFrontLeft, driveFrontRight, driveBackLeft, driveBackRight,
                    leftY, leftX, rightX);

            // create a button map
            // freight manipulator controls:
            // left bumper = outtake, right bumper = intake
            if(gamepad1.right_bumper == true) {
                freightManipulator.intake(freightManipulatorLeft, freightManipulatorRight, 0.75);
            }
            else if(gamepad1.left_bumper == true) {
                freightManipulator.outtake(freightManipulatorLeft, freightManipulatorRight, 0.75);
            }
            else {
                freightManipulator.stopCollector(freightManipulatorLeft, freightManipulatorRight);
            }

            // freight manipulator four-bar controls:
            // left trigger = lower, right trigger = raise, neither = stop
            if(gamepad1.left_trigger >= 0.5) {
                freightManipulator.raise(freightManipulatorFourBar, 0.5);
            }
            else if(gamepad1.left_trigger >= 0.5) {
                freightManipulator.lower(freightManipulatorFourBar, 0.5);
            }
            else {
                freightManipulator.stopFourBar(freightManipulatorFourBar);
            }

            // carousel manipulator controls:
            // a = forward (duck towards the field), b = reverse (duck away from the field)
            if(gamepad1.a == true) {
                carouselManipulator.forward(carouselManipulatorMotor, 0.25);
            }
            else if(gamepad1.b == true) {
                carouselManipulator.reverse(carouselManipulatorMotor, 0.25);
            }
        }
    }
}
