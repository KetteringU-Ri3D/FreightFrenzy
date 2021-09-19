package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * The primary autonomous mode for the robot
 */
@Autonomous
public class AutonomousOpMode extends LinearOpMode {
    // create an IMU object
    BNO055IMU imu;

    // state used for updating telemetry
    Orientation angles;

    // define variables for later use
    // encoder pulses per revolution at the gearbox output shaft
    final double PULSES_PER_REV = 537.7;

    // gear reduction on the drive motors
    final double DRIVE_GEAR_REDUCTION = 19.2;

    // diameter of the wheel (96mm goBILDA mecanum)
    final double WHEEL_DIAMETER_INCHES = 3.78;

    final double PULSES_PER_INCH =
            (PULSES_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // speed for various forms of movement, keeping it slow to account for wheel slipping
    final double FORWARD_SPEED = 0.5;
    final double ROTATION_SPEED = 0.5;
    final double STRAFE_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        // create objects for mechanisms
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

        // reverse the motors on the right side of the drivetrain
        driveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // define manipulator motors on the expansion hub using the following map
        // freightManipulatorLeft = 0
        // freightManipulatorRight = 1
        // freightManipulatorFourBar = 2
        // carouselManipulatorMotor = 3
        DcMotor freightManipulatorLeft = hardwareMap.dcMotor.get("freightManipulatorLeft");
        DcMotor freightManipulatorRight = hardwareMap.dcMotor.get("freightManipulatorRight");
        DcMotor freightManipulatorFourBar = hardwareMap.dcMotor.get("freightManipulatorFourBar");
        DcMotor carouselManipulatorMotor = hardwareMap.dcMotor.get("carouselManipulatorMotor");

        // reset encoders
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // reset drive to run using encoder mode
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set the IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // get the IMU information and initialize it
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // set up telemetry dashboard
        composeTelemetry();

        // wait for the user to start the robot
        waitForStart();

        // start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // TEST
        driveDistance(FORWARD_SPEED, 24);
        strafeDistance(STRAFE_SPEED, 24);

        // NOT WORKING, UNSURE WHY. ANGLES DO NOT GET UPDATED AT ALL BUT ARE UPDATED IN
        // THE BNO055IMU EXAMPLE CODE
        // turnToAngle(ROTATION_SPEED, 90);
    }

    /**
     * Drive forward a distance (in inches) during autonomous. Forward is positive,
     * backward is negative
     * @param speed         how fast the robot will go (0.0 to 1.0)
     * @param target        how far the robot will travel
     */
    public void driveDistance(double speed, int target) {
        // create a drivetrain object
        Drivetrain drivetrain = new Drivetrain();

        // define drive motors on the control hub using the following map
        // driveFrontLeft = port 1
        // driveFrontRight = port 0
        // driveBackLeft = port 3
        // driveBackRight = port 2
        DcMotor driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        DcMotor driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        DcMotor driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        DcMotor driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        // reverse the motors on the right side of the drivetrain
        driveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // set the target for each motor
        int targetFrontLeft = driveFrontLeft.getCurrentPosition() + (int)(target + PULSES_PER_INCH);
        int targetFrontRight = driveFrontRight.getCurrentPosition() + (int)(target + PULSES_PER_INCH);
        int targetBackLeft = driveBackLeft.getCurrentPosition() + (int)(target + PULSES_PER_INCH);
        int targetBackRight = driveBackRight.getCurrentPosition() + (int)(target + PULSES_PER_INCH);

        // set the target position for each motor
        driveFrontLeft.setTargetPosition(-targetFrontLeft);
        driveFrontRight.setTargetPosition(-targetFrontRight);
        driveBackLeft.setTargetPosition(-targetBackLeft);
        driveBackRight.setTargetPosition(-targetBackRight);

        // set all motors to run to position
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion
        driveFrontLeft.setPower(Math.abs(speed));
        driveFrontRight.setPower(Math.abs(speed));
        driveBackLeft.setPower(Math.abs(speed));
        driveBackRight.setPower(Math.abs(speed));

        // run while the OpMode is active and motors are busy
        while(opModeIsActive() && driveFrontLeft.isBusy() && driveBackLeft.isBusy() &&
              driveFrontRight.isBusy() && driveBackRight.isBusy()) {
            telemetry.update();
        }

        // stop all motion
        driveFrontLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackLeft.setPower(0);
        driveBackRight.setPower(0);

        // reset encoders
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // reset drive to run using encoder mode
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Strafe a distance (in inches) during autonomous. Right is positive, Left is negative
     * @param speed     how fast the robot will go (0.0 to 1.0)
     * @param target    how far the robot will travel
     */
    public void strafeDistance(double speed, int target) {
        // create a drivetrain object
        Drivetrain drivetrain = new Drivetrain();

        // define drive motors on the control hub using the following map
        // driveFrontLeft = port 1
        // driveFrontRight = port 0
        // driveBackLeft = port 3
        // driveBackRight = port 2
        DcMotor driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        DcMotor driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        DcMotor driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        DcMotor driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        // reverse the motors on the right side of the drivetrain
        driveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // set the target for each motor
        int targetFrontLeft = driveFrontLeft.getCurrentPosition() + (int)(target + PULSES_PER_INCH);
        int targetFrontRight = driveFrontRight.getCurrentPosition() + (int)(target + PULSES_PER_INCH);
        int targetBackLeft = driveBackLeft.getCurrentPosition() + (int)(target + PULSES_PER_INCH);
        int targetBackRight = driveBackRight.getCurrentPosition() + (int)(target + PULSES_PER_INCH);

        // set the target position for each motor
        driveFrontLeft.setTargetPosition(-targetFrontLeft);
        driveFrontRight.setTargetPosition(targetFrontRight);
        driveBackLeft.setTargetPosition(targetBackLeft);
        driveBackRight.setTargetPosition(-targetBackRight);

        // set all motors to run to position
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion
        driveFrontLeft.setPower(Math.abs(speed));
        driveFrontRight.setPower(Math.abs(speed));
        driveBackLeft.setPower(Math.abs(speed));
        driveBackRight.setPower(Math.abs(speed));

        // run while the OpMode is active and motors are busy
        while(opModeIsActive() && driveFrontLeft.isBusy() && driveBackLeft.isBusy() &&
                driveFrontRight.isBusy() && driveBackRight.isBusy()) {
            telemetry.update();
        }

        // stop all motion
        driveFrontLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackLeft.setPower(0);
        driveBackRight.setPower(0);

        // reset encoders
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // reset drive to run using encoder mode
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnToAngle(double speed, double target) {
        // create a drivetrain object
        Drivetrain drivetrain = new Drivetrain();

        // define drive motors on the control hub using the following map
        // driveFrontLeft = port 1
        // driveFrontRight = port 0
        // driveBackLeft = port 3
        // driveBackRight = port 2
        DcMotor driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        DcMotor driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        DcMotor driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        DcMotor driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        // reverse the motors on the right side of the drivetrain
        driveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // start motion. turn left if angle is greater than the target angle, turn right if
        // the angle is less than the target angle
        while(opModeIsActive() && target != angles.firstAngle) {
            if(angles.firstAngle < target) {
                driveFrontLeft.setPower(-speed);
                driveFrontRight.setPower(speed);
                driveBackLeft.setPower(-speed);
                driveBackRight.setPower(speed);
            }
            else if(angles.firstAngle > target) {
                driveFrontLeft.setPower(speed);
                driveFrontRight.setPower(-speed);
                driveBackLeft.setPower(speed);
                driveBackRight.setPower(-speed);
            }
            telemetry.update();
        }

        // stop all motion
        driveFrontLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackLeft.setPower(0);
        driveBackRight.setPower(0);
    }

    void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
