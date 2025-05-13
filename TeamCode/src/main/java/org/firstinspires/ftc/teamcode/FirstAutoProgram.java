package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Time", group="Robot")

public class FirstAutoProgram extends LinearOpMode {

    public DcMotor leftForward = null;
    public DcMotor rightForward = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;

    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 1.0;
    static final double TURN_SPEED = 0.5;

    static final double COUNTS_PER_MOTOR_REV = 252;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.75;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        leftForward = hardwareMap.get(DcMotor.class, "left_Forward");
        rightForward = hardwareMap.get(DcMotor.class, "right_Forward");

        leftBack = hardwareMap.get(DcMotor.class, "left_Back");
        rightBack = hardwareMap.get(DcMotor.class, "right_Back");

        leftForward.setDirection(DcMotor.Direction.REVERSE);
        rightForward.setDirection(DcMotor.Direction.FORWARD);

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        /*leftForward.setPower(FORWARD_SPEED);
        rightForward.setPower(FORWARD_SPEED);

        leftBack.setPower(FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);

        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();


        }
        leftForward.setPower(0);
        rightForward.setPower(0);

        leftBack.setPower(0);
        rightBack.setPower(0);
     */


     encoderDrive(.5,48,48,4);


    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int LeftFrontTarget;
        int RightFrontTarget;
        int LeftBackTarget;
        int RightBackTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            LeftFrontTarget = leftForward.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            RightFrontTarget = rightForward.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            LeftBackTarget = leftBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            RightBackTarget = rightBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftForward.setTargetPosition(LeftFrontTarget);
            rightForward.setTargetPosition(RightFrontTarget);

            leftBack.setTargetPosition(LeftBackTarget);
            rightBack.setTargetPosition(RightBackTarget);

            // Turn On RUN_TO_POSITION
            leftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftForward.setPower(Math.abs(speed));
            rightForward.setPower(Math.abs(speed));

            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftForward.isBusy() && rightForward.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {


                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", LeftFrontTarget, RightFrontTarget);
                telemetry.addData("Running to", " %7d :%7d", LeftBackTarget, RightBackTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftForward.getCurrentPosition(), rightForward.getCurrentPosition());
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftBack.getCurrentPosition(), rightBack.getCurrentPosition());


                telemetry.update();
            }

            // Stop all motion;
            leftForward.setPower(0);
            rightForward.setPower(0);

            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}