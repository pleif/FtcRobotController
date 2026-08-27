package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Demobot",group ="Robot")
public class Demobot extends LinearOpMode {
    public DcMotor leftDrive   = null;
    public DcMotor  rightDrive  = null;


    @Override
    public void runOpMode() throws InterruptedException {

        double left;
        double right;
        double drive;
        double turn;
        double max;
        double armPower;

        leftDrive  = hardwareMap.get(DcMotor.class, "left_Drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_Drive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive = -gamepad1.left_stick_y;
            turn  =  -gamepad1.right_stick_x;

            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left = max;
                right = max;
            }

            // Output the safe vales to the motor drives.
            leftDrive.setPower(left);
            rightDrive.setPower(right);

            sleep(50);

            }



        }


    }



