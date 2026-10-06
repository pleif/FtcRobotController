package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name = "FlywheelTestBiobuzz")
public class FlywheelTestBiobuzz extends LinearOpMode {




    private DcMotorEx Josh = null;
    private DcMotor bottomCollection = null;
    private DcMotor topCollection = null;


    public FlywheelTestBiobuzz(HardwareMap hardwareMap, OpMode opMode) {
        Josh = theOpMode.hardwareMap.get(DcMotorEx.class, "Josh");

    }


    public void teleOpControls() {


        if (gamepad2.x || gamepad1.x ) {
            bottomCollection.setPower(-1);
            topCollection.setPower(-1);
        } else if (gamepad2.left_bumper) {
            bottomCollection.setPower(1)
            ;
            topCollection.setPower(.8);
        } else {
            bottomCollection.setPower(0);
            topCollection.setPower(0);
        }

        if (theOpMode.gamepad1.right_bumper) {
            Josh.setVelocity(-1500);
        } else if (theOpMode.gamepad1.dpad_up) {

            Josh.setVelocity(Josh.getVelocity() -100);
        } else if (theOpMode.gamepad1.dpad_down) {
        Josh.setVelocity(Josh.getVelocity() + 100);

        }
        theOpMode.telemetry.addData("currentVelocity", Josh.getVelocity());




    }


    @Override
    public void runOpMode() throws InterruptedException {
        teleOpControls();
    }


}
