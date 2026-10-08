package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "FlywheelTestBiobuzz")
public class FlywheelTestBiobuzz extends LinearOpMode {




    private DcMotorEx Josh = null;
    private DcMotor bottomCollection = null;
    private DcMotor topCollection = null;

    private double desiredFlywheelVelocity;
    private boolean dpadDownPressed = false;
    private boolean dpadUpPressed = false;




    public void teleOpControls() {

        if (gamepad1.x ) {
            bottomCollection.setPower(-1);
            topCollection.setPower(-1);
        } else if (gamepad1.left_bumper) {
            bottomCollection.setPower(1)
            ;
            topCollection.setPower(.8);
        } else {
            bottomCollection.setPower(0);
            topCollection.setPower(0);
        }

        if (gamepad1.right_bumper) {
            desiredFlywheelVelocity = -1500;
        } else if (gamepad1.dpad_up && !dpadUpPressed) {
            telemetry.addData("GamepadEvent","dpadUp");
            dpadUpPressed = true;
            desiredFlywheelVelocity -=50;
            sleep(500);
        } else if (gamepad1.dpad_down && !dpadDownPressed) {
            telemetry.addData("GamepadEvent","dpadDown");
            dpadDownPressed = true;
            desiredFlywheelVelocity +=50;
            sleep(500);
        } else if (gamepad1.b) {

            desiredFlywheelVelocity = 0;
            dpadUpPressed = false;
            dpadDownPressed = false;
        } else {
            dpadUpPressed = false;
            dpadDownPressed = false;
            telemetry.addData("GamepadEvent","NothingPressed");
        }
        Josh.setVelocity(desiredFlywheelVelocity);

        telemetry.addData("DesiredVelocity", desiredFlywheelVelocity);
        telemetry.addData("ActualVelocity", Josh.getVelocity());
        telemetry.update();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Josh = hardwareMap.get(DcMotorEx.class, "Josh");
        bottomCollection = hardwareMap.get(DcMotor.class, "BottomCollection");
        topCollection = hardwareMap.get(DcMotor.class, "TopCollection");
        waitForStart();
        while (opModeIsActive()) {
            teleOpControls();
        }
    }


}
