import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Robot: TeleOp POV",group ="Robot")
public class SecondFTCProgram extends LinearOpMode {
 public DcMotor leftForward = null;
 public DcMotor rightForward = null;
 public DcMotor leftBack = null;
 public DcMotor rightBack = null;
 public DcMotor arm = null;

 public Servo claw = null;

 double clawOffset = 0;

 public static final double MID_SERVO   =  0.5 ;

 public static final double CLAW_SPEED = 0.04;

    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double armPower;
        leftForward = hardwareMap.get(DcMotor.class, "leftForward");
        rightForward = hardwareMap.get(DcMotor.class,"rightForward");

        leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        rightBack = hardwareMap.get(DcMotor.class,"rightBack");

        arm = hardwareMap.get(DcMotor.class, "arm");

        claw = hardwareMap.get(Servo.class, "hand");

        leftForward.setDirection(DcMotor.Direction.REVERSE);
        rightForward.setDirection(DcMotor.Direction.FORWARD);

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        arm.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.left_stick_x;
            armPower = gamepad1.right_stick_y *.5;

            boolean ClawOpen = gamepad1.left_bumper;
            boolean ClawClose = gamepad1.right_bumper;

            left = drive + turn;
            right = drive - turn;

            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            arm.setPower(armPower);

            leftForward.setPower(left);
            rightForward.setPower(right);

            leftBack.setPower(left);
            rightBack.setPower(right);

            arm.setPower(armPower);
            if (ClawClose)
                clawOffset += CLAW_SPEED;
            else if (ClawOpen)
                clawOffset -= CLAW_SPEED;

            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            claw.setPosition(MID_SERVO + clawOffset);


            telemetry.addData("left", "%.2f", left);
            telemetry.addData("left", "%.2f", left);

            telemetry.update();

            sleep(50);

        }
    }
}

