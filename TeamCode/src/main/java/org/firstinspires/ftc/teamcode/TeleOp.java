package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Drive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends AlteredLinearOpMode
{
    private Drive drive;
    private Auxiliary auxiliary;
    private boolean x, y, a, b, startButton, backButton, leftBumper, rightBumper, rightStickButton;
    private final double armX = 0;
    private final double armY = 0;
    private double speed = 0.8;


    @Override
    public void runOpMode() throws InterruptedException
    {
        drive = new Drive(hardwareMap);
        auxiliary = new Auxiliary(hardwareMap, false);
        drive.setPoseEstimate(lastPose);
        drive.setPoseEstimate(new Pose2d(-62, -25.5, Math.toRadians(0)));
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested())
        {
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );//.rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * speed,
                            input.getY() * speed,
                            -(gamepad1.right_trigger - gamepad1.left_trigger) * speed
                    )
            );

            drive.update();
            // auxiliary.updateTurret(drive.getPoseEstimate());
            updateKeys1();

            auxiliary.debug();
            //drive.debugEncoders();
        }
    }

    private void updateKeys1()
    {
        if (gamepad1.x && !x)
        {
            auxiliary.useIntakeServo();
            x = true;
        } else if (!gamepad1.x)
            x = false;

        if (gamepad2.y && !y)
        {
            auxiliary.shoot(3);
            y = true;
        } else if (!gamepad2.y)
            y = false;

        if (gamepad2.a && !a)
        {
            auxiliary.toggleArm();
            a = true;
        } else if (!gamepad2.a)
            a = false;

        if (gamepad2.b && !b)
        {
            auxiliary.toggleGrabber();
            b = true;
        } else if (!gamepad2.b)
            b = false;

        if (gamepad1.start && !startButton)
        {
            auxiliary.toggleIntake();
            startButton = true;
        } else if (!gamepad1.start)
            startButton = false;

        if (gamepad1.back && !backButton)
        {
            auxiliary.toggleIntakeDirection();
            backButton = true;
        } else if (!gamepad1.back)
            backButton = false;

        if (gamepad1.left_bumper && !leftBumper)
        {
            speed = 0.5;
            leftBumper = true;
        } else if (!gamepad1.left_bumper)
            leftBumper = false;

        if (gamepad1.right_bumper && !rightBumper)
        {
            speed = 0.8;
            rightBumper = true;
        } else if (!gamepad1.right_bumper)
            rightBumper = false;

        if (gamepad2.right_stick_x < -0.75 && Math.abs(gamepad2.right_stick_y) < 0.5 && gamepad2.right_stick_button && !rightStickButton)
        {
            auxiliary.shoot(1);
            rightStickButton = true;
        } else if (gamepad2.right_stick_x > 0.75 && Math.abs(gamepad2.right_stick_y) < 0.5 && gamepad2.right_stick_button && !rightStickButton)
        {

            auxiliary.shoot(3);
            rightStickButton = true;
        } else if (Math.abs(gamepad2.right_stick_x) < 0.5 && gamepad2.right_stick_y < -0.75 && gamepad2.right_stick_button && !rightStickButton)
        {

            auxiliary.shoot(2);
            rightStickButton = true;
        } else if (!gamepad2.right_stick_button)
            rightStickButton = false;

        /*
        if (gamepad1.right_stick_x < -0.75)
        {
            armX = 72;
            armY = -35;
        } else if (gamepad1.right_stick_x > 0.75)
        {
            armX = 72;
            armY = -36;
        } else if (gamepad1.right_stick_button)
        {
            armX = 72;
            armY = -37;
        }
        */
    }
}
