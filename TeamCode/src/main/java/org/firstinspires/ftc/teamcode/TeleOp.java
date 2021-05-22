package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Drive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode
{
    private Drive drive;
    private Auxiliary auxiliary;
    private boolean x, y, a, b, startButton, backButton;
    private final double armX = 0;
    private final double armY = 0;


    @Override
    public void runOpMode() throws InterruptedException
    {
        drive = new Drive(hardwareMap);
        auxiliary = new Auxiliary(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-62.4, -49.6, Math.toRadians(0)));
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested())
        {
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -(gamepad1.right_trigger - gamepad1.left_trigger)
                    )
            );

            drive.update();
            auxiliary.updateAmmo();
            auxiliary.updateTurret(drive.getPoseEstimate());
            updateKeys1();

            //auxiliary.debug();
            //drive.debugEncoders();
        }
    }

    private void updateKeys1()
    {
        if (gamepad1.x && !x)
        {
            auxiliary.toggleIntakeDirection();
            x = true;
        } else if (!gamepad1.x)
            x = false;

        if (gamepad1.y && !y)
        {
            auxiliary.shoot();
            y = true;
        } else if (!gamepad1.y)
            y = false;

        if (gamepad1.a && !a)
        {
            auxiliary.toggleArm();
            a = true;
        } else if (!gamepad1.a)
            a = false;

        if (gamepad1.b && !b)
        {
            auxiliary.toggleGrabber();
            b = true;
        } else if (!gamepad1.b)
            b = false;

        if (gamepad1.back && !backButton)
        {
            auxiliary.toggleIntake();
            backButton = true;
        } else if (!gamepad1.back)
            backButton = false;

        /*
        if (gamepad1.start && !startButton)
        {
            auxiliary.toggleArm();
            startButton = true;
        } else if (!gamepad1.start)
            startButton = false;

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
