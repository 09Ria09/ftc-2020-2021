package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Drive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends AlteredLinearOpMode
{
    private Drive drive;
    private Auxiliary auxiliary;
    private boolean x, y, a, b, startButton, backButton;
    private double armX = 0, armY = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {

        FtcDashboard.start();

        drive = new Drive(hardwareMap);
        auxiliary = new Auxiliary(hardwareMap);
        //drive.setPoseEstimate(new Pose2d(-62.4, -49.6, Math.toRadians(0)));
        drive.setPoseEstimate(lastPose);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested())
        {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -(gamepad1.right_trigger - gamepad1.left_trigger)
                    )
            );

            drive.update();
            updateKeys1();

            //auxiliary.debug();
            //drive.debugEncoders();
        }

        FtcDashboard.stop();
    }

    private void updateKeys1()
    {
        if (gamepad1.x && !x)
        {
            drive.arm(72, -5);
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
            auxiliary.toggleGrabber();
            a = true;
        } else if (!gamepad1.a)
            a = false;

        if (gamepad1.b && !b)
        {
            drive.arm(armX, armY);
            b = true;
        } else if (!gamepad1.b)
            b = false;

        if (gamepad1.start && !startButton)
        {
            auxiliary.toggleArm();
            startButton = true;
        } else if (!gamepad1.start)
            startButton = false;

        if (gamepad1.back && !backButton)
        {
            auxiliary.toggleIntake();
            backButton = true;
        } else if (!gamepad1.back)
            backButton = false;


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

    }
}
