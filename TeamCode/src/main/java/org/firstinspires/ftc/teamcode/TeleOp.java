package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Drive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode
{
    private Drive drive;
    private Auxiliary auxiliary;
    private boolean x, y, a, b;

    @Override
    public void runOpMode() throws InterruptedException
    {

        FtcDashboard.start();

        drive = new Drive(hardwareMap);
        auxiliary = new Auxiliary(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested())
        {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.3,
                            -gamepad1.left_stick_x * 0.3,
                            -(gamepad1.right_trigger - gamepad1.left_trigger) * 0.3
                    )
            );

            drive.update();
            updateKeys1();

            drive.debugEncoders(telemetry);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

        FtcDashboard.stop();
    }

    private void updateKeys1()
    {
        if (gamepad1.x && x)
        {
            drive.arm(72, 5);
            x = true;
        } else if (!gamepad1.x)
            x = false;

        if (gamepad1.y && y)
        {
            auxiliary.shoot();
            y = true;
        } else if (!gamepad1.y)
            y = false;

        if (gamepad1.a && a)
        {
            drive.arm(72, 37);
            a = true;
        } else if (gamepad1.a)
            a = false;

        if (gamepad1.b && b)
        {
            auxiliary.toggleGrabber();
            b = true;
        } else if (gamepad1.b)
            b = false;
    }
}
