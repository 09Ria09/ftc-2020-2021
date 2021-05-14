package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Drive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "drive")
public class Autonomous extends LinearOpMode
{
    private Drive drive;
    private Auxiliary auxiliary;
    private final Global global = new Global();
    private final boolean releasedWobble = false;
    private char caseABC;

    @Override
    public void runOpMode() throws InterruptedException
    {
        drive = new Drive(hardwareMap);
        auxiliary = new Auxiliary(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(-62, -50, 0));

        waitForStart();
        if (isStopRequested())
            return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-62, -50, 0))
                .splineTo(new Vector2d(-32, -55), Math.toRadians(50))
                .build();
        drive.followTrajectory(traj);
        caseABC = auxiliary.detectCase();
        double x = 0, y = 0, h = 0;
        switch (caseABC)
        {
            case 'A':
                x = 32;
                y = -60;
                h = Math.toRadians(-90);
                break;
            case 'B':
                x = 36;
                y = -55;
                h = Math.toRadians(180);
                break;
            case 'C':
                x = 40;
                y = -56;
                h = Math.toRadians(90);
                break;
        }
        traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(x, y), h)
                .build();
        drive.followTrajectory(traj);

        //auxiliary.toggleGrabber();

        Thread.sleep(1000);

        //auxiliary.toggleGrabber();

        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-2, -35), Math.toRadians(20))
                .build());

        /*
        drive.arm(72, -37);
        auxiliary.shoot();
        drive.arm(72, -38);
        auxiliary.shoot();
        drive.arm(72, -39);
        auxiliary.shoot();

         */

        if (caseABC != 'A')
        {
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(
                    drive.trajectoryBuilder(
                            drive.getPoseEstimate())
                            .splineTo(new Vector2d(-30, -36), Math.toRadians(180))
                            .splineTo(new Vector2d(-2, -35), Math.toRadians(20))
                            .build());
        }

        if (caseABC == 'C')
        {
            /*
            drive.arm(72, -37);
            auxiliary.shoot();
            drive.arm(72, -38);
            auxiliary.shoot();
            drive.arm(72, -39);
            auxiliary.shoot();

             */
        } else if (caseABC == 'B')
        {
            /*
            drive.arm(72, -37);
            auxiliary.shoot();

             */
        }

        drive.followTrajectory(
                drive.trajectoryBuilder(
                        drive.getPoseEstimate())
                        .splineTo(new Vector2d(10, -60), Math.toRadians(0))
                        .build());

        lastPose = drive.getPose();
    }
}
