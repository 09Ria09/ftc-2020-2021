package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Drive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "drive")
public class Autonomous3 extends AlteredLinearOpMode
{
    private final Global global = new Global();
    private final boolean releasedWobble = false;
    private final double shootAngle = 0;
    private Drive drive;
    private Auxiliary auxiliary;
    private char caseABC = 'A';

    @Override
    public void runOpMode() throws InterruptedException
    {
        drive = new Drive(hardwareMap);
        auxiliary = new Auxiliary(hardwareMap, true);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(-62, -30.20, 0));
        //auxiliary.preloadAmmo(3);
        //auxiliary.prepareAmmo();

        waitForStart();
        if (isStopRequested())
            return;
        Trajectory trajCase = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-35.5, -28.50), 0)
                .build();
        drive.followTrajectoryAsync(trajCase);
        waitForDrive();
        if (isStopRequested())
            return;
        caseABC = auxiliary.detectCase();

        double x = 0, y = 0, t = 0, h = 0, ox = 0, oy = 0;
        switch (caseABC)
        {
            case 'A':
                x = 13;
                y = -45;
                oy = -7;
                ox = 0;
                t = Math.toRadians(-180);
                h = Math.toRadians(-179.99);
                break;
            case 'B':
                x = 40;
                y = -45;
                oy = -7;
                ox = 0;
                t = Math.toRadians(0);
                h = Math.toRadians(0);
                break;
            case 'C':
                x = 40;
                y = -56;
                oy = 0;
                oy = -7;

                t = Math.toRadians(0);
                h = Math.toRadians(-90);
                break;
        }
        Trajectory trajShoot = drive.trajectoryBuilder(trajCase.end())
                .addDisplacementMarker(() ->
                {
                    auxiliary.setLauncher(1);
                    //auxiliary.preloadAmmo(3);
                    //auxiliary.prepareAmmo();
                })
                .splineToConstantHeading(new Vector2d(-35.5, -20), Math.toRadians(90))
                .splineTo(new Vector2d(-23, -18), Math.toRadians(0))
                .splineTo(new Vector2d(-3, -18), Math.toRadians(0))
                .build();

        Trajectory trajWobble = drive.trajectoryBuilder(trajShoot.end())
                .splineToSplineHeading(new Pose2d(x + ox, y + oy, h), t)
                .build();

        Trajectory trajStrafeLeft = drive.trajectoryBuilder(trajWobble.end()).strafeLeft(-5)
                .build();

        Trajectory trajGoTo2ndWobble = drive.trajectoryBuilder(trajStrafeLeft.end())
                .splineToSplineHeading(new Pose2d(-32, -45, Math.toRadians(90)), Math.toRadians(160))
                //.splineToConstantHeading(new Vector2d(-40, -48), Math.toRadians(0))
                .build();

        Trajectory trajStrafeRight = drive.trajectoryBuilder(trajGoTo2ndWobble.end())
                .strafeLeft(5)
                .build();

        Trajectory trajShoot2 = drive.trajectoryBuilder(trajStrafeRight.end()
                .plus(new Pose2d(0, 0, Global.GetAngleOfLineBetweenTwoPoints(trajGoTo2ndWobble.end().getX(), trajGoTo2ndWobble.end().getY(), 72, 0))))
                .addDisplacementMarker(() ->
                {
                    auxiliary.setLauncher(1);
                    //auxiliary.preloadAmmo(2);
                    //auxiliary.prepareAmmo();
                })
                .splineTo(new Vector2d(-10, -46), Math.toRadians(90))
                .build();

        Trajectory trajWobble2 = drive.trajectoryBuilder(trajShoot2.end())
                .splineToSplineHeading(new Pose2d(x, y, h), 0)
                .build();

        Trajectory trajStrafeLeft2 = drive.trajectoryBuilder(trajWobble2.end()).strafeLeft(-5)
                .build();

        Trajectory trajShoot3 = drive.trajectoryBuilder(trajStrafeLeft2.end())
                .addDisplacementMarker(() ->
                {
                    auxiliary.setLauncher(1);
                    //auxiliary.preloadAmmo(1);
                    //auxiliary.prepareAmmo();
                })
                .splineTo(new Vector2d(-3, -18), Math.toRadians(0))
                .build();

        Trajectory trajPark = drive.trajectoryBuilder(trajShoot3.end())
                .splineTo(new Vector2d(10, -18), Math.toRadians(0))
                .build();

        drive.followTrajectoryAsync(trajShoot);
        waitForDrive();
        if (isStopRequested())
            return;
        drive.arm(72, -2);
        waitForDrive();
        if (isStopRequested())
            return;
        shoot();
        drive.followTrajectoryAsync(trajWobble);
        waitForDrive();
        if (isStopRequested())
            return;
        leaveWobble(trajStrafeLeft);
        drive.followTrajectoryAsync(trajGoTo2ndWobble);
        waitForDrive();
        if (isStopRequested())
            return;
        pickupWobble(trajStrafeRight);
        drive.followTrajectoryAsync(trajShoot2);
        waitForDrive();
        if (isStopRequested())
            return;
        drive.arm(72, -5.54);
        waitForDrive();
        if (isStopRequested())
            return;
        shoot();
        drive.followTrajectoryAsync(trajWobble2);
        waitForDrive();
        if (isStopRequested())
            return;
        leaveWobble(trajStrafeLeft2);
        drive.followTrajectoryAsync(trajShoot3);
        waitForDrive();
        if (isStopRequested())
            return;
        drive.arm(72, -9.08);
        waitForDrive();
        if (isStopRequested())
            return;
        shoot();
        drive.followTrajectoryAsync(trajPark);
        waitForDrive();
        if (isStopRequested())
            return;
        lastPose = drive.getPoseEstimate();
    }

    private void leaveWobble(Trajectory traj)
    {
        auxiliary.toggleArm();
        try
        {
            Thread.sleep(1000);
        } catch (InterruptedException e)
        {
            e.printStackTrace();
        }
        auxiliary.toggleGrabber();
        try
        {
            Thread.sleep(500);
        } catch (InterruptedException e)
        {
            e.printStackTrace();
        }

        drive.followTrajectoryAsync(traj);
        waitForDrive();
        if (isStopRequested())
            return;

        auxiliary.toggleArm();
        auxiliary.toggleGrabber();
    }


    private void pickupWobble(Trajectory traj)
    {
        auxiliary.toggleGrabber();
        auxiliary.toggleArm();
        try
        {
            Thread.sleep(1500);
        } catch (InterruptedException e)
        {
            e.printStackTrace();
        }
        drive.followTrajectoryAsync(traj);
        waitForDrive();
        if (isStopRequested())
            return;
        auxiliary.toggleGrabber();
        try
        {
            Thread.sleep(1000);
        } catch (InterruptedException e)
        {
            e.printStackTrace();
        }
        auxiliary.toggleArm();
    }

    private void shoot()
    {
        auxiliary.shoot(1);
        while (auxiliary.shootBusy && opModeIsActive())
        {
            try
            {
                Thread.sleep(25);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
        auxiliary.setLauncher(0);
    }

    private void waitForDrive()
    {
        drive.update();
        while (drive.isBusy() && opModeIsActive())
        {
            drive.update();
        }
    }
}
