package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Auxiliary
{
    final double yTurretOffset = 20, turretTicksPerRotation = 50000,
            maxTurretAngle = 30, minTurretAngle = 30;

    private final DcMotorEx launcher, grabber, intake, turret;
    private final Servo grabberServo, launcherServo, triggerServo, ammoServo;
    private final DistanceSensor lowerDS, upperDS, ammoSensor;
    private final FtcDashboard dashboard;
    private int intakeSpeed, arm, ammo;
    private boolean grabberPos, triggerPos, shootBusy;

    public Auxiliary(HardwareMap hardwareMap)
    {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcherServo = hardwareMap.get(Servo.class, "launcherServo");
        triggerServo = hardwareMap.get(Servo.class, "triggerServo");
        triggerServo.setPosition(0.25);
        ammoServo = hardwareMap.get(Servo.class, "ammoServo");
        ammoSensor = hardwareMap.get(DistanceSensor.class, "ammoSensor");
        grabber = hardwareMap.get(DcMotorEx.class, "grabber");
        grabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabberServo = hardwareMap.get(Servo.class, "grabberServo");
        grabberServo.setPosition(0.53);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        lowerDS = hardwareMap.get(DistanceSensor.class, "lowerDS");
        upperDS = hardwareMap.get(DistanceSensor.class, "upperDS");
        dashboard = FtcDashboard.getInstance();
    }

    public void updateAmmo()
    {
        if (intakeSpeed == 1 && ammoSensor.getDistance(DistanceUnit.CM) < 5)
        {
            ++ammo;
            ammo = Math.max(ammo, 3);
            if (ammo == 3)
            {
                toggleIntake();
                shoot();
            }
            ammoServo.setPosition(getAmmoOffset(ammo));
        }
    }

    public void updateTurret(Pose2d pose)
    {
        pose.plus(new Pose2d(0, yTurretOffset, 0));
        double angle = Global.GetAngleOfLineBetweenTwoPoints(pose.getX(), pose.getX(), 72, -5);
        angle = Math.max(angle, maxTurretAngle);
        angle = Math.min(angle, minTurretAngle);

        turret.setTargetPosition((int) ((turretTicksPerRotation / 2) * angle) );
    }

    public void shoot()
    {
        if (shootBusy)
            return;
        new Thread(() ->
        {
            shootBusy = true;
            intakeSpeed = 0;
            intake.setPower(0);
            launcher.setPower(1);
            while (ammo > 0)
            {
                ammoServo.setPosition(1 + getAmmoOffset(ammo));
                triggerServo.setPosition(0.7);
                try
                {
                    Thread.sleep(300);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
                --ammo;
                triggerServo.setPosition(0.2);
                try
                {
                    Thread.sleep(300);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
            shootBusy = false;
            toggleIntake();
        }).start();
    }

    private int getAmmoOffset(int level)
    {
        switch (level)
        {
            case 1:
                return -2;
            case 2:
                return -1;
            case 3:
                return 0;
            default:
                return -3;
        }
    }

    public void toggleGrabber()
    {
        grabberPos = !grabberPos;
        if (!grabberPos)
            grabberServo.setPosition(0.53);

        else if (grabberPos)
            grabberServo.setPosition(0);
    }

    public void toggleIntake()
    {
        if (intakeSpeed == 0 && ammo < 3 && !shootBusy)
            intakeSpeed = 1;
        else intakeSpeed = 0;
        intake.setPower(intakeSpeed);
    }

    public void toggleIntakeDirection()
    {
        if (intake.getDirection() == DcMotorEx.Direction.FORWARD)
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        else intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void toggleArm()
    {
        if (arm == 0)
            arm = 1250;
        else arm = 0;
        grabber.setTargetPosition(arm);
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public char detectCase()
    {
        int cutOff = 20;
        if (lowerDS.getDistance(DistanceUnit.CM) < cutOff && upperDS.getDistance(DistanceUnit.CM) < cutOff)
            return 'C';
        else if (lowerDS.getDistance(DistanceUnit.CM) < cutOff && upperDS.getDistance(DistanceUnit.CM) > cutOff)
            return 'B';
        else return 'A';
    }

    public void debug()
    {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("triggerServo", triggerServo.getPosition());
        packet.put("launcherServo", launcherServo.getPosition());
        packet.put("grabber", grabber.getCurrentPosition());
        packet.put("grabberServo", grabberServo.getPosition());
        packet.put("lowerDS", lowerDS.getDistance(DistanceUnit.CM));
        packet.put("upperDS", upperDS.getDistance(DistanceUnit.CM));
        dashboard.sendTelemetryPacket(packet);
    }
}
