package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Auxiliary
{
    final double yTurretOffset = 20, turretTicksPerRotation = 50000,
            maxTurretAngle = 30, minTurretAngle = 30;

    private final DcMotorEx launcher, grabber, intake, turret;
    private final DigitalChannel launcherSensor;
    private final Servo grabberServo0, grabberServo1, triggerServo, intakeServo;
    private final CRServo ammoServo;
    private final DistanceSensor lowerDS, upperDS, crSensor;
    private final FtcDashboard dashboard;
    private final double ammoHome = 0.96;
    public boolean shootBusy, auto;
    private int intakeSpeed, arm = 0;
    private boolean grabberPos, triggerPos;

    public Auxiliary(HardwareMap hardwareMap, boolean auto)
    {
        this.auto = auto;

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        triggerServo = hardwareMap.get(Servo.class, "triggerServo");

        ammoServo = hardwareMap.get(CRServo.class, "ammoServo");

        grabber = hardwareMap.get(DcMotorEx.class, "grabber");
        if (auto)
        {
            grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        grabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabberServo0 = hardwareMap.get(Servo.class, "grabberServo0");
        grabberServo0.setDirection(Servo.Direction.FORWARD);
        grabberServo1 = hardwareMap.get(Servo.class, "grabberServo1");
        grabberServo1.setDirection(Servo.Direction.FORWARD);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        lowerDS = hardwareMap.get(DistanceSensor.class, "lowerDS");
        upperDS = hardwareMap.get(DistanceSensor.class, "upperDS");

        launcherSensor = hardwareMap.get(DigitalChannel.class, "launcherSensor");
        crSensor = hardwareMap.get(DistanceSensor.class, "crSensor");
        launcherSensor.setMode(DigitalChannel.Mode.INPUT);

        dashboard = FtcDashboard.getInstance();

        launcher.setPower(0);
        triggerServo.setPosition(0.56);
        //ammoServo.setPosition(ammoHome);
        //ammoHome();

        grabber.setTargetPosition(arm);
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber.setPower(1);
        grabberServo0.setPosition(0);
        grabberServo1.setPosition(0);
        grabberPos = true;
        intakeServo.setPosition(1);
    }

    /*
    public void updateAmmo()
    {
        if (intakeSpeed == 1)
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
    */

    public void updateTurret(Pose2d pose)
    {
        pose.plus(new Pose2d(0, yTurretOffset, 0));
        double angle = Global.GetAngleOfLineBetweenTwoPoints(pose.getX(), pose.getX(), 72, -5);
        angle = Math.max(angle, maxTurretAngle);
        angle = Math.min(angle, minTurretAngle);

        turret.setTargetPosition((int) ((turretTicksPerRotation / 2) * angle));
    }

    /*public void preloadAmmo(int n)
    {
        ammoServo.setPosition(getAmmoOffset(n));
    }*/

    public void setLauncher(double n)
    {
        launcher.setPower(n);
    }

    public void shoot(int n)
    {
        if (shootBusy)
            return;
        shootBusy = true;
        new Thread(() ->
        {
            boolean isr = false;
            if (!auto)
            {
                launcher.setPower(1);
                if (intakeSpeed != 0)
                {
                    toggleIntake();
                    isr = true;
                }
                useIntakeServo();
                try
                {
                    Thread.sleep(300);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
            for (int i = n; i > 0; --i)
            {
                //ammoServo.setPosition(getAmmoOffset(i));
                prepareAmmo();
                if (i != n)
                {
                    try
                    {
                        Thread.sleep(1000);
                    } catch (InterruptedException e)
                    {
                        e.printStackTrace();
                    }
                }
                triggerServo.setPosition(0.35);
                try
                {
                    Thread.sleep(300);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
                triggerServo.setPosition(0.56);
                try
                {
                    Thread.sleep(300);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }

            if (!auto)
            {
                //ammoServo.setPosition(ammoHome);
                try
                {
                    Thread.sleep(300);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
                launcher.setPower(0);

                ammoHome();
                if (isr)
                    toggleIntake();
            }
            shootBusy = false;
        }).start();
    }

    private double getAmmoOffset(int level)
    {
        switch (level)
        {
            case 1:
                return 0.615;
            case 2:
                return 0.65;
            case 3:
                return 0.68;
            default:
                return ammoHome;
        }
    }

    public void prepareAmmo()
    {
        ammoServo.setPower(-1);
        while (crSensor.getDistance(DistanceUnit.CM) > 1.1)
        {
            try
            {
                Thread.sleep(10);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
        ammoServo.setPower(0);
    }

    public void ammoHome()
    {
        ammoServo.setPower(1);
        while (launcherSensor.getState())
        {
            try
            {
                Thread.sleep(10);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
        ammoServo.setPower(0);
    }

    public void toggleGrabber()
    {
        grabberPos = !grabberPos;
        if (grabberPos)
        {
            grabberServo0.setPosition(0);
            grabberServo1.setPosition(0);
        } else
        {
            grabberServo0.setPosition(1);
            grabberServo1.setPosition(1);
        }
    }

    public void toggleIntake()
    {
        if (shootBusy)
            return;
        if (intakeSpeed == 0)
            intakeSpeed = 1;
        else intakeSpeed = 0;
        intake.setPower(intakeSpeed);
    }

    public void useIntakeServo()
    {
        new Thread(() ->
        {
            intakeServo.setPosition(0.61);
            try
            {
                Thread.sleep(600);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            intakeServo.setPosition(1);
        }).start();
    }

    public void toggleIntakeDirection()
    {
        if (intake.getDirection() == DcMotorEx.Direction.FORWARD)
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        else intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void toggleArm()
    {
        if (arm == 680)
            arm = 0;
        else arm = 680;
        grabber.setTargetPosition(arm);
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber.setPower(1);
    }

    public char detectCase()
    {
        int cutOff = 9;
        if (lowerDS.getDistance(DistanceUnit.CM) < cutOff && upperDS.getDistance(DistanceUnit.CM) < cutOff)
            return 'C';
        else if (lowerDS.getDistance(DistanceUnit.CM) < cutOff && upperDS.getDistance(DistanceUnit.CM) > cutOff)
            return 'B';
        else return 'A';
    }

    public void debug()
    {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("turret", turret.getCurrentPosition());
        packet.put("grabber", grabber.getCurrentPosition());
        packet.put("lowerDS", lowerDS.getDistance(DistanceUnit.CM));
        packet.put("upperDS", upperDS.getDistance(DistanceUnit.CM));
        packet.put("sensor1", launcherSensor.getState());
        packet.put("cr", crSensor.getDistance(DistanceUnit.CM));
        dashboard.sendTelemetryPacket(packet);
    }
}
