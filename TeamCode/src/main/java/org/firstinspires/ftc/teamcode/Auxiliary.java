package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Auxiliary
{
    private final DcMotorEx launcher, grabber, intake;
    private final Servo grabberServo, launcherServo, triggerServo, ammoServo;
    private final DistanceSensor lowerDS, upperDS;
    private final FtcDashboard dashboard;
    private int intakeSpeed, arm;
    private boolean grabberPos, triggerPos, shootBusy;

    public Auxiliary(HardwareMap hardwareMap)
    {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcherServo = hardwareMap.get(Servo.class, "launcherServo");
        triggerServo = hardwareMap.get(Servo.class, "triggerServo");
        triggerServo.setPosition(0.25);
        ammoServo = hardwareMap.get(Servo.class, "ammoServo");
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

    public void shoot()
    {
        if (shootBusy)
            return;
        shootBusy = true;
        launcher.setPower(1);
        ammoServo.setPosition(1);
        Global.setTimeout(() ->
        {
            triggerServo.setPosition(0.7);
        }, 2000);
        Global.setTimeout(() ->
        {
            launcher.setVelocity(0);
            triggerServo.setPosition(0.25);
            ammoServo.setPosition(0.55);
            shootBusy = false;
        }, 3500);
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
        if (intakeSpeed == 0)
            intakeSpeed = 1;
        else intakeSpeed = 0;
        intake.setPower(intakeSpeed);
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
        if (lowerDS.getDistance(DistanceUnit.CM) < 20 && upperDS.getDistance(DistanceUnit.CM) < 20)
            return 'C';
        else if (lowerDS.getDistance(DistanceUnit.CM) < 20 && upperDS.getDistance(DistanceUnit.CM) > 20)
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
