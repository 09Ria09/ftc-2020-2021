package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Auxiliary
{
    // private final DcMotor launcher, grabber;
    // private final Servo grabberServo;
    // private final DistanceSensor lowerDS, upperDS;

    private int grabberPos;

    public Auxiliary(HardwareMap hardwareMap)
    {
        /*launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber = hardwareMap.get(DcMotorEx.class, "grabber");
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabberServo = hardwareMap.get(Servo.class, "grabberServo");
        lowerDS = hardwareMap.get(DistanceSensor.class, "lowerDS");
        upperDS = hardwareMap.get(DistanceSensor.class, "upperDS");
         */
    }

    public void shoot()
    {
        /*
        launcher.setPower(1);
        Global.setTimeout(()->launcher.setPower(0),1000);
         */
    }

    public void toggleGrabber()
    {
        /*
        grabber.setPower(0.3);
        if(grabberPos==0)
        {
            grabberServo.setPosition(0);
            grabber.setTargetPosition(180);
            grabberPos=180;
        }
        else if(grabberPos==180)
        {
            grabber.setTargetPosition(0);
            Global.setTimeout(()->grabberServo.setPosition(180), 1000);
            grabberPos=0;
        }
         */
    }

    public char detectCase()
    {
        return 'A';
        /*
        int cutOff=20;
        if(lowerDS.getDistance(DistanceUnit.CM)<20 && upperDS.getDistance(DistanceUnit.CM)<20)
            return 'C';
        else if(lowerDS.getDistance(DistanceUnit.CM)<20 && upperDS.getDistance(DistanceUnit.CM)>20)
            return 'B';
        else return 'A';
         */
    }
}
