package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

import androidx.annotation.NonNull;

@Config
public class Localizer extends ThreeTrackingWheelLocalizer
{
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.98425; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 15.4330709;//15.0394;//15.2756;//15.74;//10.3299;//15;//16.4526;//9.26; //14.822835; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -8;//-7.5;//-8.1;//-4;//-4.7; // in; offset of the lateral wheel

    private final Encoder leftEncoder, rightEncoder, centerEncoder;

    private final FtcDashboard dashboard;

    public Localizer(HardwareMap hardwareMap)
    {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        centerEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight"));
        // centerEncoder.setDirection(Encoder.Direction.REVERSE);
        dashboard = FtcDashboard.getInstance();
    }

    public static double encoderTicksToInches(double ticks)
    {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions()
    {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(centerEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities()
    {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(centerEncoder.getCorrectedVelocity())
        );
    }

    public void debugEncoders()
    {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("leftEncoder", leftEncoder.getCurrentPosition());
        packet.put("rightEncoder", rightEncoder.getCurrentPosition());
        packet.put("frontEncoder", centerEncoder.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);
    }
}
