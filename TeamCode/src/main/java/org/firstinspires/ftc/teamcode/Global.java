package org.firstinspires.ftc.teamcode;

public class Global
{

    public static void setTimeout(Runnable runnable, int delay)
    {
        new Thread(() ->
        {
            try
            {
                Thread.sleep(delay);
                runnable.run();
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }).start();
    }

    public static double GetAngleOfLineBetweenTwoPoints(double x1, double y1, double x2, double y2)
    {
        double xDiff = x2 - x1;
        double yDiff = y2 - y1;
        return Math.atan2(yDiff, xDiff);
    }
}
