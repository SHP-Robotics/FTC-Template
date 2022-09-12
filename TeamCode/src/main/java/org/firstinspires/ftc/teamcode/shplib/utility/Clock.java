package org.firstinspires.ftc.teamcode.shplib.utility;

import com.qualcomm.robotcore.util.ElapsedTime;

public final class Clock {
    private static ElapsedTime timer;

    public static void start() {
        timer = new ElapsedTime();
    }

    public static double seconds() {
        return timer.seconds();
    }

    public static boolean hasElapsed(double startTime, double seconds) {
        return seconds() - startTime >= seconds;
    }

    public static void reset() {
        timer.reset();
    }
}
