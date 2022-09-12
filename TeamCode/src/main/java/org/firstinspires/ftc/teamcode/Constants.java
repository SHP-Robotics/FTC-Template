package org.firstinspires.ftc.teamcode;

public final class Constants {
    // Target voltage for voltage compensation
    public static final double kNominalVoltage = 12.0;

    public static final double kScoopTop = 0.5;
    public static final double kScoopMiddle = 0.9;
    public static final double kScoopBottom = 1.0;

    public static final class Arm {
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kTolerance = 10;

        public static final double kUpperBound = 1000;
        public static final double kLowerBound = 0;
    }
}
