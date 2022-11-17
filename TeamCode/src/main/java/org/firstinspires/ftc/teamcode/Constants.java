package org.firstinspires.ftc.teamcode;

public final class Constants {
    // Target voltage for voltage compensation
    public static final double kNominalVoltage = 12.0;

    // Multiplied by each term before assigning to the controller
    public static final double kPositionPIDFactor = 1.0 / 100.0;
    public static final double kVelocityPIDFactor = 1.0 / 1000.0;

    public static final class Drive {
        public static final String[] kMotorNames = new String[]{
                "leftFront",
                "leftRear",
                "rightFront",
                "rightRear"
        };
        public static final double kMinimumBias = 0.4;
    }

    public static final class Vision {
        public static final double kTagsizeMeters = 0.0475;
    }

    public static final class Arm {
        public static final String kClawName = "claw";
        public static final double kClawOpen = 0.2;
        public static final double kClawClosed = 0.5;

        public static final String kLeftSlideName = "leftSlide";
        public static final String kRightSlideName = "rightSlide";

        public static final double kSlideBottom = 10.0;
        public static final double kSlideHub = 200.0;
        public static final double kSlideLow = 1000.0;
        public static final double kSlideMiddle = 2500.0;
        public static final double kSlideHigh = 4000.0;
        public static final double kSlideStackDistance = 150.0;

        public static final double kSlideP = 0.17;
        public static final double kSlideD = 0;//kSlideP * 10;//10;
        public static final double kSlideTolerance = 100;

        public static final double kSlideS = 0.035; // static friction
        public static final double kSlideG = 0.07; // gravity
        public static final double kSlideV = 0;
    }

    public static final class Scoop {
        public static final String kScoopName = "scoop";

        public static final double kTop = 0.5;
        public static final double kMiddle = 0.9;
        public static final double kBottom = 1.0;
    }
}
