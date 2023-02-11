package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.shplib.controllers.FFController;

public final class Constants {
    // Target voltage for voltage compensation
    public static final double kNominalVoltage = 12.0;

    // Multiplied by each term before assigning to the controller
    public static final double K_POSITION_PID_FACTOR = 1.0 / 10000.0;
    public static final double K_VELOCITY_PID_FACTOR = 1.0 / 1000.0;

    public static final class Drive {
        public static final String[] kMotorNames = new String[]{
                "leftFront",
                "leftRear",
                "rightFront",
                "rightRear"
        };
        public static final double K_DRIVE_P = 5;
        public static final FFController[] kFFs = new FFController[]{
                new FFController(0.047),
                new FFController(0.07),
                new FFController(0.045),
                new FFController(0.035)
        };
        public static final double kMinimumBias = 0.4;
    }

    public static final class Vision {
        public static final double kTagsizeMeters = 0.039;
    }

    public static final class Arm {
        public static final double OFFSET = 0;
        public static final String K_SLIDE_NAME = "slide";

        public static final double K_SLIDE_P = 13; // 15 best value
        public static final double K_SLIDE_TOLERANCE = 0.05;
        public static final double K_SLIDE_MAX_VELOCITY = 1;
        public static final double K_SLIDE_G = 0.11; //THIS MIGHT BE AN ISSUE
        //"Didn't turn out to be" - Aarav
        //TODO: Tune G
        public static final double K_SLIDE_TOP = 3900 - OFFSET;
        public static final double K_SLIDE_MIDDLE = 2750 - OFFSET;
        public static final double K_SLIDE_SHORT = 1600 - OFFSET;
        public static final double K_SLIDE_CARRY = 250 - OFFSET;
        public static final double K_SLIDE_BOTTOM = 10 - OFFSET;

    }

    public static final class Scoop {
        public static final String K_CLAW_NAME = "claw";
        public static final double K_OUT = 1;
        public static final double K_IN = 0;
    }
}
