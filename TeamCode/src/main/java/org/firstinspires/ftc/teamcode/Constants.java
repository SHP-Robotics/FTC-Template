package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.shplib.controllers.FFController;

@Config
public class Constants {
    // Target voltage for voltage compensation
    public static final double kNominalVoltage = 12.0;

    public static final class Drive {
        public static final String[] kMotorNames = new String[]{
                "leftFront",
                "leftRear",
                "rightFront",
                "rightRear"
        };
        public static final FFController[] kFFs = new FFController[]{
                new FFController(0.047),
                new FFController(0.07),
                new FFController(0.045),
                new FFController(0.035)
        };
        public static final double kMinimumBias = 0.3;
        public static final double kMaximumBias = 0.6;
    }

    public static final class Vision {
        public static final double kTagsizeMeters = 0.0475;
    }

    public static final class Arm {
        // Servo Names
        public static final String kClawName = "claw";
        public static final String kSwingLeftName = "swingLeft";
        public static final String kSwingRightName = "swingRight";
        public static final String kCounterSwingLeftName = "counterSwingLeft";
        public static final String kCounterSwingRightName = "counterSwingRight";
        // Servo Positions
        public static final double kClawOpen = 0.4;
        public static final double kClawClosed = 0.7;

        // do not believe any of these values, i made them all up - roshan
        public static final double kSwingLeftUp = 0.0;
        public static final double kSwingRightUp = 1.0;
        public static final double kCounterSwingLeftUp = 0.0;
        public static final double kCounterSwingRightUp = 1.0;
        public static final double kSwingLeftDown = 0.5;
        public static final double kSwingRightDown = 0.5;
        public static final double kCounterSwingLeftDown = 0.5;
        public static final double kCounterSwingRightDown = 0.5;

        // Linear Slide Constants
        public static final String kLinearSlideName = "linearSlide";
        public static final double kSlideBottom = 10.0;
        public static final double kSlideHub = 200.0;
        public static final double kSlideLow = 1000.0;
        public static final double kSlideMiddle = 2500.0;
        public static final double kSlideHigh = 4000.0;
        public static final double kSlideStackDistance = 150.0;

        public static final double kSlideP = 0.0018;
        public static final double kSlideD = 0;
        public static final double kSlideTolerance = 100;

        public static final double kSlideS = 0.035; // static friction
        public static final double kSlideG = 0.07; // gravity
    }
}
