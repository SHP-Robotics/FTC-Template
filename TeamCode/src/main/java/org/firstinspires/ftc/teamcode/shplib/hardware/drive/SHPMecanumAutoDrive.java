package org.firstinspires.ftc.teamcode.shplib.hardware.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class SHPMecanumAutoDrive extends SHPMecanumDrive {
    private final double[] setpoints;

    public SHPMecanumAutoDrive(HardwareMap hardwareMap, String[] motorNames, double kP, double errorTolerance) {
        super(hardwareMap, motorNames);
        enablePositionPID(kP, errorTolerance);
        this.setpoints = getWheelPositions(MotorUnit.TICKS);
    }

    public void follow() {
        setPositions(setpoints);
    }

    public void translateBy(double leftFront, double leftRear, double rightFront, double rightRear) {
        setpoints[0] -= leftFront;
        setpoints[1] -= leftRear;
        setpoints[2] -= rightFront;
        setpoints[3] -= rightRear;
    }

    public void translateBy(double[] positions) {
        for (int i = 0; i < setpoints.length; i++) {
            setpoints[i] -= positions[i];
        }
    }
}
