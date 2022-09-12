package org.firstinspires.ftc.teamcode.shplib.hardware.drive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;

public class SHPFourWheelDrive {
    public final SHPMotor[] motors;

    public SHPFourWheelDrive(HardwareMap hardwareMap, String[] deviceNames) {
        motors = new SHPMotor[4];
        for (int i = 0; i < motors.length; i++) {
            motors[i] = new SHPMotor(hardwareMap, deviceNames[i]);
        }
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
//        leftFront = new SHPMotor(hardwareMap, "leftFront");
//        leftBack = new SHPMotor(hardwareMap, "leftRear");
//        rightFront = new SHPMotor(hardwareMap, "rightFront");
//        rightBack = new SHPMotor(hardwareMap, "rightRear");
    }

    public void enablePositionPID(double kP) {
        for (SHPMotor motor : motors) {
            motor.enablePositionPID(kP);
        }
    }

    public void enablePositionPID(double kP, double kI, double kD) {
        for (SHPMotor motor : motors) {
            motor.enablePositionPID(kP, kI, kD);
        }
    }

    public void enableVelocityPID(double kP, double maxVelocity, AngleUnit unit) {
        for (SHPMotor motor : motors) {
            motor.enableVelocityPID(kP, maxVelocity, unit);
        }
    }

    public void enableVelocityPID(double kP, double kI, double kD, double maxVelocity, AngleUnit unit) {
        for (SHPMotor motor : motors) {
            motor.enableVelocityPID(kP, kI, kD, maxVelocity, unit);
        }
    }

    public void setAll(double[] powers) {
        for (int i = 0; i < motors.length; i++) {
            set(i, powers[i]);
        }
    }

    public void set(int index, double power) {
        motors[index].set(Range.clip(power, -1.0, 1.0));
    }

    public void reverseAll() {
        for (SHPMotor motor : motors) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
}
