package org.firstinspires.ftc.teamcode.shplib.hardware.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.shplib.controllers.FFController;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class SHPFourWheelDrive {
    final SHPMotor[] motors;
    double maxVelocity = 0;

    public SHPFourWheelDrive(HardwareMap hardwareMap, String[] motorNames) {
        // 0 -> left front
        // 1 -> left rear
        // 2 -> right front
        // 3 -> right rear
        motors = new SHPMotor[4];
        for (int i = 0; i < motors.length; i++) {
            motors[i] = new SHPMotor(hardwareMap, motorNames[i]);
        }
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void enableFF(FFController ff) {
        for (SHPMotor motor : motors) {
            motor.enableFF(ff);
        }
    }

    public void enablePositionPID(double kP, double errorTolerance) {
        for (SHPMotor motor : motors) {
            motor.enablePositionPID(kP);
            motor.setPositionErrorTolerance(errorTolerance);
        }
    }

    public void enablePositionPID(double kP, double kI, double kD, double errorTolerance) {
        for (SHPMotor motor : motors) {
            motor.enablePositionPID(kP, kI, kD);
            motor.setPositionErrorTolerance(errorTolerance);
        }
    }

    public void setPositions(double position) {
        for (SHPMotor motor : motors) {
            motor.setPosition(position);
        }
    }

    public void setPositions(double[] position) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPosition(position[i]);
        }
    }

    public boolean atPositionSetpoint() {
        return motors[0].atPositionSetpoint()
                && motors[1].atPositionSetpoint()
                && motors[2].atPositionSetpoint()
                && motors[3].atPositionSetpoint();

    }

    public void enableVelocityPID(double kP, double maxVelocity) {
        for (SHPMotor motor : motors) {
            motor.enableVelocityPID(kP);
        }
        this.maxVelocity = maxVelocity;
    }

    public void enableVelocityPID(double kP, double kI, double kD, double maxVelocity) {
        for (SHPMotor motor : motors) {
            motor.enableVelocityPID(kP, kI, kD);
        }
        this.maxVelocity = maxVelocity;
    }

    public void setAllPowers(double powers) {
        for (int i = 0; i < motors.length; i++) {
            setPower(i, powers);
        }
    }

    public void setAllPowers(double[] powers) {
        for (int i = 0; i < motors.length; i++) {
            setPower(i, powers[i]);
        }
    }

    public void setAllVelocities(double[] powers) {
        for (int i = 0; i < motors.length; i++) {
            setVelocity(i, powers[i]);
        }
    }

    public void setPower(int index, double power) {
        motors[index].setPower(power);
    }

    public void setVelocity(int index, double power) {
        motors[index].setVelocity(power);
    }

    public void reverseAll() {
        for (SHPMotor motor : motors) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void enableBuiltInVelocityControl() {
        for (SHPMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double[] getWheelPowers() {
        return new double[]{
                motors[0].getPower(),
                motors[1].getPower(),
                motors[2].getPower(),
                motors[3].getPower()
        };
    }

    public double[] getWheelVelocities(MotorUnit unit) {
        return new double[]{
                motors[0].getVelocity(unit),
                motors[1].getVelocity(unit),
                motors[2].getVelocity(unit),
                motors[3].getVelocity(unit)
        };
    }

    public double[] getWheelPositions(MotorUnit unit) {
        return new double[]{
                motors[0].getPosition(unit),
                motors[1].getPosition(unit),
                motors[2].getPosition(unit),
                motors[3].getPosition(unit)
        };
    }

    public double getWheelPositionsAveraged(MotorUnit unit) {
        return (motors[0].getPosition(unit) +
                motors[1].getPosition(unit) +
                motors[2].getPosition(unit) +
                motors[3].getPosition(unit)) / 4;
    }
}
