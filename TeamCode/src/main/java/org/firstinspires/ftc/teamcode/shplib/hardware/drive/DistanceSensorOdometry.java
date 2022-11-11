package org.firstinspires.ftc.teamcode.shplib.hardware.drive;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorOdometry {
    private DistanceSensor sensorX, sensorY;
    private double offsetX, offsetY;
    private Mounting mountX, mountY;

    public enum Mounting {
        FRONT, REAR, LEFT, RIGHT
    }

    public DistanceSensorOdometry(DistanceSensor sensorY, double offsetY, Mounting mountY) {
        this.sensorY = sensorY;
        this.offsetY = offsetY;
        this.mountY = mountY;
    }

    public DistanceSensorOdometry(DistanceSensor sensorX, DistanceSensor sensorY,
                                  double offsetX, double offsetY,
                                  Mounting mountX, Mounting mountY) {
        this.sensorX = sensorX;
        this.sensorY = sensorY;
        this.offsetX = offsetX;
        this.offsetY = offsetY;
        this.mountX = mountX;
        this.mountY = mountY;
    }

    // need to fix theta calculations using mounting

    public double getThetaX(double heading) {
        if (mountY == Mounting.FRONT) return Math.abs(heading - 270);
        else if (mountY == Mounting.REAR) return Math.abs(heading - 90);
        else return 0;
    }

    public double getThetaY(double heading) {
        if (mountX == Mounting.LEFT) return Math.abs(heading - 360);
        else if (mountX == Mounting.RIGHT) return Math.abs(heading - 180);
        else return 0;
    }

    public double getXCorrected(double heading) {
        return (offsetX + sensorX.getDistance(DistanceUnit.INCH)) * Math.cos(getThetaX(heading));
    }

    public double getYCorrected(double heading) {
        return (offsetY + sensorY.getDistance(DistanceUnit.INCH)) * Math.cos(getThetaY(heading));
    }
}
