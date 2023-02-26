package org.firstinspires.ftc.teamcode.shplib.hardware.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.roadrunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.roadrunner.util.BNO055IMUUtil;

public class SHPIMU {
    private final BNO055IMU imu;

    public SHPIMU(HardwareMap hardwareMap, AxesOrder order, AxesSigns signs) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, order, signs);
    }

    public SHPIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void initialize() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public double getYaw() {
        return -imu.getAngularOrientation().firstAngle;
    }
    public double getAutoYaw() {
        return imu.getAngularOrientation().angleUnit.getUnnormalized().toDegrees(getYaw());
    }



/**
 * This method returns a value of the Z axis of the REV Expansion Hub IMU.
 * It transforms the value from (-180, 180) to (-inf, inf).
 * This code was taken and modified from https://ftcforum.usfirst.org/forum/ftc-technology/53477-rev-imu-questions?p=53481#post53481.
 * @return The integrated heading on the interval (-inf, inf).
 */
        private double previousHeading = 0; //Outside of method
        private double integratedHeading = 0;
        private double getIntegratedHeading() {
        double currentHeading = imu.getAngularOrientation().firstAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }



}
