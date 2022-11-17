package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Drive.*;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.drive.SHPMecanumDrive;
import org.firstinspires.ftc.teamcode.shplib.hardware.sensors.SHPIMU;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class DriveSubsystem extends Subsystem {
    //    private final RRMecanumDrive rr;
    private final SHPMecanumDrive drive;
    private final SHPIMU imu;

    private double bias = 1.0; // will always be between 0.0 and 1.0

    public DriveSubsystem(HardwareMap hardwareMap) {
//        rr = new RRMecanumDrive(hardwareMap, kMotorNames);
        drive = new SHPMecanumDrive(hardwareMap, kMotorNames);

        // Change AxesOrder and AxesSigns according to your hub orientation
        // Omit Axes arguments for standard orientation
        imu = new SHPIMU(hardwareMap, AxesOrder.ZYX, AxesSigns.PPN);
//        imu = new SHPIMU(hardwareMap);
    }

    public void mecanum(double leftY, double leftX, double rightX) {
//        Vector2d vector = new Vector2d(
//                leftY,
//                leftX
//        ).rotated(-imu.getYaw());

//        drive.mecanum(vector.getX(), vector.getY(), rightX); // field oriented
        drive.mecanum(leftY * bias, leftX * bias, rightX * bias); // robot oriented
    }

    public void setDriveBias(double driveBias) {
        bias = Range.clip(driveBias, kMinimumBias, 1.0);
    }

//    public void enablePositionPID() {
//        drive.enablePositionPID(Constants.Drive.kP, 0, Constants.Drive.kD, 50);
//    }
//
//    public void driveTo(boolean usingPID, double ticks) {
//        if (usingPID) {
//            drive.setPosition(ticks);
//        } else {
//            while (drive.getWheelPositionsAveraged(MotorUnit.TICKS) <= ticks) {
//                drive.mecanum(0.5, 0.5, 0);
//            }
//            drive.mecanum(0.0, 0.0, 0);
//        }
//    }

//    public boolean atPositionSetpoint() {
//        return drive.atPositionSetpoint();
//    }

    @Override
    public void periodic(Telemetry telemetry) {
//        telemetry.addData("Bot Direction: ", Math.toDegrees(imu.getYaw()));
        for (int i = 0; i < 4; i++) {
            telemetry.addData("Motor " + i + " Position: ", drive.getWheelPositions(MotorUnit.TICKS)[i]);
        }
//        telemetry.addData("Drive at position setpoint: ", drive.atPositionSetpoint() ? "true" : "false");
    }
}
