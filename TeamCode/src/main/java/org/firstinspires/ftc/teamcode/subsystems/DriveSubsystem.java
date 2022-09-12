package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.roadrunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.drive.SHPMecanumDrive;
import org.firstinspires.ftc.teamcode.shplib.hardware.sensors.SHPIMU;

public class DriveSubsystem extends Subsystem {
    private final SHPMecanumDrive drive;
    private final SHPIMU imu;

//    private final RoadRunnerMecanumDrive rr;

//    private final VelocityPID controller = new VelocityPID(0.01, 0);

    public DriveSubsystem(HardwareMap hardwareMap) {
        drive = new SHPMecanumDrive(hardwareMap, new String[]{"leftFront", "leftRear", "rightFront", "rightRear"});
//        drive.reverseAll();
//        drive.enableVelocityPID(0.03, 3.0, AngleUnit.RADIANS);

        imu = new SHPIMU(hardwareMap, AxesOrder.ZYX, AxesSigns.PPN);
//        imu = new SHPIMU(hardwareMap);

//        rr = new RoadRunnerMecanumDrive()
    }

    public void mecanum(double leftY, double leftX, double rightX) {
        Vector2d vector = new Vector2d(
                leftY,
                leftX
        ).rotated(-imu.getYaw());
//
        drive.mecanum(vector.getX(), vector.getY(), rightX);
//        drive.mecanum(leftY, leftX, rightX);
    }

    @Override
    public void periodic(Telemetry telemetry) {
//        telemetry.addData("Heading: ", imu.getYaw());
//        telemetry.addData("PID calc: ", controller.calculate(5))
//        telemetry.addData("Left Front Encoder: ", leftFront.getCurrentPosition());
//        telemetry.addData("Left Back Encoder: ", leftBack.getCurrentPosition());
//        telemetry.addData("Right Front Encoder: ", rightFront.getCurrentPosition());
//        telemetry.addData("Right Back Encoder: ", rightBack.getCurrentPosition());
    }
}
