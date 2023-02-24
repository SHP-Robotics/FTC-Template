package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.drive.SHPMecanumDrive;
import org.firstinspires.ftc.teamcode.shplib.hardware.sensors.SHPIMU;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class DriveSubsystem extends Subsystem {
    //    private final RRMecanumDrive rr;
    private final SHPMecanumDrive drive;
    public final SHPIMU imu;
    public final Encoder parallelEncoder, perpendicularEncoder;

    double strafe = 0;
    double vert = 0;
    private double bias = 1.0;

    public DriveSubsystem(HardwareMap hardwareMap) {
//        rr = new RRMecanumDrive(hardwareMap, Constants.Drive.kMotorNames);
        drive = new SHPMecanumDrive(hardwareMap, Constants.Drive.kMotorNames);

        for (int i = 0; i<4; i++)
            drive.motors[i].enablePositionPID(Constants.Drive.K_DRIVE_P);

        // Change AxesOrder and AxesSigns according to your hub orientation
        // Omit Axes arguments for standard orientation
        imu = new SHPIMU(hardwareMap, AxesOrder.ZYX, AxesSigns.PPN);

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));

        //parallelEncoder.setDirection(Encoder.Direction.REVERSE);
        //perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public void mecanum(double leftY, double leftX, double rightX) {
        vert = leftY;
        if (Math.abs(leftX)<0.5)
            strafe = 0;
        else
            strafe = leftX;
        Vector2d vector = new Vector2d(
                vert,
                strafe
        ).rotated(-imu.getYaw()+PoseStorage.offset);




        drive.mecanum(-bias*vector.getY(), bias*vector.getX(), bias*0.8*rightX); // field oriented
    }


    public void setDriveBias(double driveBias, double add) {
        bias = Range.clip(driveBias, 0.1+add, 0.55+add);
    }

    public void normalmecanum(double leftY, double leftX, double rightX) {
        drive.mecanum(leftY, leftX, rightX); // robot oriented
    }

    @Override
    public void periodic(Telemetry telemetry) {
//        telemetry.addData("heading: ", Math.toDegrees(imu.getYaw()));
        telemetry.addData("leftFront: ", drive.motors[0].getPosition(MotorUnit.TICKS));
        telemetry.addData("leftRear: ", drive.motors[1].getPosition(MotorUnit.TICKS));
        telemetry.addData("rightFront: ", drive.motors[2].getPosition(MotorUnit.TICKS));
        telemetry.addData("rightRear: ", drive.motors[3].getPosition(MotorUnit.TICKS));

    }

}
