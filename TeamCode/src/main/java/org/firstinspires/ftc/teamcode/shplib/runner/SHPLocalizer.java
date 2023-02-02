package org.firstinspires.ftc.teamcode.shplib.runner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.shplib.hardware.sensors.SHPDistanceSensor;

public class SHPLocalizer {
    private final SampleMecanumDrive wheelOdometry;
    private final DistanceSensorOdometry distanceOdometry;

    public SHPLocalizer(HardwareMap hardwareMap, SHPDistanceSensor side, SHPDistanceSensor rear) {
        this.wheelOdometry = new SampleMecanumDrive(hardwareMap);
        this.distanceOdometry = new DistanceSensorOdometry(side, rear);
    }

    public void update() {
        Pose2d distancePose = distanceOdometry.getPoseEstimate(-wheelOdometry.getRawExternalHeading());
        if (distancePose == null) wheelOdometry.updatePoseEstimate();
        else setPoseEstimate(distancePose);

        // if (distancePose isnt wack compared to getPoseEstimate()) {
        //  wheelOdometry.setPoseEstimate(distancePose);
        // } else {
        //  wheelOdometry.updatePoseEstimate();
        // }
    }

    public Pose2d getPoseEstimate() {
        return wheelOdometry.getPoseEstimate();
    }

    public void setPoseEstimate(Pose2d pose) {
        wheelOdometry.setPoseEstimate(pose);
    }

    public void log(Telemetry telemetry) {
        distanceOdometry.log(telemetry, wheelOdometry.getRawExternalHeading());

        Pose2d pose = getPoseEstimate();
        telemetry.addData("Pose X: ", pose.getX());
        telemetry.addData("Pose Y: ", pose.getY());
        telemetry.addData("Pose Heading: ", pose.getHeading());

        telemetry.addData("Yaw (rads): ", -wheelOdometry.getRawExternalHeading());
        telemetry.addData("Yaw (degs): ", Math.toDegrees(-wheelOdometry.getRawExternalHeading()));

    }
}
