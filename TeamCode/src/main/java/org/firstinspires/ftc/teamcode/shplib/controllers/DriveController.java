package org.firstinspires.ftc.teamcode.shplib.controllers;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.drive.TwoWheelTrackingLocalizer;

public class DriveController {
    PositionPID y_pid;
    PositionPID x_pid;
    PositionPID heading_pid;
    TwoWheelTrackingLocalizer localizer;



    public DriveController(PositionPID x_pid, PositionPID y_pid, PositionPID heading_pid, TwoWheelTrackingLocalizer localizer) {
        this.y_pid = y_pid;
        this.x_pid = x_pid;
        this.heading_pid = heading_pid;
        this.localizer = localizer;
    }

    public void setInitialPose(Pose2d pose) {
        y_pid.setInitialPosition(pose.getY());
        x_pid.setInitialPosition(pose.getX());
        heading_pid.setInitialPosition(pose.getHeading());
    }

    public void setCurrentPose() {
        Pose2d pose = localizer.getPoseEstimate();
        y_pid.setCurrentPosition(pose.getY());
        x_pid.setCurrentPosition(pose.getX());
        heading_pid.setCurrentPosition(pose.getHeading());
    }

//    public Pose2d getRobotError() {
//        if (robotError != null)
//            return robotError;
//        else
//            return null;
//    }

    public Pose2d calculate(Pose2d setpoint) {
        //Vector2d robot_setpoint = setpoint.vec().rotated(-field_pose.getHeading());
        double x_power = x_pid.calculate(setpoint.getX());
        double y_power = y_pid.calculate(setpoint.getY());
        double heading_power = heading_pid.calculate(setpoint.getHeading());
        return new Pose2d(x_power, y_power, heading_power);

    }
}