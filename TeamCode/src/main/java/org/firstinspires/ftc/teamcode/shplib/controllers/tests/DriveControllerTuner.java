package org.firstinspires.ftc.teamcode.shplib.controllers.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.shplib.controllers.DriveController;
import org.firstinspires.ftc.teamcode.shplib.controllers.PositionPID;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

@TeleOp
public class DriveControllerTuner extends BaseRobot {
    DriveController controller;
    PositionPID xPid, yPid, turnPid;
    TwoWheelTrackingLocalizer localizer;
    double xGain = 100;
    double yGain = 100;
    double turnGain = 1300;

    Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
    Pose2d setpoint1 = new Pose2d(new Vector2d(0, 0), 0);
    Pose2d setpoint2 = new Pose2d(new Vector2d(0, 10), Math.toRadians(90));
    Pose2d setpoint3 = new Pose2d(new Vector2d(10, 0), Math.toRadians(90));
    Pose2d setpoint = initialPose;

    private double debounce;
    @Override
    public void init() {
        super.init();
        localizer = new TwoWheelTrackingLocalizer(hardwareMap);
        localizer.setPoseEstimate(initialPose);

        xPid = new PositionPID(xGain, localizer.getPoseEstimate().getX());
        yPid = new PositionPID(yGain, localizer.getPoseEstimate().getY());
        turnPid = new PositionPID(turnGain, localizer.getHeading());

        controller = new DriveController(xPid, yPid, turnPid, localizer);
        controller.setInitialPose(initialPose);

    }

    @Override
    public void start() {
        super.start();
        debounce = Clock.now();
    }

    @Override
    public void loop() {
        // Allows CommandScheduler.run() to be called - DO NOT DELETE!
        super.loop();
        localizer.update();

        if (gamepad1.a && Clock.hasElapsed(debounce, 0.5)) {
            setpoint = setpoint1;
            controller.setInitialPose(localizer.getPoseEstimate());
            debounce = Clock.now();
        } else if (gamepad1.b && Clock.hasElapsed(debounce, 0.5)) {
            setpoint = setpoint2;
            controller.setInitialPose(localizer.getPoseEstimate());
            debounce = Clock.now();
        } else if (gamepad1.x && Clock.hasElapsed(debounce, 0.5)) {
            setpoint = setpoint3;
            controller.setInitialPose(localizer.getPoseEstimate());
            debounce = Clock.now();
        } else if (gamepad1.y && Clock.hasElapsed(debounce, 0.5)) {
            setpoint = initialPose;
            controller.setInitialPose(localizer.getPoseEstimate());
            debounce = Clock.now();
        }

        Pose2d currentPosition = localizer.getPoseEstimate();

        controller.setCurrentPose();
        Pose2d powers = controller.calculate(setpoint);
        drive.mecanum(powers.getY(), powers.getX(), powers.getHeading());

//        Pose2d errors = controller.getRobotError();

        telemetry.addData("current x: ", currentPosition.getX());
        telemetry.addData("current y: ", currentPosition.getY());
        telemetry.addData("current heading: ", currentPosition.getHeading());
//        telemetry.addData("x error: ", errors.getX());
//        telemetry.addData("y error: ", errors.getY());
//        telemetry.addData("heading error: ", errors.getHeading());
        telemetry.addData("set x: ", setpoint.getX());
        telemetry.addData("set y: ", setpoint.getY());
        telemetry.addData("set heading: ", setpoint.getHeading());
        telemetry.addData("x power: ", powers.getX());
        telemetry.addData("y power: ", powers.getY());
        telemetry.addData("rotation power: ", powers.getHeading());
        //telemetry.addData("Current Position Left Front: ", leftFront.getPosition(MotorUnit.TICKS));
//        telemetry.addData("Initial Position: ", pid.getInitialPosition());
//        telemetry.addData("Setpoint: ", setpoint);
//        telemetry.addData("Output: ", output);
//        pid.logGains(telemetry);
    }
}
