package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.RoadRunnerMecanumDrive;


/**
 * To understand how Road Runner works and setting it up: https://learnroadrunner.com/
 */


@TeleOp

public class RoadRunnerTeleOp extends LinearOpMode {
    private RoadRunnerMecanumDrive drive;

    public void runOpMode() {
        drive = new RoadRunnerMecanumDrive(hardwareMap);
//
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // Set your initial pose to x: 10, y: 10, facing 90 degrees
//        // change initial pose x and y depending on starting position
//        drive.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));
//
//        telemetry.addData(">", "Press Play to start op mode");
//        telemetry.update();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && !isStopRequested()) {
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x,
//                            -gamepad1.right_stick_x
//                    )
//            );
//
//            // Make sure to call drive.update() on *every* loop
//            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
//            drive.update();
//
//            // Retrieve your pose
            Pose2d myPose = drive.getPoseEstimate();
//
//            telemetry.addData("x", myPose.getX());
//            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());
//            telemetry.update();
//
//            // Insert whatever teleop code you're using
////            this.tankanumDrive(gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_x);
//        }
    }

//    public void tankanumDrive(double rightPwr, double leftPwr, double lateralPwr) {
//        double leftFrontPower = Range.clip(leftPwr - lateralPwr, -1.0, 1.0);
//        double leftBackPower = Range.clip(leftPwr + lateralPwr, -1.0, 1.0);
//        double rightFrontPower = Range.clip(rightPwr - lateralPwr, -1.0, 1.0);
//        double rightBackPower = Range.clip(rightPwr + lateralPwr, -1.0, 1.0);
//
//        drive.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
//    }
}