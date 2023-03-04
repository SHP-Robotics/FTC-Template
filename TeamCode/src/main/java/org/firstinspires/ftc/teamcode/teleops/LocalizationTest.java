//package org.firstinspires.ftc.teamcode.teleops;
//
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.roadrunner.drive.TwoWheelTrackingLocalizer;
//import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
//import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
//import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//
//@TeleOp
//public class LocalizationTest extends OpMode {
//    TwoWheelTrackingLocalizer localizer;
//    DriveSubsystem drive;
//    @Override
//    public void init() {
//        configure();
//        localizer = new TwoWheelTrackingLocalizer(hardwareMap);
//        drive = new DriveSubsystem(hardwareMap);
//        drive.setDefaultCommand(
//                new RunCommand(
//                        () -> drive.mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
//                )
//        );
//    }
//    @Override
//    public void start() {
//        CommandScheduler.getInstance().scheduleCommand(
//                new RunCommand(()->drive.setPosition(10))
//        );
//    }
//    @Override
//    public void loop() {
//        try {
//            CommandScheduler.getInstance().run();
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//
//        telemetry.addData("heading: ", localizer.getHeading());
//        telemetry.addData("x: ", localizer.getPoseEstimate().getX());
//        telemetry.addData("y: ", localizer.getPoseEstimate().getY());
//    }
//    @Override
//    public void stop() {
//        CommandScheduler.resetInstance();
//    }
//    public void configure() {
//        // Starts universal clock - DO NOT DELETE!
//        Clock.start();
//        // Assigns telemetry object for Subsystem.periodic - DO NOT DELETE!
//        CommandScheduler.getInstance().setTelemetry(telemetry);
//        // Turn on bulk reads to help optimize loop times
//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//    }
//}
