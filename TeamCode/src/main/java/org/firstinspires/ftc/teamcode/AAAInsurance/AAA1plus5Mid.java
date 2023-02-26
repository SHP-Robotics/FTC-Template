//package org.firstinspires.ftc.teamcode.AAAInsurance;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.BaseRobot;
//import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
//import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
//import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
//import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
//import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
//import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
//
//@TeleOp
//public class AAA1plus5Mid extends BaseRobot {
//    //DriveSubsystem drive;
//    private double debounce;
//    private int desiredPosition;
//    private double maxSpeed;
//    private ArmSubsystem.State topState;
//    @Override
//    public void init() {
//        super.init();
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        // Default command runs when no other commands are scheduled for the subsystem
//        drive.setDefaultCommand(
//                new RunCommand(
//                        () ->
//                                drive.automecanum(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x)
//                )
//        );
//
//        arm.resetEncoder();
//        drive.parallelEncoder.resetEncoder();
//        drive.perpendicularEncoder.resetEncoder();
//
//        telemetry.addData("slide ticks: ", arm.slide.getPosition(MotorUnit.TICKS));
//        ArmSubsystem.State topState = ArmSubsystem.State.TOP;
//
//        new RunCommand(( () -> {arm.setState(ArmSubsystem.State.BOTTOM);}));
//
//    }
//    @Override
//    public void start() {
//        super.start();
//
//        debounce = Clock.now();
//        arm.override = false;
//        maxSpeed = 0.75; //TODO: SPEED HERE
//        topState = ArmSubsystem.State.TOP;
//
//        CommandScheduler myCommand = CommandScheduler.getInstance();
//        myCommand.scheduleCommand(
//                new RunCommand(() -> {
//                    claw.setState(ClawSubsystem.State.CLOSED);
//                })
//                        .then(new WaitCommand(2))
//                        .then(new RunCommand(() -> {
//                            arm.setState(ArmSubsystem.State.CARRYING);
//                        }))
//
//                        .then(new RunCommand(()->{
//                            arm.setState(ArmSubsystem.State.MIDDLE);
//                        }))
//                        .then(new EncoderStraightDriveCommand(drive,0.4,38))
//                        .then(new EncoderStrafeDriveCommand(drive,-0.2,  4, false))
//                        .then(new RunCommand(()->{
//                            claw.setState(ClawSubsystem.State.OPEN);
//                        }))
//                        .then(new EncoderStrafeDriveCommand(drive,0.2,  4, false))
//                        .then(new RunCommand(()->{
//                            arm.setState(ArmSubsystem.State.BOTTOM);
//                        }))
//                        .then (new WaitCommand(0.5))
//                        .then(new EncoderStraightDriveCommand(drive, 0.2, 13))
//                        .then(new WaitCommand(1))
//                        .then(new EncoderTurnDriveCommand(drive, 0.2,179))
//                        .then(new RunCommand(()->{
//                            arm.setState(ArmSubsystem.State.STACKED_CONES);
//                        }))
//                        .then (new WaitCommand(0.5))
//                        .then(new EncoderStrafeDriveCommand(drive,0.3,  19, false))
//                        .then(new WaitCommand(1.5))
//                        .then(new RunCommand(()->{
//                            claw.setState(ClawSubsystem.State.CLOSED);
//                        }))
//                        .then(new WaitCommand(1))
//                        .then(new RunCommand(()->{
//                            arm.setState(ArmSubsystem.State.MIDDLE);
//                        }))
//                        .then(new WaitCommand(1))
//                        .then(new EncoderStrafeDriveCommand(drive,-0.4,  31.5, false))
//                        .then(new WaitCommand(1))
//                        .then(new EncoderTurnDriveCommand(drive, 0.2,-90))
//                        .then(new WaitCommand(1.5))
//                        .then(new EncoderStrafeDriveCommand(drive,-0.5,  3, true))
//                        .then(new RunCommand(()->{
//                            claw.setState(ClawSubsystem.State.OPEN);
//                        }))
//
//
//
//
//        );
//
//
//    }
//
//    @Override
//    public void loop() {
//        super.loop();
//        telemetry.addData("Y Ticks", drive.parallelEncoder.getCurrentPosition());
//        telemetry.addData("X Ticks", drive.perpendicularEncoder.getCurrentPosition());
//    }
//}
