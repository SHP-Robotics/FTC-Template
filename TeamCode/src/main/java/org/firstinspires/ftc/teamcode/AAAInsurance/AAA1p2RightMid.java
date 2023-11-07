package org.firstinspires.ftc.teamcode.AAAInsurance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FindAprilTagCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous
public class AAA1p2RightMid extends BaseRobot {
    //DriveSubsystem drive;
    private double debounce;
    private int desiredPosition;
    private double maxSpeed;
    private ArmSubsystem.State topState;

    DriveCommand driveCommand;
    private double strafeTime = 0.25;
    private int tagID;
    @Override
    public void init() {
        super.init();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Default command runs when no other commands are scheduled for the subsystem


        arm.resetEncoder();
        drive.parallelEncoder.resetEncoder();
        drive.perpendicularEncoder.resetEncoder();

        telemetry.addData("slide ticks: ", arm.slide.getPosition(MotorUnit.TICKS));
        ArmSubsystem.State topState = ArmSubsystem.State.TOP;

        new RunCommand(( () -> {arm.setState(ArmSubsystem.State.BOTTOM);}));

        CommandScheduler myInitCommand = CommandScheduler.getInstance();
        myInitCommand.scheduleCommand(
                new RunCommand(() -> {
                    claw.setState(ClawSubsystem.State.CLOSED);
                })
                        .then(new FindAprilTagCommand(vision))
        );

    }
    @Override
    public void start() {
        super.start();
        if (vision.getTags().size()>0) {
            tagID = vision.getTags().get(0).id;
            if (tagID == 12) {
                driveCommand = new DriveCommand(drive, 0.5, 0, 0.0, 1.2, false);
            } else if (tagID==8) {
                driveCommand = new DriveCommand(drive, 0.3, 0, 0.0, 0.6, false);
            } else if (tagID==7) {
                driveCommand = new DriveCommand(drive, -0.5, 0, 0.0, 0.6, false);
            }

        }
        else {
            driveCommand = new DriveCommand(drive, -0.3, 0, 0, 0.5, false);
        }
        debounce = Clock.now();
        arm.override = false;
        maxSpeed = 0.75; //TODO: SPEED HERE
        topState = ArmSubsystem.State.TOP;
        //TODO: 2.5 inches away for set up
        CommandScheduler myCommand = CommandScheduler.getInstance();
        myCommand.scheduleCommand(
                new RunCommand(() -> {
                    claw.setState(ClawSubsystem.State.CLOSED);
                })
                        .then(new WaitCommand(0.25))
                        .then(new RunCommand(() -> {
                            arm.setState(ArmSubsystem.State.CARRYING);
                        }))

                        .then(new RunCommand(()->{
                            arm.setState(ArmSubsystem.State.MIDDLE);
                        }))
                        //DROP FIRST CONE
                        .then(new EncoderStraightDriveCommand(drive,"forward",38.5))
                        .then(new EncoderStrafeDriveCommand(drive,"left",  .75, false))
                        .then (new WaitCommand(0.25))
                        .then(new RunCommand(()->{claw.setState(ClawSubsystem.State.OPEN);}))
                        .then (new WaitCommand(0.5))
                        .then(new EncoderStrafeDriveCommand(drive,"right",  .75, false))
                        .then (new WaitCommand(0.25))
                        .then(new RunCommand(()->{arm.setState(ArmSubsystem.State.BOTTOM);}))
                        // FORWARD AND TURN TOWARD STACKED CONE 1
                        .then (new WaitCommand(0.5))
                        .then(new EncoderStraightDriveCommand(drive, "forward", 14))
                        .then(new WaitCommand(0.5))
                        .then(new EncoderStraightDriveCommand(drive, "backward", 1.5))
                        .then(new WaitCommand(0.5))
                        .then(new EncoderTurnDriveCommand(drive, "cw",180))
                        .then(new RunCommand(()->{
                            arm.setState(ArmSubsystem.State.STACKED_CONES);
                        }))
                        //GO TO PICK UP STACKED CONES
                        .then (new WaitCommand(0.5))
                        .then(new EncoderStrafeDriveCommand(drive,"right",  21, false))
                        .then(new WaitCommand(1))
                        .then(new RunCommand(()->{
                            claw.setState(ClawSubsystem.State.CLOSED);
                        }))
                        .then(new WaitCommand(0.5))
                        .then(new RunCommand(()->{
                            arm.setState(ArmSubsystem.State.MIDDLE);
                        }))
                        .then(new WaitCommand(0.5))
                        //STRAFE BACK TOWARDS HIGH POLE TO DROP STACKED CONE 1

                        .then(new EncoderStrafeDriveCommand(drive,"left",  34.75, false))
                        .then(new WaitCommand(0.5))
                        .then(new EncoderTurnDriveCommand(drive, "cw",273))
                        .then(new WaitCommand(0.5))
                        //DRIVE AND DROP
                        .then (new DriveCommand(drive, -0.25, 0, 0.0, strafeTime, true))
                        .then(new RunCommand(()->{claw.setState(ClawSubsystem.State.OPEN);}))
                        .then(new DriveCommand(drive, 0.25, 0, 0.0, strafeTime, true))
                        .then(new RunCommand(()->{arm.setState(ArmSubsystem.State.STACKED_CONES);}))

                        //TURN BACK TOWARDS STACK FOR STACK CONE 2 and DRIVE RIGHT
                        .then(new EncoderTurnDriveCommand(drive, "ccw",180))
                        .then(new WaitCommand(0.5))
                        .then(new EncoderStrafeDriveCommand(drive,"right",  33, false))
                        .then(new WaitCommand(0.5))
                        .then(new RunCommand(()->{claw.setState(ClawSubsystem.State.CLOSED);}))
                        .then(new WaitCommand(0.25))
                        .then(new RunCommand(()->{arm.setState(ArmSubsystem.State.MIDDLE);}))
                        .then(new WaitCommand(0.5))

                        //DRIVE BACK TOWARDS MIDDLE POLE TO DROP STACKED CONE 2
                        .then(new EncoderStrafeDriveCommand(drive,"left",  33.5, false))
                        .then(new EncoderTurnDriveCommand(drive, "cw",270))
                        .then (new DriveCommand(drive, -0.25, 0, 0.0, strafeTime, true))

                        .then(new RunCommand(()->{claw.setState(ClawSubsystem.State.OPEN);}))
                        .then(new DriveCommand(drive, 0.25, 0, 0.0, strafeTime+0.2, true))
                        .then(new RunCommand(()->{arm.setState(ArmSubsystem.State.BOTTOM);}))

                        //PARK
                        .then(driveCommand)




        );



    }

    @Override
    public void loop() {
        super.loop();
        for (AprilTagDetection tag : vision.getTags()) {
            telemetry.addData("Tag ID: ", tag.id);
        }
        telemetry.addData("IMU", drive.imu.getIntegratedHeading());
        telemetry.addData("Y Ticks", drive.parallelEncoder.getCurrentPosition());
        telemetry.addData("X Ticks", drive.perpendicularEncoder.getCurrentPosition());
    }
}
