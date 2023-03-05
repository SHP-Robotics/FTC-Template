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
public class AAATestingAuto extends BaseRobot {
    //DriveSubsystem drive;
    private double debounce;
    private int desiredPosition;
    private double strafeTime = 0.2;
    private double parkTime = 1;
    private double parkSpeed = 0.4;
    private String parkDirection = "backward";
    private double maxSpeed;
    private int tagID;
    DriveCommand driveCommand;
    private ArmSubsystem.State topState;

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

        new RunCommand((() -> {
            arm.setState(ArmSubsystem.State.BOTTOM);
        }));

        CommandScheduler myInitCommand = CommandScheduler.getInstance();
        myInitCommand.scheduleCommand(
                new FindAprilTagCommand(vision)

        );



    }

    @Override
    public void init_loop() {
        super.init_loop();
        if (vision.getTags().size()>0)
            telemetry.addData("TagID", vision.getTags().get(0).id);
    }

    @Override
    public void start() {
        super.start();
        if (vision.getTags().size()>0) {
            tagID = vision.getTags().get(0).id;
            if (tagID == 12) {
                driveCommand = new DriveCommand(drive, 0, 0.3, 0.0, parkTime, false);
            } else if (tagID==8) {
                driveCommand = new DriveCommand(drive, 0, 0, 0.0, parkTime, false);
            } else if (tagID==7) {
                driveCommand = new DriveCommand(drive, 0, -0.3, 0.0, parkTime, false);
            }

        }
        else {
            driveCommand = new DriveCommand(drive, 0.3, 0, 0.0, parkTime, false);
        }



        CommandScheduler.getInstance().scheduleCommand(driveCommand.then(new DriveCommand(drive,0,0,0,10, true)));


    }

    @Override
    public void loop() {
        super.loop();

        for (AprilTagDetection tag : vision.getTags()) {
            telemetry.addData("Tag ID: ", vision.getTags().get(0).id);
        }
        telemetry.addData("IMU", drive.imu.getIntegratedHeading());
        telemetry.addData("Y Ticks", drive.parallelEncoder.getCurrentPosition());
        telemetry.addData("X Ticks", drive.perpendicularEncoder.getCurrentPosition());
    }
}
