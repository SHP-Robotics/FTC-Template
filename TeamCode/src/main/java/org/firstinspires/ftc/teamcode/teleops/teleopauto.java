package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.EncoderDriveCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FindAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.InteractWithConeCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(preselectTeleOp = "TestTeleOp")
public class teleopauto extends BaseRobot {
    DriveSubsystem drive;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();
        CommandScheduler myCommand = CommandScheduler.getInstance();
        myCommand.scheduleCommand(
                new RunCommand(() -> {
                    claw.setState(ClawSubsystem.State.CLOSED);
                })
                        .then(new RunCommand(() -> {
                            arm.setState(ArmSubsystem.State.CARRYING);
                        }))
                        .then(new EncoderDriveCommand(drive,0.4, 0, 0, 0, 20))
        );


    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("Y Ticks", drive.parallelEncoder.getCurrentPosition());
        telemetry.addData("X Ticks", drive.perpendicularEncoder.getCurrentPosition());
    }
}