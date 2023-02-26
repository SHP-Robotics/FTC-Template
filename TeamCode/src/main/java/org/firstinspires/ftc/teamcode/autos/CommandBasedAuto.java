package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.AAAInsurance.EncoderStraightDriveCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@Autonomous(preselectTeleOp = "TestTeleOp")
public class CommandBasedAuto extends BaseRobot {
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
                        .then (new WaitCommand(2))
                        .then(new RunCommand(() -> {
                            arm.setState(ArmSubsystem.State.CARRYING);
                        }))
                        //.then(new EncoderDriveCommand(drive,0, 0.3, 0, 40, 0))
                        .then(new RunCommand(() -> {
                            arm.setState(ArmSubsystem.State.TOP);
                        }))
                        .then(new EncoderStraightDriveCommand(drive, 60))
        );


    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("Y Ticks", drive.parallelEncoder.getCurrentPosition());
        telemetry.addData("X Ticks", drive.perpendicularEncoder.getCurrentPosition());
    }
}
