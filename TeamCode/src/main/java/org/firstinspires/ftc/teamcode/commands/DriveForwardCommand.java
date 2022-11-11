package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoopSubsystem;

public class DriveForwardCommand extends Command {
    private final DriveSubsystem drive;
    private final double ticks;

    public DriveForwardCommand(DriveSubsystem drive, double ticks) {
        // Pass through any subsystems that are uninterruptible
        super(drive);

        this.drive = drive;
        this.ticks = ticks;
    }

    @Override
    public void init() {
//        drive.enablePositionPID();
    }

    @Override
    public void execute() {
//        drive.driveTo(true, ticks);
    }

//    @Override
//    public boolean isFinished() {
//        return drive.atPositionSetpoint();
//    }
}
