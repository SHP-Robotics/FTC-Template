package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class EncoderDriveCommand extends Command {
    private final DriveSubsystem drive;
    private double startTime;
    private double xPos;
    private double yPos;
    double leftY; double leftX; double rightX; double time;
    boolean robot = true;

    public EncoderDriveCommand(DriveSubsystem drive, double leftY, double leftX, double rightX, boolean robot, double xPos, double yPos) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(drive);
        this.drive = drive;
        this.leftY = -leftY;
        this.leftX = leftX;
        this.rightX = rightX;
        this.xPos = xPos;
        this.yPos = yPos;
        this.robot = robot;
    }


    // Called once when the command is initially schedule

    public void init() {
        
    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        drive.mecanum(leftY, leftX, rightX);

    }

    // Called once after isFinished() returns true
    @Override
    public void end() {

    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Math.abs(drive.perpendicularEncoder.getCurrentPosition()-xPos)<100 && Math.abs(drive.parallelEncoder.getCurrentPosition()-yPos)<100;
    }
}
