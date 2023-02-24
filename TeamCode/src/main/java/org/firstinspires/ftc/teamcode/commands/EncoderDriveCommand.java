package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxModule;

public class EncoderDriveCommand extends Command {
    private final DriveSubsystem drive;
    private double startTime;
    private double xPos;
    private double yPos;
    double leftY; double leftX; double rightX; double time;

    double targetX, targetY;
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in

    public EncoderDriveCommand(DriveSubsystem drive, double leftY, double leftX, double rightX, double xPosInches, double yPosInches) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(drive);
        this.drive = drive;
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;
        this.xPos = inchesToEncoderTicks(xPosInches);
        this.yPos = inchesToEncoderTicks(yPosInches);;
        //System.out.println(xPos);

    }


    // Called once when the command is initially schedule

    public void init() {

    }
    public static double encoderTicksToInches(double ticks) {

        return WHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static double inchesToEncoderTicks(double inches) {
        double c = (((WHEEL_RADIUS*2)*Math.PI));
        double ticksPerInch = TICKS_PER_REV/c;
        return ticksPerInch * inches;
        //return inches/(WHEEL_RADIUS*2*TICKS_PER_REV*Math.PI);
    }
    public double getxPos() {
        return xPos;
    }
    public double getyPos() {
        return yPos;
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
        return (drive.perpendicularEncoder.getCurrentPosition()>xPos && drive.parallelEncoder.getCurrentPosition()>yPos);
    }
}
