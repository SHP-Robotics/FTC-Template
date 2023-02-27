package org.firstinspires.ftc.teamcode.AAAInsurance;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class EncoderStrafeDriveCommand extends Command {
    private final DriveSubsystem drive;
    private double startTime;
    private double xPos;
    private double yPos;
    double leftY; double leftX; double rightX; double time;
    private boolean robot;

    double targetX, targetY;
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in

    public EncoderStrafeDriveCommand(DriveSubsystem drive, String direction, double distance, boolean robot) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(drive);
        this.drive = drive;
        this.leftY = 0;
        if(direction.equals("left"))
            this.leftX = -0.3;
        else
            this.leftX = 0.3;
        this.rightX = 0;
        this.xPos = inchesToEncoderTicks(distance);
        this.yPos = 0;
        this.robot = robot;
        //System.out.println(xPos);

    }


    // Called once when the command is initially schedule

    public void init() {
        drive.parallelEncoder.resetEncoder();
        drive.perpendicularEncoder.resetEncoder();
    }
    public static double encoderTicksToInches(double ticks) {

        return WHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static double inchesToEncoderTicks(double inches) {
        double c = (((WHEEL_RADIUS*2)*Math.PI));
        double ticksPerInch = TICKS_PER_REV/c;
        return ticksPerInch * inches;
    }


    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        if (robot) {
            //drive.normalmecanum(0, leftX, 0);
            if (drive.perpendicularEncoder.getCurrentPosition() < 0.2 * Math.abs(xPos))
                drive.normalmecanum(leftY, leftX, 0);
            else if (drive.perpendicularEncoder.getCurrentPosition() < 0.8 * Math.abs(xPos))
                drive.normalmecanum(leftY, leftX*2, 0);
            else
                drive.normalmecanum(leftY, leftX,0 );
        }
        else {
//            drive.automecanum(0, leftX, 0);
            if (drive.perpendicularEncoder.getCurrentPosition() < 0.2 * Math.abs(xPos))
                drive.automecanum(0, leftX, 0);
            else if (drive.perpendicularEncoder.getCurrentPosition() < 0.8 * Math.abs(xPos))
                drive.automecanum(0, leftX*2, 0);
            else
                drive.automecanum(0, leftX,0 );
        }

    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        drive.parallelEncoder.resetEncoder();
        drive.perpendicularEncoder.resetEncoder();
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Math.abs(drive.perpendicularEncoder.getCurrentPosition())>Math.abs(xPos);
        //(Math.abs(Math.abs(drive.perpendicularEncoder.getCurrentPosition())-Math.abs(xPos))<500);
            //&& (Math.abs(Math.abs(drive.parallelEncoder.getCurrentPosition())-Math.abs(yPos))<500);
    }
}
