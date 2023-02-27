package org.firstinspires.ftc.teamcode.AAAInsurance;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class EncoderStraightDriveCommand extends Command {
    private final DriveSubsystem drive;
    private double startTime;
    private double xPos;
    private double yPos;
    double leftY; double leftX; double rightX; double time;

    double targetX, targetY;
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in

    public EncoderStraightDriveCommand(DriveSubsystem drive, String direction, double distance) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(drive);
        this.drive = drive;
        if(direction.equals("forward"))
            this.leftY = -0.3;
        else
            this.leftY = 0.3;
        this.leftX = 0;
        this.rightX = 0;
        this.xPos = 0;
        this.yPos = inchesToEncoderTicks(distance);;
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
        if (drive.parallelEncoder.getCurrentPosition()<0.2*Math.abs(yPos))
            drive.automecanum(leftY, leftX, rightX);
        else if (drive.parallelEncoder.getCurrentPosition()<0.8*Math.abs(yPos))
            drive.automecanum(1.5*leftY, leftX, rightX);
        else
            drive.automecanum(leftY, leftX, rightX);

       // drive.automecanum(leftY, leftX, rightX);

    }

    double difference = 0;
    double percentage = 0;
    //@Override
    public void executeAttempt() {
        difference = Math.abs(yPos - drive.parallelEncoder.getCurrentPosition());
        if (difference<0.25*Math.abs(yPos)) {
            drive.automecanum(0.2, 0, 0);
        }
        else if (difference<0.5*Math.abs(yPos)) {
            percentage = (difference/(Math.abs(yPos)/2));
            percentage = Range.clip(percentage, 0.2, 0.8);
            drive.automecanum(percentage, 0, 0);
        }
        else if (difference<0.75*Math.abs(yPos)) {

            percentage = (Math.abs(difference-Math.abs(yPos)))/(Math.abs(yPos)/2);
            percentage = Range.clip(percentage, 0.2, 0.8);
            drive.automecanum(percentage, 0,  0);
        }
        else
            drive.automecanum(0.2, 0, 0);

        // drive.automecanum(leftY, leftX, rightX);

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
        return drive.parallelEncoder.getCurrentPosition()>Math.abs(yPos);

    }
}
