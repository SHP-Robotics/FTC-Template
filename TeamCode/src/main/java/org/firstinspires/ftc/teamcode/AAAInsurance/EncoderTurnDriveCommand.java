package org.firstinspires.ftc.teamcode.AAAInsurance;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.Range;

public class EncoderTurnDriveCommand extends Command {
    private final DriveSubsystem drive;
    private double startTime;
    private double degrees;
    private double fullDegrees;
    public double initialHeading;
    double leftY; double leftX; double rightX;

    double targetX, targetY;
    String direction;
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in

    public EncoderTurnDriveCommand(DriveSubsystem drive, String direction, double degrees) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(drive);
        this.drive = drive;
        this.leftY = 0;
        this.leftX = 0;
        this.degrees = degrees;
        this.direction = direction;
        //TODO: switch cc/cw input to based on +/- of degrees where positive is clockwise

        if(direction.equals("cw")) {
            this.rightX = 0.5;
            fullDegrees = drive.imu.getIntegratedHeading() + degrees;
        }
        else {
            this.rightX = -0.5;
            fullDegrees = drive.imu.getIntegratedHeading() - degrees;
        }

        initialHeading = drive.imu.getIntegratedHeading();



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

    // Called repeatedly until isFinished() returns true
    //@Override
    public void executeAttempt() {
        if (drive.imu.getIntegratedHeading()<0.2*degrees)
            drive.automecanum(0, 0, rightX);
        else if (drive.imu.getIntegratedHeading()<0.4*degrees)
            drive.automecanum(0, 0, 1.5*rightX);
        else if (drive.imu.getIntegratedHeading()<0.6*degrees)
            drive.automecanum(0, 0, 2*rightX);
        else if (drive.imu.getIntegratedHeading()<0.8*degrees)
            drive.automecanum(0, 0, 1.5*rightX);
        else if (drive.imu.getIntegratedHeading()<0.9*degrees)
            drive.automecanum(0, 0, rightX);
        else
            drive.automecanum(0, 0, 0.75*rightX);


    }

    double percentage = 0;
    double difference = 0;

    //@Override
    public void execute(){
        if (direction.equals("cw")){
        difference = Math.abs(initialHeading-drive.imu.getIntegratedHeading());
        if (difference<Math.abs(0.25*degrees)) {
            drive.automecanum(0, 0, 0.2);
        }
        else if (difference<Math.abs(0.5*degrees)) {
            percentage = (difference/(degrees/2));
            percentage = Range.clip(percentage, 0.2, 0.8);
            drive.automecanum(0, 0, percentage);
        }
        else if (difference<Math.abs(0.75*degrees)) {

            percentage = (Math.abs(difference-degrees))/(degrees/2);
            percentage = Range.clip(percentage, 0.2, 0.8);
            drive.automecanum(0, 0,  percentage);
        }
        else
            drive.automecanum(0, 0, 0.2);
        }

        else {
            difference = Math.abs(initialHeading - drive.imu.getIntegratedHeading());
            if (difference < Math.abs(0.25 * degrees)) {
                drive.automecanum(0, 0, -0.3);
            } else if (difference < Math.abs(0.5 * degrees)) {
                percentage = -(difference / (degrees / 2));
                percentage = Range.clip(percentage, -0.2, -0.8);
                drive.automecanum(0, 0, percentage);
            } else if (difference < Math.abs(0.75 * degrees)) {

                percentage = -(Math.abs(difference - degrees)) / (degrees / 2);
                percentage = Range.clip(percentage, -0.2, -0.8);
                drive.automecanum(0, 0, percentage);
            } else
                drive.automecanum(0, 0, -0.3);
        }
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        drive.automecanum(0,0,0);
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    //TODO: IMU IS WEIRD VALUES ARHGILSHDG BIOSAalifuhdlafbohub
    @Override
    public boolean isFinished() {
        if(direction.equals("cw")) //>0 means turning CW
            return drive.imu.getIntegratedHeading()>fullDegrees*0.98;
        else //turning CCW
            return drive.imu.getIntegratedHeading()<fullDegrees*1.02;

    }
}
