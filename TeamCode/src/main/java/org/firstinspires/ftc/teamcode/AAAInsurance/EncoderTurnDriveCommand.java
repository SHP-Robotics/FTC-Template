package org.firstinspires.ftc.teamcode.AAAInsurance;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxModule;

public class EncoderTurnDriveCommand extends Command {
    private final DriveSubsystem drive;
    private double startTime;
    private double degrees;
    double leftY; double leftX; double rightX; double time;

    double targetX, targetY;
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in

    public EncoderTurnDriveCommand(DriveSubsystem drive, double power, double degrees) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(drive);
        this.drive = drive;
        this.leftY = 0;
        this.leftX = 0;
        this.rightX = power;
        this.degrees = degrees;

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
    @Override
    public void execute() {
        if (-drive.imu.getAutoYaw()<0.2*degrees)
            drive.automecanum(0, 0, rightX);
        else if (-drive.imu.getAutoYaw()<0.8*degrees)
            drive.automecanum(0, 0, 2*rightX);
        else
            drive.automecanum(0, 0, rightX);


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
        return Math.toDegrees(-drive.imu.getAutoYaw())>degrees;
    }
}
