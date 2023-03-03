package org.firstinspires.ftc.teamcode.teleops;

import static java.lang.Math.PI;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.controllers.PIDController;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp
public class EncoderTest extends OpMode {
    Encoder parallelEncoder, perpendicularEncoder;
    DriveSubsystem drive;

    PIDController pid;
    double ppr = 8192;
    double radius = .93; //inches
    //double parallel_multiplier =

    @Override
    public void init() {
        configure();
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));
        drive = new DriveSubsystem(hardwareMap);
        drive.enablePositionPID();
        pid = new PIDController(.00005);

        /*
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(gamepad1.left_stick_y*0.5, gamepad1.left_stick_x*0.5, gamepad1.right_stick_x*0.5)
                )
        );

         */




    }

    @Override
    public void start() {
        drive.setInitialPositions();
    }

    @Override
    public void loop() {
        try {
            CommandScheduler.getInstance().run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.setPosition(2000);

//        if (Math.abs(ticks_to_in(parallelEncoder.getCurrentPosition())) < 10)
//            drive.mecanum(profile(ticks_to_in(parallelEncoder.getCurrentPosition()), 10), 0, 0);
//        else
//            drive.mecanum(0, 0, 0);





        telemetry.addData("parallel - ticks: ", parallelEncoder.getCurrentPosition());
        telemetry.addData("perpendicular - ticks: ", perpendicularEncoder.getCurrentPosition());
        telemetry.addData("parallel - rotations: ", parallelEncoder.getCurrentPosition()/ppr);
        telemetry.addData("perpendicular - rotations: ", perpendicularEncoder.getCurrentPosition()/ppr);
        telemetry.addData("parallel - inches: ", parallelEncoder.getCurrentPosition()/ppr * 2 * PI * radius);
        telemetry.addData("perpendicular - inches: ", perpendicularEncoder.getCurrentPosition()/ppr * 2 * PI * radius);

    }

    public void stop() {
        CommandScheduler.resetInstance();
    }

    public void configure() {
        // Starts universal clock - DO NOT DELETE!
        Clock.start();
        // Assigns telemetry object for Subsystem.periodic - DO NOT DELETE!
        CommandScheduler.getInstance().setTelemetry(telemetry);
        // Turn on bulk reads to help optimize loop times
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public double ticks_to_in(int ticks) {
        return ticks/ppr * 2 * PI * radius;
    }

    public double in_to_ticks(int inches) {
        return inches / (1/ppr * 2 * PI * radius);
    }

//    public double profile(double current, double setpoint)
}