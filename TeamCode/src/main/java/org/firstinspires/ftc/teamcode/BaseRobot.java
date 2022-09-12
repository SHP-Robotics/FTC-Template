package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoopSubsystem;

/**
 * Template created by Ayaan Govil on 8/21/2021.
 *
 * FTC Java Documentation: http://ftctechnh.github.io/ftc_app/doc/javadoc/index.html
 */

// previous gradle: 4.0.1

public class BaseRobot extends OpMode {
    public DriveSubsystem drive;
    public ArmSubsystem arm;
    public ScoopSubsystem scoop;

    public SHPMotor intake;

    private final ElapsedTime timer;

    public BaseRobot() {
        timer = new ElapsedTime();
    }

    @Override
    public void init() {
        telemetry.clearAll();

        // Assigns telemetry object for Subsystem.periodic - DO NOT DELETE!
        CommandScheduler.getInstance().setTelemetry(telemetry);
        Clock.start();

        // Instantiate your subsystems and devices
        drive = new DriveSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap);
        scoop = new ScoopSubsystem(hardwareMap);

        intake = new SHPMotor(hardwareMap, "intake");
    }

    // this function runs when you hit the start button after the init button
    @Override
    public void start() {
//        CommandScheduler.getInstance().addCommand(new MoveArm(arm).then(new Command().with(new Command())));
    }

    // this function runs when you hit the stop button
    @Override
    public void stop() {
        CommandScheduler.resetInstance();
    }

    @Override
    public void loop() {
        telemetry.addData("Loop Time (ms): ", timer.milliseconds());
        timer.reset();

        // Handles all subsystem and command execution - DO NOT DELETE!
        try {
            CommandScheduler.getInstance().run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
