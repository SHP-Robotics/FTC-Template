package org.firstinspires.ftc.teamcode.AAAInsurance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@TeleOp
public class AAATestingAuto extends BaseRobot {
    //DriveSubsystem drive;
    private double debounce;
    private int desiredPosition;
    private double maxSpeed;
    private ArmSubsystem.State topState;
    @Override
    public void init() {
        super.init();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Default command runs when no other commands are scheduled for the subsystem
        drive.setDefaultCommand(
                new RunCommand(
                        () ->
                                drive.automecanum(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x)
                )
        );

        arm.resetEncoder();
        drive.parallelEncoder.resetEncoder();
        drive.perpendicularEncoder.resetEncoder();

        telemetry.addData("slide ticks: ", arm.slide.getPosition(MotorUnit.TICKS));
        ArmSubsystem.State topState = ArmSubsystem.State.TOP;

        new RunCommand(( () -> {arm.setState(ArmSubsystem.State.BOTTOM);}));

    }
    @Override
    public void start() {
        super.start();

        debounce = Clock.now();
        arm.override = false;
        maxSpeed = 0.75; //TODO: SPEED HERE
        topState = ArmSubsystem.State.TOP;

        CommandScheduler myCommand = CommandScheduler.getInstance();
        myCommand.scheduleCommand(
                new RunCommand(() -> {
                    claw.setState(ClawSubsystem.State.CLOSED);
                })
                        .then(new WaitCommand(2))
                        .then(new RunCommand(() -> {
                            arm.setState(ArmSubsystem.State.CARRYING);
                        }))
                        .then(new EncoderStrafeDriveCommand(drive,"left", 20, false))
                        .then(new EncoderStrafeDriveCommand(drive,"right", 20, false))

        );


    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("Y Ticks", drive.parallelEncoder.getCurrentPosition());
        telemetry.addData("X Ticks", drive.perpendicularEncoder.getCurrentPosition());
    }
}
