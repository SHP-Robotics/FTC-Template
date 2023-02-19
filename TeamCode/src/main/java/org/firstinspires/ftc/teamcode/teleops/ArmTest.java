package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.TestSlideBaseRobot;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@Config
@TeleOp
public class ArmTest extends TestSlideBaseRobot {
    private double debounce;
    private int desiredPosition;
    private double maxSpeed;
    private ArmSubsystem.State topState;
    @Override
    public void init() {
        super.init();

        // Default command runs when no other commands are scheduled for the subsystem


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm.resetEncoder();
        telemetry.addData("slide ticks: ", arm.slide.getPosition(MotorUnit.TICKS));
        ArmSubsystem.State topState = ArmSubsystem.State.TOP;

        new RunCommand(( () -> {arm.setState(ArmSubsystem.State.BOTTOM);}));

    }

    @Override
    public void start() {
        super.start();
        debounce = Clock.now();
        arm.override = false;
        maxSpeed = 0.45;
        topState = ArmSubsystem.State.TOP;


        // Add anything that needs to be run a single time when the OpMode starts
    }


    @Override
    public void loop() {

        // Allows CommandScheduler.run() to be called - DO NOT DELETE!
        super.loop();
        //drive.setDriveBias(arm.getDriveBias(), maxSpeed);
        telemetry.addData("max speed: ", maxSpeed);
        telemetry.addData("SLIDE ENCODER: ", arm.slide.getPosition(MotorUnit.TICKS));
        telemetry.addData("POWER: ", arm.slide.getPower());
        telemetry.addData("DIFFERENCE From TOP: ", Constants.Arm.K_SLIDE_TOP - arm.slide.getPosition(MotorUnit.TICKS));
        telemetry.addData("DIFFERENCE From BOTTOM: ", arm.slide.getPosition(MotorUnit.TICKS) -Constants.Arm.K_SLIDE_BOTTOM);

        new Trigger(gamepad1.dpad_left, new RunCommand(( () -> {arm.override = false;}))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.SHORT))
                .then(new RunCommand(( () -> {topState = ArmSubsystem.State.SHORT;}))));

        new Trigger(gamepad1.dpad_right, new RunCommand(( () -> {arm.override = false;}))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.MIDDLE))
                .then(new RunCommand(( () -> {topState = ArmSubsystem.State.MIDDLE;}))));

        new Trigger(gamepad1.dpad_up, new RunCommand(( () -> {arm.override = false;}))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.TOP))
                .then(new RunCommand(( () -> {topState = ArmSubsystem.State.TOP;}))));

        new Trigger(gamepad1.dpad_down, new RunCommand(( () -> {arm.override = false;}))
                .then(new MoveArmCommand(arm, MoveArmCommand.Direction.BOTTOM))
                .then(new RunCommand(( () -> {topState = ArmSubsystem.State.BOTTOM;}))));

        new Trigger(gamepad1.right_bumper, new RunCommand(( () -> {
            desiredPosition = (int)(arm.slide.getPosition(MotorUnit.TICKS)) + 500;
            arm.setManualPos(desiredPosition);
        })));
        new Trigger(gamepad1.left_bumper, new RunCommand(( () -> {
            desiredPosition = (int)(arm.slide.getPosition(MotorUnit.TICKS)) - 500;
            arm.setManualPos(desiredPosition);
        })));

    }

}
