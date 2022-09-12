package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DumpCargo;
import org.firstinspires.ftc.teamcode.commands.MoveArm;
import org.firstinspires.ftc.teamcode.shplib.commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.subsystems.ScoopSubsystem;

@TeleOp
public class CommandBasedTeleOp extends BaseRobot {

    @Override
    public void init() {
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
                )
        );
//        scoop.setDefaultCommand(
//                new RunCommand(() -> {
//                    scoop.setState(ScoopSubsystem.State.BOTTOM);
//                })
//        );
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        // Allows CommandScheduler.run to be called - DO NOT DELETE!
        super.loop();

        new Trigger(gamepad1.dpad_up, new MoveArm(arm, MoveArm.Direction.UP));
        new Trigger(gamepad1.dpad_down, new MoveArm(arm, MoveArm.Direction.DOWN));
        new Trigger(gamepad1.a,
                new RunCommand((() -> {
                    scoop.setState(ScoopSubsystem.State.MIDDLE);
                }), scoop)
                        .then(new MoveArm(arm, MoveArm.Direction.TOP))
//                new ParallelCommandGroup(
//                        new RunCommand((() -> {
//                            scoop.setState(ScoopSubsystem.State.MIDDLE);
//                        }), scoop),
//                        new MoveArm(arm, MoveArm.Direction.TOP)
//                )
                        .then(new DumpCargo(scoop))
                        .then(new MoveArm(arm, MoveArm.Direction.MIDDLE))
                        .then(new MoveArm(arm, MoveArm.Direction.BOTTOM))
        );
        new Trigger(gamepad1.b && !arm.atBottom(), new DumpCargo(scoop));
        intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

//        new Trigger(gamepad1.a, new BackAndForth(arm, scoop));
//        new Trigger(gamepad1.b, new RunCommand(() -> {
//            scoop.setState(ScoopSubsystem.State.TOP);
//        }, scoop));


//        if (gamepad1.b && scoop.getPosition() != Constants.kScoopTop) scoop.setPosition(Constants.kScoopTop);
//        else if (scoop.getPosition() != Constants.kScoopBottom) scoop.setPosition(Constants.kScoopBottom);

//        if (gamepad1.dpad_up) arm.setState(ArmSubsystem.State.UP);
//        else if (gamepad1.dpad_down) arm.setState(ArmSubsystem.State.DOWN);
//        if (gamepad1.b) arm.setState(ArmSubsystem.State.DISABLE);

    }
}