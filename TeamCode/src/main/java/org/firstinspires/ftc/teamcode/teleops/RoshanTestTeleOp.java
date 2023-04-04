package org.firstinspires.ftc.teamcode.teleops;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;

import java.util.Base64;

public class RoshanTestTeleOp extends BaseRobot {

    @Override
    public void init() {
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
                )
        );
    }

    @Override
    public void start(){
        super.start();
    }

    @Override
    public void loop(){
        super.loop();


    }
}
