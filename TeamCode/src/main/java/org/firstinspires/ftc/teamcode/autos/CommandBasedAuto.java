package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.FindAprilTagCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;

@Autonomous
public class CommandBasedAuto extends BaseRobot {
    @Override
    public void start() {
//        CommandScheduler.getInstance().scheduleCommand(
//                new FindAprilTagCommand(vision).then(
//                        new RunCommand();
//                )
//        );
    }
}
