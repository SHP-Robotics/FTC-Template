package org.firstinspires.ftc.teamcode.commands;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoopSubsystem;

public class BackAndForth extends Command {
    private final ArmSubsystem arm;
    private final ScoopSubsystem scoop;
    private ElapsedTime timer;


    public BackAndForth(ArmSubsystem arm, ScoopSubsystem scoop) {
        // Pass through any subsystems that are uninterruptible
        super(arm, scoop);

        this.arm = arm;
        this.scoop = scoop;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
        arm.setState(ArmSubsystem.State.TOP);
    }

    @Override
    public void execute() {
        if (arm.atSetpoint() && timer.seconds() >= 1) scoop.setState(ScoopSubsystem.State.TOP);
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= 2;
    }

    @Override
    public void end() {
        scoop.setState(ScoopSubsystem.State.BOTTOM);
        arm.setState(ArmSubsystem.State.BOTTOM);
    }
}
