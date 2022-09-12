package org.firstinspires.ftc.teamcode.commands;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoopSubsystem;

public class DumpCargo extends Command {
    private final ScoopSubsystem scoop;
    private double startTime;

    public DumpCargo(ScoopSubsystem scoop) {
        // Pass through any subsystems that are uninterruptible
        super(scoop);

        this.scoop = scoop;
    }

    @Override
    public void init() {
        startTime = Clock.seconds();
        scoop.setState(ScoopSubsystem.State.TOP);
    }

    @Override
    public void execute() {
        if (Clock.hasElapsed(startTime, 1)) scoop.setState(ScoopSubsystem.State.BOTTOM);
    }

    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, 2);
    }

    @Override
    public void end() {
    }
}
