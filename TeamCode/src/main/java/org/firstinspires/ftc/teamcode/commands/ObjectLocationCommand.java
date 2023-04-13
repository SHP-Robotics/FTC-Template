package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.TemplateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraOn;


public class ObjectLocationCommand extends Command {
    private final CameraOn cameraSub;
    private double startTime;

    public ObjectLocationCommand(CameraOn cameraSub) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(cameraSub);

        this.cameraSub = cameraSub;
    }

    // Called once when the command is initially schedule
    @Override
    public void init() {
        this.startTime = Clock.now();
        //just need to start to actually see stuff
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {return Clock.hasElapsed(startTime, 60);}

    // Called once after isFinished() returns true
    @Override
    public void end() {}
}

