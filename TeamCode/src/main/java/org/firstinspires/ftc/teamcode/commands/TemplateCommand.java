package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.TemplateSubsystem;

public class TemplateCommand extends Command {
    private final TemplateSubsystem template;

    public TemplateCommand(TemplateSubsystem template) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(template);

        this.template = template;
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end() {

    }
}
