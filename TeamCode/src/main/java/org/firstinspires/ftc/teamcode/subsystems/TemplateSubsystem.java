package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class TemplateSubsystem extends Subsystem {
    // Declare devices here
    // e.g. private final SHPMotor motor;

    public enum State {
        DISABLED,
        // Add states here
    }

    private State state;

    public TemplateSubsystem(HardwareMap hardwareMap) {
        // Initialize devices here
        // e.g. motor = new SHPMotor(hardwareMap, "motor");

        state = State.DISABLED;
    }

    // Add control methods here
    // e.g. public void setPower(double power) { motor.setPower(power); }

    public String getState() {
        return state.toString();
    }

    public void setState(State state) {
        this.state = state;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        // Add any logging here
        // e.g. telemetry.addData("Motor Encoder: ", motor.getCurrentPosition());

        switch (state) {
            case DISABLED:
                // Handle disabled state
                // e.g. motor.setPower(0.0);
                break;
        }
    }
}
