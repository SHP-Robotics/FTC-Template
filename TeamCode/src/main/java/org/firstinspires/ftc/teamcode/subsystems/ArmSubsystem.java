package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;

// Use this class as a reference for creating new subsystems

public class ArmSubsystem extends Subsystem {
    public final SHPMotor slide;
    public final SHPMotor actuator;

    public enum State {
        TOP,
        MIDDLE,
        BOTTOM,
    }

    private State state;

    public ArmSubsystem(HardwareMap hardwareMap) {
        slide = new SHPMotor(hardwareMap, Constants.Arm.kSlideName);
        slide.enablePositionPID(Constants.Arm.kSlideP);
        slide.setPositionErrorTolerance(Constants.Arm.kSlideTolerance);

        actuator = new SHPMotor(hardwareMap, Constants.Arm.kActuatorName);
        actuator.enablePositionPID(Constants.Arm.kActuatorP);
        actuator.setPositionErrorTolerance(Constants.Arm.kActuatorTolerance);

        this.state = State.BOTTOM;
    }

    public void setState(State state) {
        this.state = state;
    }

    public void nextState() {
        if (this.state == State.MIDDLE) this.state = State.TOP;
        else if (this.state == State.BOTTOM) this.state = State.MIDDLE;
    }

    public void previousState() {
        if (this.state == State.TOP) this.state = State.MIDDLE;
        else if (this.state == State.MIDDLE) this.state = State.BOTTOM;
    }

    public boolean atSetpoint() {
        return slide.atPositionSetpoint() && actuator.atPositionSetpoint();
    }

    public boolean atBottom() {
        return this.state == State.BOTTOM;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("slide enc: ", slide.getPosition());
        telemetry.addData("actuator enc: ", actuator.getPosition());
        telemetry.addData("arm at setpoint: ", atSetpoint() ? "true" : "false");

        switch (state) {
            case TOP:
                slide.setPosition(Constants.Arm.kSlideTop);
                actuator.setPosition(Constants.Arm.kActuatorTop);
                telemetry.addData("state: ", "TOP");
                break;
            case MIDDLE:
                slide.setPosition(Constants.Arm.kSlideMiddle);
                actuator.setPosition(Constants.Arm.kActuatorMiddle);
                telemetry.addData("state: ", "MIDDLE");
                break;
            case BOTTOM:
                slide.setPosition(Constants.Arm.kSlideBottom);
                actuator.setPosition(Constants.Arm.kActuatorBottom);
                telemetry.addData("state: ", "BOTTOM");
                break;
        }
    }
}
