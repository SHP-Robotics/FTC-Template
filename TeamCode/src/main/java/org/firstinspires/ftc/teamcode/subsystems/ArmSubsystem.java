package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;

public class ArmSubsystem extends Subsystem {
    public final SHPMotor arm;
    public final SHPMotor elevator;

    public enum State {
        TOP,
        MIDDLE,
        BOTTOM,
        DISABLE
    }

    private State state;

    public ArmSubsystem(HardwareMap hardwareMap) {
        arm = new SHPMotor(hardwareMap, "arm");
        arm.enablePositionPID(0.002);
//        arm.setStaticVoltage(0.05);
        arm.setPositionErrorTolerance(50);
        elevator = new SHPMotor(hardwareMap, "elevator");
        elevator.enablePositionPID(0.005);
        elevator.setPositionErrorTolerance(50);

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
        return arm.atPositionSetpoint() && elevator.atPositionSetpoint();
    }

    public boolean atBottom() {
        return this.state == State.BOTTOM;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("arm enc: ", arm.getPosition());
        telemetry.addData("elevator enc: ", elevator.getPosition());
        telemetry.addData("arm at setpoint: ", atSetpoint() ? "true" : "false");

        if (this.state != State.DISABLE && arm.isMotorDisabled())  arm.setMotorEnable();
        switch (state) {
            case TOP:
                arm.setPosition(600);
                elevator.setPosition(2500);
                telemetry.addData("state: ", "TOP");
                break;
            case MIDDLE:
                arm.setPosition(600);
                elevator.setPosition(1000);
                telemetry.addData("state: ", "MIDDLE");
                break;
            case BOTTOM:
                arm.setPosition(10);
                elevator.setPosition(10);
                telemetry.addData("state: ", "BOTTOM");
                break;
            case DISABLE:
                arm.setMotorDisable();
                break;
        }
    }
}
