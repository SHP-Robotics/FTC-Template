package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;

public class ScoopSubsystem extends Subsystem {
    public final Servo scoop;

    public enum State {
        TOP,
        MIDDLE,
        BOTTOM
    }

    private State state;

    public ScoopSubsystem(HardwareMap hardwareMap) {
        scoop = hardwareMap.get(Servo.class, "scoop");

        this.state = State.BOTTOM;
    }

    public void setState(State state) {
        this.state = state;
    }

//    public void nextState() {
//        if (this.state == State.MIDDLE) this.state = State.TOP;
//        else if (this.state == State.BOTTOM) this.state = State.MIDDLE;
//    }
//
//    public void previousState() {
//        if (this.state == State.TOP) this.state = State.MIDDLE;
//        else if (this.state == State.MIDDLE) this.state = State.BOTTOM;
//    }

    @Override
    public void periodic(Telemetry telemetry) {
        switch (state) {
            case TOP:
                scoop.setPosition(Constants.kScoopTop);
                break;
            case MIDDLE:
                scoop.setPosition(Constants.kScoopMiddle);
                break;
            case BOTTOM:
                scoop.setPosition(Constants.kScoopBottom);
                break;
        }
    }
}
