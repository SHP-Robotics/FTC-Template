package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Arm.*;

import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RoshanArmSubsystem extends Subsystem {
    private final Servo claw, swingLeft, swingRight, counterSwingLeft, counterSwingRight;
    private final SHPMotor linearSlide;
    private double clawPosition;
    private int stackIndex = 4;
    public enum ArmState {
        TOP, BOTTOM
    }
    public enum SlideState{
        BOTTOM, HUB, MIDDLE, HIGH, STACK, CUSTOM
    }
    private ArmState armState;
    private SlideState slideState;

    public RoshanArmSubsystem(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, kClawName);
        swingLeft = hardwareMap.get(Servo.class, kSwingLeftName);
        swingRight = hardwareMap.get(Servo.class, kSwingRightName);
        counterSwingLeft = hardwareMap.get(Servo.class, kCounterSwingLeftName);
        counterSwingRight = hardwareMap.get(Servo.class, kCounterSwingRightName);

        linearSlide = new SHPMotor(hardwareMap, kLinearSlideName);
        linearSlide.reverseDirection();
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void setArmState(ArmState state) {
        this.armState = state;
    }

    public void setSlideState(SlideState state) {
        this.slideState = state;
    }

    // Claw Methods
    public void setClawPosition(double position) {
        claw.setPosition(position);
        clawPosition = position;
    }

    public boolean isClawOpen() {
        return clawPosition == kClawOpen;
    }

    public boolean isClawClosed() {
        return clawPosition == kClawClosed;
    }

    public void openClaw() {
        setClawPosition(kClawOpen);
    }

    public void closeClaw() {
        setClawPosition(kClawClosed);
    }

    // swing arm methods

    public void setSwingLeftPosition(double position) {
        swingLeft.setPosition(position);
    }

    public void setSwingRightPosition(double position) {
        swingRight.setPosition(position);
    }

    public void setCounterSwingLeftPosition(double position) {
        counterSwingLeft.setPosition(position);
    }

    public void setCounterSwingRightPosition(double position) {
        counterSwingRight.setPosition(position);
    }

    public boolean isSwingArmUp() {
        return swingLeft.getPosition() == kSwingLeftUp && swingRight.getPosition() == kSwingRightUp;
    }

    public boolean isSwingArmDown() {
        return swingLeft.getPosition() == kSwingLeftDown && swingRight.getPosition() == kSwingRightDown;
    }

    public void swingArmUp() {
        setSwingLeftPosition(kSwingLeftUp);
        setSwingRightPosition(kSwingRightUp);
        setCounterSwingLeftPosition(kCounterSwingLeftUp);
        setCounterSwingRightPosition(kCounterSwingRightUp);
        setArmState(ArmState.TOP);
    }

    public void swingArmDown() {
        setSwingLeftPosition(kSwingLeftDown);
        setSwingRightPosition(kSwingRightDown);
        setCounterSwingLeftPosition(kCounterSwingLeftDown);
        setCounterSwingRightPosition(kCounterSwingRightDown);
        setArmState(ArmState.BOTTOM);
    }



    // Linear Slide Methods

    public void setLinearSlidePower(double power) {
        linearSlide.setPower(power);
    }

    public void setLinearSlidePosition(double position) {
        linearSlide.setPosition(position);
        setSlideState(SlideState.CUSTOM);
    }

    private double processSlideState(){
        switch(slideState){
            case BOTTOM:
                return linearSlide.setPosition(kSlideBottom);
            case HUB:
                return linearSlide.setPosition(kSlideHub);
            case MIDDLE:
                return linearSlide.setPosition(kSlideMiddle);
            case HIGH:
                return linearSlide.setPosition(kSlideHigh);
            case STACK:
                return linearSlide.setPosition(stackIndex * kSlideStackDistance + kSlideBottom);
            case CUSTOM:
                return 0.0;
            default:
                return 0;
        }
    }
}
