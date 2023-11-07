package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.controllers.ElevatorFFController;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

// Use this class as a reference for creating new subsystems

public class ArmSubsystem extends Subsystem {
    public final SHPMotor slide;
    public boolean override;
    double stateEncoderValue;
    public int coneLevel;
    private double manualPosition;

    public enum State {
        TOP,
        TOP_OF_TOP,
        MIDDLE,
        BOTTOM,
        SHORT,
        TOP_OF_SHORT,
        TOP_OF_MIDDLE,
        CARRYING,
        STACKED_CONES,
        MANUAL,
    }

    private State state;

    private double previousTime;

    public ArmSubsystem(HardwareMap hardwareMap) {
        slide = new SHPMotor(hardwareMap, Constants.Arm.K_SLIDE_NAME, MotorUnit.TICKS);
        slide.reverseDirection();
        slide.enablePositionPID(Constants.Arm.K_SLIDE_P);
        slide.setPositionErrorTolerance(Constants.Arm.K_SLIDE_TOLERANCE);
        slide.enableFF(new ElevatorFFController(0, Constants.Arm.K_SLIDE_G));
        coneLevel = 5;
        stateEncoderValue = Constants.Arm.K_SLIDE_TOP;
        /*
         slide.enableVelocityPID(Constants.Arm.kSlideP);
         slide.enableProfiling(Constants.Arm.kSlideMaxVelocity);
        */


        manualPosition = 0;
        previousTime = Clock.now();
        setState(State.BOTTOM);
    }

    public void resetEncoder() {
        slide.resetEncoder();
    }

    public void setState(State state) {
        if (state == State.STACKED_CONES)
            incrementConeLevelDown();
        this.state = state;
        previousTime = Clock.now();
    }

//    public double getCarryingDriveBias() {
//        if (getState() != State.BOTTOM && getState() != State.CARRYING) {
//            if (slide.getPosition(MotorUnit.TICKS) > stateEncoderValue * 0.9)
//                return 0.3;
//            else if (slide.getPosition(MotorUnit.TICKS) > stateEncoderValue * 0.2)
//                return Math.abs(slide.getPosition(MotorUnit.TICKS) / stateEncoderValue - 1.0);
//            else
//                return 0.8;
//        }
//        else {
//            return 0.8;
//        }
//    }


    public double getDriveBias(boolean clawOpen) {
        if(getState() != State.BOTTOM && getState() != State.CARRYING && getState() != State.STACKED_CONES){
            if (slide.getPosition(MotorUnit.TICKS) > stateEncoderValue * 0.95)
                return 0.3;
            else if (slide.getPosition(MotorUnit.TICKS) > stateEncoderValue * 0.5)
                return Math.abs(slide.getPosition(MotorUnit.TICKS) / stateEncoderValue - 1.0);
            else
                return 0.7;
        }
        else if (slide.getPosition(MotorUnit.TICKS)<3500)
            return Math.abs(slide.getPosition(MotorUnit.TICKS) / Constants.Arm.K_SLIDE_TOP - 1.0);
        else if (getState() == State.MIDDLE) {
            return 0.5;
        }
        else
            return 0.8;
    }
    public void nextState() {
        if (this.state == State.MIDDLE) setState(State.TOP);
        else if (this.state == State.SHORT) setState(State.MIDDLE);
        else if (this.state == State.CARRYING) setState(State.SHORT);
        else if (this.state == State.BOTTOM) setState(State.CARRYING);
        else if (this.state == State.STACKED_CONES) setState(State.MIDDLE);
    }

    public void setManualPos(double encoderValue) {

        if (encoderValue < Constants.Arm.K_SLIDE_BOTTOM)
            manualPosition = Constants.Arm.K_SLIDE_BOTTOM;
        if (encoderValue > Constants.Arm.K_SLIDE_TOP + 200)
            manualPosition = Constants.Arm.K_SLIDE_TOP;
        manualPosition = encoderValue;
        this.state = State.MANUAL;

    }

    public double getManualPosition() {
        return manualPosition;
    }

    public State getState() {
        return state;
    }

    public void incrementConeLevelDown() {
        if (coneLevel <= 0)
            coneLevel = 5;
        else
            coneLevel--;
    }

    public int getConeLevel() {
        return coneLevel;
    }
    public void incrementConeLevelUp() {
        coneLevel++;
    }

    public void previousState() {
        if (this.state == State.TOP) setState(State.TOP_OF_TOP);
        else if (this.state == State.MANUAL && manualPosition >3200) setState(State.TOP_OF_TOP);
        else if (this.state == State.MANUAL && manualPosition >2000) setState(State.TOP_OF_MIDDLE);
        else if (this.state == State.MANUAL && manualPosition >1000) setState(State.TOP_OF_SHORT);
        else if (this.state == State.MANUAL) setState(State.CARRYING);
        else if (this.state == State.MIDDLE) setState(State.TOP_OF_MIDDLE);
        else if (this.state == State.SHORT) setState(State.TOP_OF_SHORT);
        else if (this.state == State.CARRYING) setState(State.BOTTOM);

    }

    public boolean atBottom() {
        return this.state == State.BOTTOM;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("slide ticks: ", slide.getPosition(MotorUnit.TICKS));
        telemetry.addData("stateEncoderValue: ", stateEncoderValue);


//        telemetry.addData("profile output: ", slide.followProfile(Clock.elapsed(previousTime)));
        if (!override) {
            switch (state) {
                case TOP:
                    slide.setPosition(Constants.Arm.K_SLIDE_TOP);
                    stateEncoderValue = Constants.Arm.K_SLIDE_TOP;
                    telemetry.addData("state: ", "TOP");
                    break;
                case CARRYING:
                    slide.setPosition(Constants.Arm.K_SLIDE_CARRY);
                    telemetry.addData("state: ", "Carrying");
                    break;
                case MIDDLE:
                    slide.setPosition(Constants.Arm.K_SLIDE_MIDDLE);
                    stateEncoderValue = Constants.Arm.K_SLIDE_MIDDLE;
                    telemetry.addData("state: ", "Middle");
                    break;
                case TOP_OF_MIDDLE:
                    slide.setPosition(Constants.Arm.K_SLIDE_MIDDLE - 600);
                    break;
                case BOTTOM:
                    slide.setPosition(Constants.Arm.K_SLIDE_BOTTOM);
                    stateEncoderValue = Constants.Arm.K_SLIDE_BOTTOM;
                    telemetry.addData("state: ", "BOTTOM");
                    break;
                case SHORT:
                    slide.setPosition(Constants.Arm.K_SLIDE_SHORT);
                    stateEncoderValue = Constants.Arm.K_SLIDE_SHORT;
                    break;
                case TOP_OF_SHORT:
                    slide.setPosition(Constants.Arm.K_SLIDE_SHORT - 400);
                    break;
                case TOP_OF_TOP:
                    slide.setPosition(Constants.Arm.K_SLIDE_TOP - 500);
                    break;
                case STACKED_CONES:
                    slide.setPosition(coneLevel*165);
                    break;
                case MANUAL:
                    slide.setPosition(manualPosition);
                    break;
            }
        }
        else {
            telemetry.addData("Current Power", slide.getPower());
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
