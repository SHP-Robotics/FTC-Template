package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.controllers.ElevatorFFController;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class ArmSubsystem extends Subsystem {
    private final Servo claw;
//    private final DistanceSensor poleSensor;
    private final SHPMotor leftSlide;
    private final SHPMotor rightSlide;

    public enum State {
        BOTTOM, HUB, LOW, MIDDLE, HIGH
    }

    private State state;

    public ArmSubsystem(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, Constants.Arm.kClawName);

//        poleSensor = hardwareMap.get(DistanceSensor.class, "coneSensor");

        leftSlide = new SHPMotor(hardwareMap, Constants.Arm.kLeftSlideName);
        leftSlide.reverseDirection();
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftSlide.enablePositionPID(Constants.Arm.kSlideP, Constants.Arm.kSlideD);
        leftSlide.setPositionErrorTolerance(Constants.Arm.kSlideTolerance);
        leftSlide.enableFF(new ElevatorFFController(Constants.Arm.kSlideS, Constants.Arm.kSlideG));

        rightSlide = new SHPMotor(hardwareMap, Constants.Arm.kRightSlideName);
//        rightSlide.reverseDirection();
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlide.enablePositionPID(Constants.Arm.kSlideP, Constants.Arm.kSlideD);
        rightSlide.setPositionErrorTolerance(Constants.Arm.kSlideTolerance);
        rightSlide.enableFF(new ElevatorFFController(Constants.Arm.kSlideS, Constants.Arm.kSlideG));

        setState(State.BOTTOM);
    }

    public void setState(State state) {
        this.state = state;
    }

    public void openClaw() {
        if (isClawClosed()) claw.setPosition(Constants.Arm.kClawOpen);
    }

    public void closeClaw() {
        if (!isClawClosed()) claw.setPosition(Constants.Arm.kClawClosed);
    }

    public boolean isClawClosed() {
        return claw.getPosition() == Constants.Arm.kClawClosed;
    }

//    public boolean isOverPole() {
//        return poleSensor.getDistance(DistanceUnit.INCH) <= 6.0;
//    }

    @Override
    public void periodic(Telemetry telemetry) {
//        telemetry.addData("Slide State: ", state.toString());
        telemetry.addData("Left Slide Position: ", leftSlide.getPosition(MotorUnit.TICKS));
        telemetry.addData("Right Slide Position: ", rightSlide.getPosition(MotorUnit.TICKS));
//        telemetry.addData("Pole Distance (in): ", poleSensor.getDistance(DistanceUnit.INCH));

        switch (state) {
            case BOTTOM:
//                telemetry.addData("Power: ", slide.setPosition(10.0));
                leftSlide.setPosition(10.0);
                rightSlide.setPosition(10.0);
                break;
            case HUB:
                break;
            case LOW:
                leftSlide.setPosition(1000.0);
                rightSlide.setPosition(1000.0);
                break;
            case MIDDLE:
                leftSlide.setPosition(2000.0);
                rightSlide.setPosition(2000.0);
                break;
            case HIGH:
                leftSlide.setPosition(4000.0);
                rightSlide.setPosition(4000.0);
                break;
        }
    }
}
