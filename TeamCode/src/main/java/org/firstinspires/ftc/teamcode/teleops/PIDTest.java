//package org.firstinspires.ftc.teamcode.teleops;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.BaseRobot;
//import org.firstinspires.ftc.teamcode.Constants;
//import org.firstinspires.ftc.teamcode.shplib.controllers.GainSchedule;
//import org.firstinspires.ftc.teamcode.shplib.controllers.ScheduledPIDController;
//import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
//import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
//import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
//
//@TeleOp
//public class PIDTest extends BaseRobot {
////    private ScheduledPIDController pid;
//
//    private double debounce;
//    private double setpoint = 0.0;
//
//    @Override
//    public void init() {
//        super.init();
//
////        pid = new ScheduledPIDController(5.0 * kPositionPIDFactor, 0.0, 0.0);
////        pid.scheduleGains(new GainSchedule(3.0 * kPositionPIDFactor, 0.0, 0.0, 0.90));
////        pid.scheduleGains(new GainSchedule(2.0 * kPositionPIDFactor, 0.0, 0.0, 0.50));
////        pid.scheduleGains(new GainSchedule(1.5 * kPositionPIDFactor, 0.0, 0.0, 0.20));
//        drive.enablePositionPID();
//    }
//
//    @Override
//    public void start() {
//        super.start();
//
//        // Add anything that needs to be run a single time when the OpMode starts
//        debounce = Clock.now();
//    }
//
//    @Override
//    public void loop() {
//        // Allows CommandScheduler.run() to be called - DO NOT DELETE!
//        super.loop();
//
//        if (gamepad1.a && Clock.hasElapsed(debounce, 0.5)) {
//            setpoint = Constants.kTopSetpoint;
//            drive.setInitialPositions();
////            pid.setInitialPosition(leftFront.getPosition(MotorUnit.TICKS));
//            debounce = Clock.now();
//        } else if (gamepad1.b && Clock.hasElapsed(debounce, 0.5)) {
//            setpoint = 0.0;
//            drive.setInitialPositions();
////            pid.setInitialPosition(leftFront.getPosition(MotorUnit.TICKS));
//            debounce = Clock.now();
//        }
//
//        // get the current position of the motor and calculate the output
////        double currentPosition = motor.getPosition(MotorUnit.TICKS);
////        double output = pid.calculate(currentPosition, setpoint);
//
//        // set the power of the motor to the output
////        motor.setPower(output);
//        drive.setPosition(setpoint);
//
////        telemetry.addData("Current Position Left Front: ", leftFront.getPosition(MotorUnit.TICKS));
////        telemetry.addData("Initial Position: ", pid.getInitialPosition());
//        telemetry.addData("Setpoint: ", setpoint);
////        telemetry.addData("Output: ", output);
////        pid.logGains(telemetry);
//    }
//}
