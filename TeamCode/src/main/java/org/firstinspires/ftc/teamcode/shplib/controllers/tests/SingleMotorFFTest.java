package org.firstinspires.ftc.teamcode.shplib.controllers.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.util.RegressionUtil;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.utility.LinearRegression;

import java.util.ArrayList;

@Autonomous
public class SingleMotorFFTest extends LinearOpMode {

    final String MOTOR_NAME = "motor";
    final double LIMIT_TICKS = 1000;
    final AngleUnit ANGLE_UNIT = null;

    final double VOLTAGE_RAMP = 0.25; // V/s

    @Override
    public void runOpMode() {
        SHPMotor motor = new SHPMotor(hardwareMap, MOTOR_NAME);
        ElapsedTime timer = new ElapsedTime();

        ArrayList<Double> positions = new ArrayList<>();
        ArrayList<Double> powers = new ArrayList<>();
//        ArrayList<Double> velocities = new ArrayList<>();
        ArrayList<Double> times = new ArrayList<>();

        telemetry.addData(">", "Press start to begin the test.");

        waitForStart();

        telemetry.clearAll();
        timer.reset();
        motor.resetEncoder();

        while (opModeIsActive() && motor.getPosition() < LIMIT_TICKS) {
            double seconds = timer.seconds();
            double power = seconds * VOLTAGE_RAMP;

            times.add(timer.seconds());
            positions.add((double) motor.getPosition());
            powers.add(power);
//            double velocity = ANGLE_UNIT == null ? motor.getVelocityTicks() : motor.getVelocity(ANGLE_UNIT);
//            telemetry.addData("velocity: ", velocity);
//            velocities.add(velocity);
            motor.setPower(power);
        }
        motor.setPower(0);

        RegressionUtil.RampResult result = RegressionUtil.fitRampData(times, positions, powers, true, null);
        telemetry.addData("kV: ", result.kV);
        telemetry.addData("kS: ", result.kStatic);
        telemetry.addData("R^2 (Accuracy): ", result.rSquare);

//        LinearRegression graph = new LinearRegression(times.stream().mapToDouble(d -> d).toArray(), velocities.stream().mapToDouble(d -> d).toArray());
//        telemetry.addData("kV: ", graph.slope());
    }
}
