package org.firstinspires.ftc.teamcode.shplib.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.controllers.PositionPID;
import org.firstinspires.ftc.teamcode.shplib.controllers.VelocityPID;

public class SHPMotor {
    private final DcMotorEx motor;
    private VoltageSensor voltageSensor;

    private PositionPID positionPID;
    private VelocityPID velocityPID;
    private AngleUnit unit;

    private double maxVelocity = 0;

    public SHPMotor(@NonNull HardwareMap hardwareMap, String deviceName) {
        motor = hardwareMap.get(DcMotorEx.class, deviceName);
        enableVoltageCompensation(hardwareMap);
        configure();
    }

    private void configure() {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void enableVoltageCompensation(@NonNull HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void disableVoltageCompensation() {
        voltageSensor = null;
    }

    public void enablePositionPID(double kP) {
        positionPID = new PositionPID(kP, getPosition());
    }

    public void enablePositionPID(double kP, double kI, double kD) {
        positionPID = new PositionPID(kP, kI, kD, getPosition());
    }

    public void disablePositionPID() {
        positionPID = null;
    }

    public void setPositionErrorTolerance(double errorTolerance) {
        if (positionPID == null) return;
        positionPID.setErrorTolerance(errorTolerance);
    }

    public boolean atPositionSetpoint() {
        if (positionPID == null) return true;
        return positionPID.atSetpoint();
    }

    public void enableVelocityPID(double kP, double maxVelocity, AngleUnit unit) {
        setMaxVelocity(maxVelocity);
        setUnit(unit);
        velocityPID = new VelocityPID(kP, getVelocity(unit));
    }

    public void enableVelocityPID(double kP, double kI, double kD, double maxVelocity, AngleUnit unit) {
        setMaxVelocity(maxVelocity);
        setUnit(unit);
        velocityPID = new VelocityPID(kP, kI, kD, getVelocity(unit));
    }

    public void disableVelocityPID() {
        velocityPID = null;
    }

    public void setVelocityErrorTolerance(double errorTolerance) {
        if (velocityPID == null) return;
        velocityPID.setErrorTolerance(errorTolerance);
    }

    public boolean atVelocitySetpoint() {
        if (velocityPID == null) return true;
        return velocityPID.atSetpoint();
    }

    public void set(double power) {
        if (velocityPID != null && maxVelocity > 0) {
            setVelocity(power * maxVelocity);
        } else setPower(power);
    }

    public void setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    public void setPower(double power) {
        if (voltageSensor != null)
            power *= (Constants.kNominalVoltage / voltageSensor.getVoltage());
        power = Range.clip(power, -1.0, 1.0);
        if (getPower() != power)
            motor.setPower(power);
    }

    public double getPower() {
        return motor.getPower();
    }

    public void setUnit(AngleUnit unit) {
        this.unit = unit;
    }

    public AngleUnit getUnit() {
        return unit;
    }

    public void setDirection(DcMotorEx.Direction direction) {
        motor.setDirection(direction);
    }

    public DcMotorSimple.Direction getDirection() {
        return motor.getDirection();
    }

    public void setMotorEnable() {
        motor.setMotorEnable();
    }

    public void setMotorDisable() {
        motor.setMotorDisable();
    }

    public boolean isMotorEnabled() {
        return motor.isMotorEnabled();
    }

    public boolean isMotorDisabled() {
        return !motor.isMotorEnabled();
    }

    public double setVelocity(double velocity) {
        if (velocityPID == null) return 0.0;
        velocityPID.setCurrentVelocity(getVelocity(unit));
        double power = velocityPID.calculate(velocity);
        setPower(power);
        return power;
    }

    public double getVelocityTicks() {
        return motor.getVelocity();
    }

    public double getVelocity(AngleUnit unit) {
        return motor.getVelocity(unit);
    }

    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }

    public double getCurrentAlert(CurrentUnit unit) {
        return motor.getCurrentAlert(unit);
    }

    public void setCurrentAlert(double current, CurrentUnit unit) {
        motor.setCurrentAlert(current, unit);
    }

    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }

    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    public DcMotorController getController() {
        return motor.getController();
    }

    public int getPortNumber() {
        return motor.getPortNumber();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public double setPosition(double position) {
        if (positionPID == null) return 0.0;
        positionPID.setCurrentPosition(getPosition());
        double power = positionPID.calculate(position);
        setPower(power);
        return power;
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public DcMotor.RunMode getMode() {
        return motor.getMode();
    }

    public HardwareDevice.Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    public String getDeviceName() {
        return motor.getDeviceName();
    }

    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    public int getVersion() {
        return motor.getVersion();
    }

    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    public void resetEncoder() {
        resetEncoder(false);
    }

    public void resetEncoder(boolean runUsingBuiltInControl) {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (runUsingBuiltInControl) setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
