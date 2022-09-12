package org.firstinspires.ftc.teamcode.shplib.controllers;

public class FFController {
    public double kV, kA, kS  = 0;

    public FFController(double kV) {
        this.kV = kV;
    }

    public FFController(double kV, double kS) {
        this.kV = kV;
        this.kS = kS;
    }

    public FFController(double kV, double kA, double kS) {
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
    }

    public double calculate(double setpoint) {
        return kV * setpoint + kA * Math.pow(setpoint, 2) + kS;
    }
}
