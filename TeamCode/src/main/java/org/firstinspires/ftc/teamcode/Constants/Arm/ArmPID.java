package org.firstinspires.ftc.teamcode.Constants.Arm;

public enum ArmPID {
    // TODO#3 Tune PID
    P(0.0),
    I(0.0),
    D(0.0);

    private final double val;

    ArmPID(double val) {
        this.val = val;
    }

    public double getVal() {
        return val;
    }
}
