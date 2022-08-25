package org.firstinspires.ftc.teamcode.Constants.Arm;

public enum ArmFF {
    // TODO#2 Tune Feed Forward
    kS(0.0),
    kCos(0.0),
    kV(0.0);

    private final double val;

    ArmFF(double val) {
        this.val = val;
    }

    public double getVal() {
        return val;
    }
}
