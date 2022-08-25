package org.firstinspires.ftc.teamcode.Constants.Arm;

public enum ArmState {
    BASE(0),
    UP((Math.PI / 2.0) - 0.3874631),
    OUT((Math.PI) - 0.3874631);

    private final double val;

    ArmState(double val) {
        this.val = val;
    }

    public double getVal() {
        return val;
    }
}