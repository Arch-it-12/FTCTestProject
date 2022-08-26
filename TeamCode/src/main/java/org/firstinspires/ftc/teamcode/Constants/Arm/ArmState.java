package org.firstinspires.ftc.teamcode.Constants.Arm;

import org.firstinspires.ftc.teamcode.Constants.MotorConst;

public enum ArmState {
    INITIAL(22.2 * (Math.PI / 180)),
    UP(Math.PI / 2.0),
    OUT(Math.PI);

    private final double val;

    ArmState(double val) {
        this.val = val;
    }

    public double getVal() {
        return val;
    }
}