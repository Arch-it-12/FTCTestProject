package org.firstinspires.ftc.teamcode.Constants;

public enum MotorConst {
    // TODO#1 Tune Max Velocity and Acceleration
    // Unit: Radians
    ARM(384.5, 2, 0, 0);

    private final double TPR;
    private final double gearRatio;
    private final double maxVel;
    private final double maxAcc;

    MotorConst(double TPR, double gearRatio, double maxVel, double maxAcc) {
        this.TPR = TPR;
        this.gearRatio = gearRatio;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
    }

    public double getTPR() {
        return TPR;
    }

    public double getGearRatio() {
        return gearRatio;
    }

    public double getMaxVel() {
        return maxVel;
    }

    public double getMaxAcc() {
        return maxAcc;
    }
}
