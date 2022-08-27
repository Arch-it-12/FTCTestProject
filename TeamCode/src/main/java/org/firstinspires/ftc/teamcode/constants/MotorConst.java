package org.firstinspires.ftc.teamcode.constants;

/**
 * Constants used in motion profiling for the various motors in the robot
 * @author Archit A.
 */
public enum MotorConst {
    // TODO#1 Tune Max Velocity and Acceleration
    /**
     * Arm motor; Unit: RADIANS
     */
    ARM(384.5, 2, 0, 0);

    /**
     * Ticks per revolution (Encoder Resolution) of the motor.  Can be found on GoBilda website
     */
    private final double TPR;
    /**
     * Gear ratio between the motor and the moving part.  Calculated as PART gear / MOTOR gear
     */
    private final double gearRatio;
    /**
     * The maximum achievable velocity of the motor in the specified unit / s
     */
    private final double maxVel;
    /**
     * The maximum achievable acceleration of the motor in the specified unit / s^2
     */
    private final double maxAcc;

    MotorConst(double TPR, double gearRatio, double maxVel, double maxAcc) {
        this.TPR = TPR;
        this.gearRatio = gearRatio;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
    }

    /**
     * @return the ticks per revolution of the motor
     */
    public double getTPR() {
        return TPR;
    }

    /**
     * @return the gear ratio associated with the motor
     */
    public double getGearRatio() {
        return gearRatio;
    }

    /**
     * @return the maximum velocity of the motor in the specified unit / s
     */
    public double getMaxVel() {
        return maxVel;
    }

    /**
     * @return the maximum acceleration of the motor in the specified unit / s^2
     */
    public double getMaxAcc() {
        return maxAcc;
    }
}
