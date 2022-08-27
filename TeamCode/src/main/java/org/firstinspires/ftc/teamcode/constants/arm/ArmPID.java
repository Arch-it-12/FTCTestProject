package org.firstinspires.ftc.teamcode.constants.arm;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

/**
 * Constants for the PID Controller used in {@link ArmSubsystem}.
 * Values WILL be less than 1 (~0.001)
 *
 * @author Archit A.
 */
public enum ArmPID {
    // TODO#3 Tune PID
    /**
     * Proportional coefficient.
     * Tuned initially to maintain a steady oscillation around the setpoint
     */
    P(0.0),
    /**
     * Integral coefficient.
     * Should be 0 when used in a motion profiled system
     */
    I(0.0),
    /**
     * Derivative coefficient.
     * Tuned to limit oscillation and overshoot
     */
    D(0.0);

    /**
     * The coefficient's actual double value
     */
    private final double val;

    ArmPID(double val) {
        this.val = val;
    }

    /**
     * @return the coefficient value associated to the enum
     */
    public double getVal() {
        return val;
    }
}
