package org.firstinspires.ftc.teamcode.constants.arm;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

/**
 * Constants for the Arm Feed Forward used in {@link ArmSubsystem}
 * Values WILL be less than 1 (~0.001)
 *
 * @author Archit A.
 */
public enum ArmFF {
    // TODO#2 Tune Feed Forward
    /**
     * Static Friction coefficient.
     * Tuned initially to make the arm move as little as possible, overcoming static friction
     */
    kS(0.0),
    /**
     * Gravity coefficient.  Tuned until arm is able to hold its position against gravity, but not enough to move the arm itself
     */
    kCos(0.0),
    /**
     * Velocity coefficient.  Tuned until the arm follows the motion profile's setpoint velocity as accurately as possible
     */
    kV(0.0);

    /**
     * The coefficient's actual double value
     */
    private final double val;

    ArmFF(double val) {
        this.val = val;
    }

    /**
     * @return the coefficient value associated to the enum
     */
    public double getVal() {
        return val;
    }
}
