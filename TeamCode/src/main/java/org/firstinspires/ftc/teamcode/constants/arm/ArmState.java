package org.firstinspires.ftc.teamcode.constants.arm;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

/**
 * Constants for the Arm's State used in {@link ArmSubsystem}.
 * Values will be in absolute radians; The arm at 0 radians should be normal to the gravity vector
 *
 * @author Archit A.
 */
public enum ArmState {
    /**
     * The initial arm position.
     * Should be empirically measured when arm is at rest
     */
    INITIAL(22.2 * (Math.PI / 180)),
    /**
     * Vertical arm position
     */
    UP(Math.PI / 2.0),
    /**
     * Horizontal arm position
     */
    OUT(Math.PI);

    /**
     * The arm state's actual radian value
     */
    private final double val;

    ArmState(double val) {
        this.val = val;
    }

    /**
     * @return the arm state value associated to the enum
     */
    public double getVal() {
        return val;
    }
}