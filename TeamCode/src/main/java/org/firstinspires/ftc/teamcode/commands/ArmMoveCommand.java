package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

/**
 * Commands the arm to the target position using motion profiling, feedforward, and PID
 *
 * @author Archit A.
 * @see ArmSubsystem
 * @see ArmState
 */
public class ArmMoveCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final ArmState targetState;

    /**
     * Creates a new ArmMoveCommand to command the arm to move to the specified target state
     *
     * @param armSubsystem The associated {@link ArmSubsystem}
     * @param targetState  The target {@link ArmState} to be associated to the command
     */
    public ArmMoveCommand(ArmSubsystem armSubsystem, ArmState targetState) {
        this.armSubsystem = armSubsystem;
        this.targetState = targetState;
    }

    /**
     * Initializes the commanding of the arm within the subsystem
     */
    @Override
    public void initialize() {
        armSubsystem.setArmState(targetState);
    }

    /**
     * Control loop for motion profiling, feed forward, and feedback control
     */
    @Override
    public void execute() {
        armSubsystem.operateArm();
    }

    /**
     * Establishes the end of the command based on the motion profile
     *
     * @return whether the motion profile, and thus arm movement, is complete
     */
    @Override
    public boolean isFinished() {
        return armSubsystem.isProfileFinished();
    }

    /**
     * Stops the arm
     *
     * @param interrupted Whether the command was interrupted or not
     */
    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopArm();
    }
}
