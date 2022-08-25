package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Constants.Arm.ArmState;

public class ArmMoveCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final ArmState targetState;

    public ArmMoveCommand(ArmSubsystem armSubsystem, ArmState targetState) {
        this.armSubsystem = armSubsystem;
        this.targetState = targetState;
    }

    @Override
    public void initialize() {
        armSubsystem.setArmState(targetState);
    }

    @Override
    public void execute() {
        armSubsystem.operateArm();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isProfileFinished();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopArm();
    }
}
