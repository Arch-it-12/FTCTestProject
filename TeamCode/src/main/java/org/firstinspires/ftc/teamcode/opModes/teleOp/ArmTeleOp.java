package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.ArmMoveCommand;
import org.firstinspires.ftc.teamcode.constants.DeviceConfig;
import org.firstinspires.ftc.teamcode.constants.MotorConst;
import org.firstinspires.ftc.teamcode.constants.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

/**
 * Fully functioning TeleOp for Arm ONLY
 *
 * @author Archit A.
 */
@TeleOp(name = "Arm Only")
@SuppressWarnings("FieldCanBeLocal")
public class ArmTeleOp extends CommandOpMode {

    // Hardware
    private Motor armMotor;
    private Servo armServo;

    // Subsystems
    private ArmSubsystem armSubsystem;

    // Commands
    private ArmMoveCommand armInitialCommand, armUpCommand, armOutCommand;

    // Gamepads
    private GamepadEx driver, operator;

    @Override
    public void initialize() {
        // Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware
        armMotor = hardwareMap.get(Motor.class, DeviceConfig.ARM_MOTOR.getName());
        armServo = hardwareMap.get(Servo.class, DeviceConfig.ARM_SERVO.getName());

        // Subsystems
        armSubsystem = new ArmSubsystem(armMotor, armServo, MotorConst.ARM, telemetry);

        // Commands
        armInitialCommand = new ArmMoveCommand(armSubsystem, ArmState.INITIAL);
        armUpCommand = new ArmMoveCommand(armSubsystem, ArmState.UP);
        armOutCommand = new ArmMoveCommand(armSubsystem, ArmState.OUT);

        // Gamepads
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Button Binds
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(armInitialCommand);
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(armUpCommand);
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(armOutCommand);

        driver.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(
                new InstantCommand(armSubsystem::closeServo, armSubsystem),
                new InstantCommand(armSubsystem::openServo, armSubsystem)
        );
    }
}
