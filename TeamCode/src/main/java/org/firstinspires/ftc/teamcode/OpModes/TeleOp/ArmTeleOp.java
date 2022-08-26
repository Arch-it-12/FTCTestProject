package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.ArmMoveCommand;
import org.firstinspires.ftc.teamcode.Constants.Arm.ArmState;
import org.firstinspires.ftc.teamcode.Constants.MotorConst;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

// Fully functioning TeleOp for Arm ONLY

@TeleOp(name = "Arm Only")
public class ArmTeleOp extends CommandOpMode {

    // Hardware
    private Motor armMotor;
    private Servo armServo;

    // Subsystems
    private ArmSubsystem armSubsystem;

    // Commands
    private ArmMoveCommand armBaseCommand;
    private ArmMoveCommand armUpCommand;
    private ArmMoveCommand armOutCommand;

    // Gamepads
    private GamepadEx driver;
    private GamepadEx operator;

    @Override
    public void initialize() {
        // Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware
        armMotor = hardwareMap.get(Motor.class, "arm motor");
        armServo = hardwareMap.get(Servo.class, "arm servo");

        // Subsystems
        armSubsystem = new ArmSubsystem(armMotor, armServo, MotorConst.ARM, telemetry);

        // Commands
        armBaseCommand = new ArmMoveCommand(armSubsystem, ArmState.INITIAL);
        armUpCommand = new ArmMoveCommand(armSubsystem, ArmState.UP);
        armOutCommand = new ArmMoveCommand(armSubsystem, ArmState.OUT);

        // Gamepads
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Button Binds
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(armBaseCommand);
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(armUpCommand);
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(armOutCommand);

        driver.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(
                new InstantCommand(armSubsystem::closeServo, armSubsystem),
                new InstantCommand(armSubsystem::openServo, armSubsystem)
        );
    }
}
