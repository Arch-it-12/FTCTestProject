package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.ArmMoveCommand;
import org.firstinspires.ftc.teamcode.constants.DeviceConfig;
import org.firstinspires.ftc.teamcode.constants.MotorConfig;
import org.firstinspires.ftc.teamcode.constants.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Fully functioning TeleOp for Arm ONLY
 *
 * @author Archit A.
 */
@TeleOp(name = "Drive Only")
@SuppressWarnings("FieldCanBeLocal")
public class DriveTeleOp extends CommandOpMode {

    // Hardware
    private Motor backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor;
    private BNO055IMU imu;

    // Subsystems
    private DriveSubsystem driveSubsystem;

    // Gamepads
    private GamepadEx driver, operator;

    @Override
    public void initialize() {
        // Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware
        imu = hardwareMap.get(BNO055IMU.class, DeviceConfig.IMU.getName());

        frontLeftMotor = hardwareMap.get(Motor.class, DeviceConfig.FRONT_LEFT_MOTOR.getName());
        frontRightMotor = hardwareMap.get(Motor.class, DeviceConfig.FRONT_RIGHT_MOTOR.getName());
        backLeftMotor = hardwareMap.get(Motor.class, DeviceConfig.BACK_LEFT_MOTOR.getName());
        backRightMotor = hardwareMap.get(Motor.class, DeviceConfig.BACK_RIGHT_MOTOR.getName());

        // Subsystems
        driveSubsystem = new DriveSubsystem(driver, imu, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, telemetry);

        // Gamepads
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Default commands
        register(driveSubsystem);

        driveSubsystem.setDefaultCommand(
                new RunCommand(driveSubsystem::driveFieldCentric, driveSubsystem)
        );
    }
}
