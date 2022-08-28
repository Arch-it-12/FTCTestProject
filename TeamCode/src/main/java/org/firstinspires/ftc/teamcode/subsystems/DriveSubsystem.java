package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A wrapper for the drivetrain motors and integrated gyroscope
 *
 * @author Archit A.
 */
public class DriveSubsystem extends SubsystemBase {

    // Telemetry
    private final Telemetry telemetry;

    // Hardware
    private final BNO055IMU imu;
    private final MecanumDrive mecanumDrive;

    // Input
    private final GamepadEx driverGamepad;

    /**
     * Creates a new DriveSubsystem along with an associated MecanumDrive object
     *
     * @param driverGamepad the {@link GamepadEx} object for the driver gamepad
     * @param frontLeft     the {@link Motor} object for the front left motor
     * @param frontRight    the {@link Motor} object for the front right motor
     * @param backLeft      the {@link Motor} object for the back left motor
     * @param backRight     the {@link Motor} object for the back right motor
     * @param imu           the {@link BNO055IMU} object for the integrated gyroscope
     */
    public DriveSubsystem(GamepadEx driverGamepad, BNO055IMU imu, Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, Telemetry telemetry) {
        this.telemetry = telemetry;

        frontRight.setInverted(true);
        backRight.setInverted(true);
        imu.initialize(new BNO055IMU.Parameters());

        this.driverGamepad = driverGamepad;
        this.imu = imu;
        this.mecanumDrive = new MecanumDrive(false, frontLeft, frontRight, backLeft, backRight);
    }

    /**
     * Runs one iteration of the control loop in a field-centric manner
     */
    public void driveFieldCentric() {
        double strafe = cubeInput(driverGamepad.getLeftX());
        double forward = cubeInput(driverGamepad.getLeftY());
        double rotate = cubeInput(driverGamepad.getRightX());

        telemetry.addData("Strafe", strafe);
        telemetry.addData("Forward", forward);
        telemetry.addData("Rotate", rotate);
        telemetry.addData("Heading", -imu.getAngularOrientation().secondAngle);
        telemetry.update();

        mecanumDrive.driveFieldCentric(strafe, forward, rotate, -imu.getAngularOrientation().secondAngle);
    }

    /**
     * Runs one iteration of the control loop in a robot-centric manner
     */
    public void driveRobotCentric() {
        double strafe = cubeInput(driverGamepad.getLeftX());
        double forward = cubeInput(driverGamepad.getLeftY());
        double rotate = cubeInput(driverGamepad.getRightX());

        telemetry.addData("Strafe", strafe);
        telemetry.addData("Forward", forward);
        telemetry.addData("Rotate", rotate);
        telemetry.addData("Heading", -imu.getAngularOrientation().secondAngle);
        telemetry.update();

        mecanumDrive.driveRobotCentric(strafe, forward, rotate);
    }

    /**
     * Cubes the input value to smoothen control
     *
     * @param input value to be cubed
     * @return the cubed input value
     */
    public double cubeInput(double input) {
        return input * input * input;
    }
}
