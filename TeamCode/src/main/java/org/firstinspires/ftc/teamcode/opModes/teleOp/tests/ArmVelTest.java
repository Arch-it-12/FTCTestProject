package org.firstinspires.ftc.teamcode.opModes.teleOp.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.DeviceConfig;
import org.firstinspires.ftc.teamcode.constants.MotorConst;
import org.firstinspires.ftc.teamcode.constants.arm.ArmState;

/**
 * TeleOp Used to tune the max velocity and acceleration of the arm motor
 *
 * @author Archit A.
 */
@TeleOp(name = "Arm Velocity Test", group = "Tests")
public class ArmVelTest extends OpMode {

    // Constants
    private final double dpp = (MotorConst.ARM.getTPR() * MotorConst.ARM.getGearRatio()) / (2 * Math.PI);

    // Hardware
    private Motor armMotor;

    private double getCorrectedPosition() {
        return armMotor.getDistance() + ArmState.INITIAL.getVal();
    }

    @Override
    public void init() {
        // Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware
        armMotor = hardwareMap.get(Motor.class, DeviceConfig.ARM_MOTOR.getName());

        // Init stuff
        armMotor.setInverted(true);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotor.setDistancePerPulse(dpp);

        armMotor.resetEncoder();
    }

    @Override
    public void loop() {
        if (getCorrectedPosition() < Math.PI) {
            armMotor.set(1.0);
        } else {
            armMotor.stopMotor();
        }

        telemetry.addData("Position", "%f rad", getCorrectedPosition());
        telemetry.addData("Velocity", "%f rad/s", armMotor.getRate());
        telemetry.addData("Acceleration", "%f rad/s/s", armMotor.encoder.getAcceleration() * dpp);
        telemetry.update();
    }
}
