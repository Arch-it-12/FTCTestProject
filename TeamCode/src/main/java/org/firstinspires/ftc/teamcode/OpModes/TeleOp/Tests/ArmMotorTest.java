package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// TeleOp Used to tune the max velocity and acceleration of the arm motor

@TeleOp(name = "Arm Motor Test", group = "Tests")
public class ArmMotorTest extends OpMode {

    // Hardware
    private Motor armMotor;

    @Override
    public void init() {
        // Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware
        armMotor = hardwareMap.get(Motor.class, "arm motor");

        // Init stuff
        armMotor.setInverted(true);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotor.resetEncoder();
    }

    @Override
    public void loop() {
        if (armMotor.getCurrentPosition() < Math.PI) {
            armMotor.set(1.0);
        } else {
            armMotor.stopMotor();
        }

        telemetry.addData("Position", armMotor.getCurrentPosition());
        telemetry.addData("Velocity", armMotor.getCorrectedVelocity());
        telemetry.update();
    }
}
