package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.Arm.ArmState;
import org.firstinspires.ftc.teamcode.Constants.MotorConst;

// TeleOp Used to tune the max velocity and acceleration of the arm motor

@TeleOp(name = "Arm Position Test", group = "Tests")
public class ArmPosTest extends OpMode {

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
        armMotor = hardwareMap.get(Motor.class, "arm motor");

        // Init stuff
        armMotor.setInverted(true);
        armMotor.setDistancePerPulse(dpp);

        armMotor.resetEncoder();
    }

    @Override
    public void loop() {
        telemetry.addData("Position", "%f rad", getCorrectedPosition());
        telemetry.update();
    }
}
