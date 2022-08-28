package org.firstinspires.ftc.teamcode.opModes.teleOp.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.DeviceConfig;
import org.firstinspires.ftc.teamcode.constants.MotorConfig;
import org.firstinspires.ftc.teamcode.constants.arm.ArmState;

/**
 * TeleOp Used to test the IMU
 *
 * @author Archit A.
 */
@TeleOp(name = "IMU Test", group = "Tests")
public class IMUTest extends OpMode {

    // Hardware
    private BNO055IMU imu;

    @Override
    public void init() {
        // Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware
        imu = hardwareMap.get(BNO055IMU.class, DeviceConfig.IMU.getName());
    }

    @Override
    public void loop() {
        telemetry.addData("Heading", "%f rad", imu.getAngularOrientation().secondAngle);
        telemetry.update();
    }
}
