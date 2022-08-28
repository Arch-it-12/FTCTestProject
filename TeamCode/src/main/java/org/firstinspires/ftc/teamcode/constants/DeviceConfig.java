package org.firstinspires.ftc.teamcode.constants;

/**
 * Driver station device config names
 *
 * @author Archit A.
 */
public enum DeviceConfig {
    ARM_MOTOR("arm motor"),
    ARM_SERVO("arm servo"),
    FRONT_LEFT_MOTOR("front left motor"),
    FRONT_RIGHT_MOTOR("front right motor"),
    BACK_LEFT_MOTOR("back left motor"),
    BACK_RIGHT_MOTOR("back right motor"),
    IMU("imu");

    private final String name;

    DeviceConfig(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }
}
