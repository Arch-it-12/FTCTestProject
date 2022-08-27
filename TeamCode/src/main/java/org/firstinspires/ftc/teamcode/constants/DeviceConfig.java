package org.firstinspires.ftc.teamcode.constants;

/**
 * Driver station device config names
 *
 * @author Archit A.
 */
public enum DeviceConfig {
    ARM_MOTOR("arm motor"),
    ARM_SERVO("arm servo");

    private final String name;

    DeviceConfig(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }
}
