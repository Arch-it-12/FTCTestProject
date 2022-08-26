package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Arm.ArmState;
import org.firstinspires.ftc.teamcode.Constants.MotorConst;

@Config
public class ArmSubsystem extends SubsystemBase {

    // Dashboard Constants
    public static double kS;
    public static double kCos;
    public static double kV;

    public static double P;
    public static double I;
    public static double D;

    // Telemetry
    private final Telemetry telemetry;

    // Hardware
    private final Motor armMotor;
    private final Servo armServo;

    // Constants
    private final TrapezoidProfile.Constraints motionProfileConstraints;

    // Controllers
    private final PIDController controller;
    private /*final*/ ArmFeedforward feedforward;

    // Timer
    private final ElapsedTime timer = new ElapsedTime();

    // Motion profile
    private TrapezoidProfile motionProfile;

    public ArmSubsystem(Motor armMotor, Servo armServo, @NonNull MotorConst motorConst, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.armMotor = armMotor;
        this.armServo = armServo;

        this.armMotor.setDistancePerPulse((motorConst.getTPR() * motorConst.getGearRatio()) / (2 * Math.PI));

//        this.controller = new PIDController(
//                ArmPID.P.getVal(),
//                ArmPID.I.getVal(),
//                ArmPID.D.getVal()
//        );

//        this.feedforward = new ArmFeedforward(
//                ArmFF.kS.getVal(),
//                ArmFF.kCos.getVal(),
//                ArmFF.kV.getVal()
//        );

        // TODO Tune PID
        this.controller = new PIDController(P, I, D);

        // TODO Tune Feedforward
        this.feedforward = new ArmFeedforward(kS, kCos, kV);

        this.motionProfileConstraints = new TrapezoidProfile.Constraints(
                motorConst.getMaxVel(),
                motorConst.getMaxAcc()
        );

        this.armMotor.setInverted(true);
        this.armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.armMotor.resetEncoder();
    }

    public void setArmState(@NonNull ArmState targetState) {
        // TODO For Realtime PID and FF Tuning
        controller.setPID(P, I, D);
        feedforward = new ArmFeedforward(kS, kCos, kV);

        motionProfile = new TrapezoidProfile(
                motionProfileConstraints,
                new TrapezoidProfile.State(targetState.getVal(), 0),
                new TrapezoidProfile.State(getCorrectedDistance(), armMotor.getRate())
        );

        timer.reset();
    }

    public void operateArm() {
        TrapezoidProfile.State setpoint = motionProfile.calculate(timer.seconds());
        double ff = feedforward.calculate(setpoint.position, setpoint.velocity);
        double error = controller.calculate(getCorrectedDistance(), setpoint.position);

        telemetry.addData("Target Position", "%f rad", setpoint.position);
        telemetry.addData("Target Velocity", "%f rad/s", setpoint.velocity);
        telemetry.addData("Actual Position", "%f rad", getCorrectedDistance());
        telemetry.addData("Actual Velocity", "%f rad/s", armMotor.getRate());
        telemetry.addData("Feed Forward", ff);
        telemetry.addData("PID Error", error);
        telemetry.addData("Raw Power", ff + error);
        telemetry.update();

        armMotor.set(error + ff);
    }

    public void stopArm() {
        armMotor.stopMotor();
    }

    public boolean isProfileFinished() {
        return motionProfile.isFinished(timer.seconds());
    }

    public void openServo() {
        armServo.setPosition(0.0);
    }

    public void closeServo() {
        armServo.setPosition(1.0);
    }

    private double getCorrectedDistance() {
        return armMotor.getDistance() + ArmState.INITIAL.getVal();
    }
}
