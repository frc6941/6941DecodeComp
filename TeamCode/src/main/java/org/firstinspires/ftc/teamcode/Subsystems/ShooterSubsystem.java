package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.tuning.ShooterPidTuning;

public class ShooterSubsystem extends SubsystemBase {

    private final Motor leader;
    private final Motor follower;
    private final MotorGroup shooterGroup;

    private final PIDController velocityController = new PIDController(0.0, 0.0, 0.0);
    private boolean velocityClosedLoopEnabled = false;
    private double targetRpm = 0.0;
    private double lastAppliedPower = 0.0;

    public ShooterSubsystem(final HardwareMap hardwareMap) {
        leader = buildShooterMotor(
                hardwareMap,
                Constants.Shooter.LEADER_NAME,
                Constants.Shooter.LEADER_INVERTED
        );
        follower = buildShooterMotor(
                hardwareMap,
                Constants.Shooter.FOLLOWER_NAME,
                Constants.Shooter.FOLLOWER_INVERTED
        );
        shooterGroup = new MotorGroup(leader, follower);
    }

    public void setOpenLoop(final double power) {
        setVelocityClosedLoopEnabled(false);
        shooterGroup.setRunMode(Motor.RunMode.RawPower);
        lastAppliedPower = clamp(power, -1.0, 1.0);
        shooterGroup.set(lastAppliedPower);
    }

    public double getVelocityRpm() {
        return getLeaderVelocityRpm();
    }

    public void setVelocityRpm(final double rpm) {
        setTargetRpm(rpm);
        setVelocityClosedLoopEnabled(true);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void setTargetRpm(final double rpm) {
        targetRpm = Math.max(0.0, rpm);
    }

    public boolean isVelocityClosedLoopEnabled() {
        return velocityClosedLoopEnabled;
    }

    public void setVelocityClosedLoopEnabled(final boolean enabled) {
        if (velocityClosedLoopEnabled == enabled) {
            return;
        }
        velocityClosedLoopEnabled = enabled;
        velocityController.reset();
    }

    public double getLastAppliedPower() {
        return lastAppliedPower;
    }

    public double getVelocityErrorRpm() {
        return targetRpm - getVelocityRpm();
    }

    public boolean atTargetRpm() {
        return atTargetRpm(ShooterPidTuning.RPM_TOLERANCE);
    }

    public boolean atTargetRpm(double toleranceRps) {
        return Math.abs(getVelocityErrorRpm()) <= toleranceRps;
    }

    public double getLeaderVelocityRpm() {
        return leader.getCorrectedVelocity()
                / Constants.Shooter.TICKS_PER_REV_OUTPUT
                * 60.0
                * Constants.Shooter.OUTPUT_TO_WHEEL_RATIO;
    }

    public double getFollowerVelocityRpm() {
        return follower.getCorrectedVelocity()
                / Constants.Shooter.TICKS_PER_REV_OUTPUT
                * 60.0
                * Constants.Shooter.OUTPUT_TO_WHEEL_RATIO;
    }

    public double getFollowerPosition() {
        return follower.getCurrentPosition();
    }

    public void stop() {
        setVelocityClosedLoopEnabled(false);
        lastAppliedPower = 0.0;
        shooterGroup.set(lastAppliedPower);
        shooterGroup.setRunMode(Motor.RunMode.RawPower);
    }


    @Override
    public void periodic() {
        if (!velocityClosedLoopEnabled) {
            return;
        }

        // Dashboard 热更新 PID 参数
        velocityController.setPID(ShooterPidTuning.kP, ShooterPidTuning.kI, ShooterPidTuning.kD);
        velocityController.setTolerance(ShooterPidTuning.RPM_TOLERANCE);
        velocityController.setSetPoint(targetRpm);

        final double currentRpm = getVelocityRpm();
        final double pid = velocityController.calculate(currentRpm);
        final double ff = ShooterPidTuning.kF * targetRpm;

        lastAppliedPower = clamp(
                ff + pid,
                ShooterPidTuning.MIN_POWER,
                ShooterPidTuning.MAX_POWER
        );

        shooterGroup.setRunMode(Motor.RunMode.RawPower);
        shooterGroup.set(lastAppliedPower);
    }

    private Motor buildShooterMotor(final HardwareMap hardwareMap,
                                    final String name,
                                    final boolean inverted) {
        final Motor motor = new Motor(hardwareMap, name, Constants.Shooter.GEARING);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setInverted(inverted);
        return motor;
    }

    private double clamp(final double value, final double min, final double max) {
        return Math.max(min, Math.min(max, value));
    }
}
