package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.tuning.ShooterPidTuning;

public class ShooterSubsystem extends SubsystemBase {

    private final MotorEx leader;
    private final MotorEx follower;

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
    }

    public void setOpenLoop(final double power) {
        lastAppliedPower = clamp(power, -1.0, 1.0);
        leader.set(lastAppliedPower);
        followLeader();
    }

    public void followLeader() {
        follower.setRunMode(Motor.RunMode.RawPower);
        follower.set(leader.get());
    }

    public double getVelocityRpm() {
        return getLeaderVelocityRpm();
    }

    public void setVelocityRpm(final double rpm) {
        leader.setRunMode(Motor.RunMode.VelocityControl);
        leader.setVelocity(rpm / 60.0 * 2 * Math.PI, AngleUnit.RADIANS);
        targetRpm = rpm;
        followLeader();
    }

    public double getTargetRpm() {
        return targetRpm;
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
        setOpenLoop(0.0);
    }

    @Override
    public void periodic() {
        leader.setVeloCoefficients(ShooterPidTuning.kP, ShooterPidTuning.kI, ShooterPidTuning.kD);
    }

    private MotorEx buildShooterMotor(final HardwareMap hardwareMap,
            final String name,
            final boolean inverted) {
        final MotorEx motor = new MotorEx(hardwareMap, name, Constants.Shooter.GEARING);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setInverted(inverted);
        return motor;
    }

    private double clamp(final double value, final double min, final double max) {
        return Math.max(min, Math.min(max, value));
    }
}
