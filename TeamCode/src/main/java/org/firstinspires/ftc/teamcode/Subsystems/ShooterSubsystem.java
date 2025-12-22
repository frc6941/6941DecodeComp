package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private final Motor leader;
    private final Motor follower;
    private final MotorGroup shooterGroup;
    private final Servo latch;

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

        latch = hardwareMap.get(Servo.class, Constants.Shooter.LATCH_NAME);
        latch.setPosition(Constants.Shooter.LATCH_CLOSED_POS);
    }

    public void setOpenLoop(final double power) {
        shooterGroup.setRunMode(Motor.RunMode.RawPower);
        shooterGroup.set(clamp(power, -1.0, 1.0));
    }

    public double getVelocityRpm() {
        return getFollowerVelocityRpm();
    }

    public void setVelocityRpm(final double rpm) {
        shooterGroup.setRunMode(Motor.RunMode.VelocityControl);
        shooterGroup.set(Math.max(0.0, rpm));
    }

    public double getLeaderVelocityRpm() {
        return leader.getCorrectedVelocity() / Constants.Shooter.TICKS_PER_REV_OUTPUT * 60.0;
    }

    public double getFollowerVelocityRpm() {
        return follower.getCorrectedVelocity() / Constants.Shooter.TICKS_PER_REV_OUTPUT * 60.0;
    }

    public void stop() {
        shooterGroup.set(0.0);
        shooterGroup.setRunMode(Motor.RunMode.RawPower);
    }

    public void openLatch() {
        setLatchPosition(Constants.Shooter.LATCH_OPEN_POS);
    }

    public void closeLatch() {
        setLatchPosition(Constants.Shooter.LATCH_CLOSED_POS);
    }

    public void setLatchPosition(final double position) {
        latch.setPosition(clamp(position, 0.0, 1.0));
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

