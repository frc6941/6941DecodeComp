package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.tuning.ShooterPidTuning;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx leader;
    private final DcMotorEx follower;
    ElapsedTime timer = new ElapsedTime();
    private Telemetry telemetry;
    private boolean atTargetRpm = false;
    private boolean wasWithinTolerance = false;

    private double targetRpm = 0.0;
    private double lastAppliedPower = 0.0;

    public ShooterSubsystem(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this(hardwareMap);
        this.telemetry = telemetry;
    }

    public ShooterSubsystem(final HardwareMap hardwareMap) {
        telemetry = null;
        leader = hardwareMap.get(DcMotorEx.class, Constants.Shooter.LEADER_NAME);
        leader.setDirection(Constants.Shooter.LEADER_INVERTED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        follower = hardwareMap.get(DcMotorEx.class, Constants.Shooter.FOLLOWER_NAME);
        follower.setDirection(Constants.Shooter.FOLLOWER_INVERTED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        leader.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        follower.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setOpenLoop(final double power) {
        lastAppliedPower = clamp(power, -1.0, 1.0);
        leader.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leader.setPower(lastAppliedPower);
        follower.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        follower.setPower(lastAppliedPower);
    }

    public double getVelocityRpm() {
        return getLeaderVelocityRpm();
    }

    public void setVelocityRpm(final double rpm) {
        leader.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leader.setVelocity(rpm / 60.0 * Constants.Shooter.TICKS_PER_REV_OUTPUT);
        follower.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        follower.setVelocity(rpm / 60.0 * Constants.Shooter.TICKS_PER_REV_OUTPUT);
        targetRpm = rpm;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getLastAppliedPower() {
        return lastAppliedPower;
    }

    public boolean atTargetRpm() {
        return atTargetRpm;
    }

    public double getLeaderVelocityRpm() {
        return leader.getVelocity()
                / Constants.Shooter.TICKS_PER_REV_OUTPUT
                * 60.0
                * Constants.Shooter.OUTPUT_TO_WHEEL_RATIO;
    }

    public double getFollowerVelocityRpm() {
        return follower.getVelocity()
                / Constants.Shooter.TICKS_PER_REV_OUTPUT
                * 60.0
                * Constants.Shooter.OUTPUT_TO_WHEEL_RATIO;
    }

    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public void periodic() {
        if (telemetry != null) {
            telemetry.addData("heartBeat", leader.getPower());
        }
        setPIDF(ShooterPidTuning.kP, ShooterPidTuning.kI, ShooterPidTuning.kD, ShooterPidTuning.kF);
        boolean withinTolerance = targetRpm > 0.0 && Math.abs(targetRpm - getVelocityRpm()) <= ShooterPidTuning.RPM_TOLERANCE;
        if (withinTolerance) {
            if (!wasWithinTolerance) {
                timer.reset();
            }
            atTargetRpm = timer.seconds() >= 1;
        } else {
            atTargetRpm = false;
            timer.reset();
        }
        wasWithinTolerance = withinTolerance;
    }

    public void setPIDF(double kP, double kI, double kD, double kF) {
        leader.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
        follower.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
    }

    public PIDFCoefficients getPIDF() {
        return leader.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private double clamp(final double value, final double min, final double max) {
        return Math.max(min, Math.min(max, value));
    }
}
