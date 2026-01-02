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

public class AutoShooterSubsystem extends SubsystemBase {

    private final DcMotorEx leader;
    private final DcMotorEx follower;
    private final ElapsedTime timer = new ElapsedTime();
    private Telemetry telemetry;

    private boolean atTargetRpm = false;
    private boolean wasWithinTolerance = false;
    private double targetRpm = 0.0;

    public AutoShooterSubsystem(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this(hardwareMap);
        this.telemetry = telemetry;
    }

    public AutoShooterSubsystem(final HardwareMap hardwareMap) {
        leader = hardwareMap.get(DcMotorEx.class, Constants.Shooter.LEADER_NAME);
        follower = hardwareMap.get(DcMotorEx.class, Constants.Shooter.FOLLOWER_NAME);

        leader.setDirection(Constants.Shooter.LEADER_INVERTED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        follower.setDirection(Constants.Shooter.FOLLOWER_INVERTED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        leader.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        follower.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leader.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        follower.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        updatePIDF();
    }

    public void updatePIDF() {
        PIDFCoefficients coeffs = new PIDFCoefficients(
                ShooterPidTuning.kP,
                ShooterPidTuning.kI,
                ShooterPidTuning.kD,
                ShooterPidTuning.kF
        );
        leader.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
        follower.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
    }

    public void setVelocityRpm(final double rpm) {
        this.targetRpm = rpm;
        if (rpm == 0) {
            stop();
            return;
        }
        if (leader.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
            leader.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            follower.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        double ticksPerSec = (rpm / 60.0) * Constants.Shooter.TICKS_PER_REV_OUTPUT;
        leader.setVelocity(ticksPerSec);
        follower.setVelocity(ticksPerSec);
    }

    public void setOpenLoop(final double power) {
        targetRpm = 0;
        leader.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        follower.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leader.setPower(power);
        follower.setPower(power);
    }

    public void stop() {
        setOpenLoop(0.0);
    }


    public double getTargetRpm() {
        return targetRpm;
    }

    public double getVelocityRpm() {
        return (leader.getVelocity() / Constants.Shooter.TICKS_PER_REV_OUTPUT) * 60.0;
    }

    public double getLeaderVelocityRpm() {
        return (leader.getVelocity() / Constants.Shooter.TICKS_PER_REV_OUTPUT) * 60.0;
    }

    public double getFollowerVelocityRpm() {
        return (follower.getVelocity() / Constants.Shooter.TICKS_PER_REV_OUTPUT) * 60.0;
    }

    public double getLastAppliedPower() {
        return leader.getPower();
    }

    public boolean atTargetRpm() {
        return atTargetRpm;
    }

    public PIDFCoefficients getPIDF() {
        return leader.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {
        boolean withinTolerance = targetRpm > 0.0 &&
                Math.abs(targetRpm - getVelocityRpm()) <= ShooterPidTuning.RPM_TOLERANCE;

        if (withinTolerance) {
            if (!wasWithinTolerance) {
                timer.reset();
            }
            atTargetRpm = timer.seconds() >= 0.1;
        } else {
            atTargetRpm = false;
        }
        wasWithinTolerance = withinTolerance;

        if (telemetry != null) {
            telemetry.addData("Shooter Target", targetRpm);
            telemetry.addData("Shooter Actual", getVelocityRpm());
            telemetry.addData("At Target", atTargetRpm);
        }
    }
}