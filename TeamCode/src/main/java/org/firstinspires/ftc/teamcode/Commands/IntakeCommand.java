package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;

import java.util.function.BooleanSupplier;

public class IntakeCommand extends CommandBase {

    private final FeederSubsystem feeder;
    private BooleanSupplier breamBreak;
    private double intakePower, indexPower;
    private static final long INDEX_FEED_PULSE_MS = 180L;
    private static final double INDEX_FEED_PULSE_POWER = 0.35;

    private boolean beamBlockedLatched;
    private long indexFeedUntilNanos;

    public IntakeCommand(final FeederSubsystem feeder, BooleanSupplier breamBreak) {
        this(feeder, breamBreak, Constants.Feeder.DEFAULT_INTAKE_POWER, Constants.Feeder.DEFAULT_INDEX_POWER);
    }

    public IntakeCommand(final FeederSubsystem feeder, BooleanSupplier breamBreak, final double intakePower, final double indexPower) {
        this.feeder = feeder;
        this.intakePower = intakePower;
        this.indexPower = indexPower;
        this.breamBreak = breamBreak;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        beamBlockedLatched = false;
        indexFeedUntilNanos = 0L;

        feeder.setIntakeOpenLoop(intakePower);
        feeder.setIndexOpenLoop(0.0);
    }

    @Override
    public void execute() {
        final boolean beamBlocked = breamBreak.getAsBoolean();
        final long nowNanos = System.nanoTime();

        if (!beamBlocked) {
            beamBlockedLatched = false;
            indexFeedUntilNanos = 0L;
            feeder.setIntakeOpenLoop(intakePower);
            feeder.setIndexOpenLoop(indexPower);
            return;
        }

        if (beamBlocked && !beamBlockedLatched) {
            indexFeedUntilNanos = nowNanos + INDEX_FEED_PULSE_MS * 1_000_000L;
            beamBlockedLatched = true;
        }

        final boolean shouldFeedIndexer = nowNanos < indexFeedUntilNanos;
        final double indexPulsePower = getIndexPulsePower();

        feeder.setIntakeOpenLoop(intakePower);
        feeder.setIndexOpenLoop(shouldFeedIndexer ? indexPulsePower : 0.0);
    }

    private double getIndexPulsePower() {
        if (Math.abs(indexPower) < 1e-6) {
            return INDEX_FEED_PULSE_POWER;
        }

        return Math.copySign(
                Math.min(Math.abs(indexPower), INDEX_FEED_PULSE_POWER),
                indexPower
        );
    }


    @Override
    public void end(final boolean interrupted) {
        feeder.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
