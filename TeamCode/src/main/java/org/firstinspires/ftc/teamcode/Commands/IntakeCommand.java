package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;

import java.util.function.BooleanSupplier;

public class IntakeCommand extends CommandBase {

    private final FeederSubsystem feeder;
    private BooleanSupplier breamBreak;
    private double intakePower, indexPower;
    private static final long INDEX_FEED_PULSE_MS = 260L;

    private boolean previousBeamBlocked;
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
        previousBeamBlocked = breamBreak.getAsBoolean();
        indexFeedUntilNanos = 0L;

        feeder.setIntakeOpenLoop(intakePower);
        feeder.setIndexOpenLoop(0.0);
    }

    @Override
    public void execute() {
        final boolean beamBlocked = breamBreak.getAsBoolean();
        final long nowNanos = System.nanoTime();

        if (beamBlocked && !previousBeamBlocked) {
            indexFeedUntilNanos = nowNanos + INDEX_FEED_PULSE_MS * 1_000_000L;
        }

        final boolean shouldFeedIndexer = nowNanos < indexFeedUntilNanos;

        feeder.setIntakeOpenLoop(intakePower);
        feeder.setIndexOpenLoop(shouldFeedIndexer ? indexPower : 0.0);

        previousBeamBlocked = beamBlocked;
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
