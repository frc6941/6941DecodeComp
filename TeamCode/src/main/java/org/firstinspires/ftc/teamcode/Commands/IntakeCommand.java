package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;

public class IntakeCommand extends CommandBase {

    private static final double DEFAULT_POWER = 1;

    private final FeederSubsystem feeder;
    private final double power;

    public IntakeCommand(final FeederSubsystem feeder) {
        this(feeder, DEFAULT_POWER);
    }

    public IntakeCommand(final FeederSubsystem feeder, final double power) {
        this.feeder = feeder;
        this.power = power;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.setIntakeOpenLoop(power);
        feeder.setOuttakeOpenLoop(0.15);
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

