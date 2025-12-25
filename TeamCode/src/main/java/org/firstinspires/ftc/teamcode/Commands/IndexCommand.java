package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;

public class IndexCommand extends CommandBase {

    private final FeederSubsystem feeder;
    private double intakePower, indexPower;

    public IndexCommand(final FeederSubsystem feeder) {
        this(feeder, Constants.Feeder.DEFAULT_INTAKE_POWER, Constants.Feeder.DEFAULT_INTAKE_POWER);
    }

    public IndexCommand(final FeederSubsystem feeder, final double intakePower, final double indexPower) {
        this.feeder = feeder;
        this.intakePower = intakePower;
        this.indexPower = indexPower;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.setIntakeOpenLoop(intakePower);
        feeder.setIndexOpenLoop(indexPower);
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

