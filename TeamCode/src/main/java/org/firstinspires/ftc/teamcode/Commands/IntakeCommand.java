package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;

import java.util.function.BooleanSupplier;

public class IntakeCommand extends CommandBase {

    private final FeederSubsystem feeder;
    private BooleanSupplier breamBreak;
    private double intakePower, indexPower;

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
        feeder.setIntakeOpenLoop(intakePower);
        feeder.setIndexOpenLoop(indexPower);
    }

    @Override
    public void execute() {
        if (breamBreak.getAsBoolean()) {
            feeder.setIndexOpenLoop(0.0);
        }
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

