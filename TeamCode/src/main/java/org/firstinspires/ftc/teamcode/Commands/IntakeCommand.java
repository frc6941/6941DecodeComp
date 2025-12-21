package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;

public class IntakeCommand extends CommandBase {

    private static final double DEFAULT_INTAKE_POWER = 1;
    private static final double DEFAULT_OUTTAKE_POWER =0.7;

    private final FeederSubsystem feeder;
    private double intakePower, outtakePower;

    public IntakeCommand(final FeederSubsystem feeder) {
        this(feeder, DEFAULT_INTAKE_POWER, DEFAULT_INTAKE_POWER);
    }

    public IntakeCommand(final FeederSubsystem feeder, final double intakePower, final double outtakePower) {
        this.feeder = feeder;
        this.intakePower = intakePower;
        this.outtakePower = outtakePower;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.setIntakeOpenLoop(intakePower);
        feeder.setOuttakeOpenLoop(outtakePower);

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

