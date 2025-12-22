package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class PreShootCommand extends CommandBase {

    private static final double DEFAULT_POWER = 0.8;

    private final ShooterSubsystem shooter;
    private final double power;

    public PreShootCommand(final ShooterSubsystem shooter) {
        this(shooter, DEFAULT_POWER);
    }

    public PreShootCommand(final ShooterSubsystem shooter, final double power) {
        this.shooter = shooter;
        this.power = power;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setOpenLoop(power);
    }

    @Override
    public void end(final boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

