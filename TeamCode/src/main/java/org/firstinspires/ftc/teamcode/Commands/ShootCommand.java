package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.tuning.ShooterPidTuning;

public class ShootCommand extends SequentialCommandGroup {

    private static final double POWER = 1;

    public ShootCommand(final ShooterSubsystem shooter, final FeederSubsystem feeder) {
        super(
                new InstantCommand(() -> {
                    shooter.setVelocityClosedLoopEnabled(true);
                    shooter.setTargetRpm(ShooterPidTuning.TARGET_RPM);
                }, shooter),
                new RepeatCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    feeder.setIntakeOpenLoop(0.7);
                                    feeder.setIndexOpenLoop(0.5);
                                }, feeder),
                                new WaitCommand(5000),
                                new WaitUntilCommand(shooter::atTargetRpm),
                                new InstantCommand(() -> {
                                    feeder.setIntakeOpenLoop(POWER);
                                    feeder.setIndexOpenLoop(POWER);
                                }, feeder),
                                new WaitCommand(0)
                        )
                )
        );
    }
}