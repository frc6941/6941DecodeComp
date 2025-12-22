package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.tuning.ShooterPidTuning;

public class ShootCommand extends SequentialCommandGroup {

    private static final double POWER = 1;
    private static final long LATCH_LEAD_TIME_MS = 1000;

    public ShootCommand(final ShooterSubsystem shooter, final FeederSubsystem feeder) {
        super(
                new InstantCommand(() -> {
                    shooter.openLatch();
                    shooter.setVelocityClosedLoopEnabled(true);
                    shooter.setTargetRpm(ShooterPidTuning.TARGET_RPM);
                }, shooter),
                new WaitUntilCommand(shooter::atTargetRpm),
                new InstantCommand(() -> {
                    feeder.setIntakeOpenLoop(POWER);
                    feeder.setOuttakeOpenLoop(POWER);
                }, feeder),
                new WaitCommand(175),
                new InstantCommand(shooter::closeLatch, shooter),
                new WaitUntilCommand(shooter::atTargetRpm),
                new InstantCommand(() -> {
                    feeder.setIntakeOpenLoop(POWER);
                    feeder.setOuttakeOpenLoop(POWER);
                }, feeder),
                new WaitCommand(175),
                new InstantCommand(shooter::closeLatch, shooter),
                new WaitUntilCommand(shooter::atTargetRpm),
                new InstantCommand(() -> {
                    feeder.setIntakeOpenLoop(POWER);
                    feeder.setOuttakeOpenLoop(POWER);
                }, feeder),
                new WaitCommand(175),
                new InstantCommand(shooter::closeLatch, shooter),
                new WaitUntilCommand(shooter::atTargetRpm),
                new InstantCommand(() -> {
                    feeder.setIntakeOpenLoop(POWER);
                    feeder.setOuttakeOpenLoop(POWER);
                }, feeder),
                new WaitCommand(175),
                new InstantCommand(shooter::closeLatch, shooter),
                new WaitUntilCommand(shooter::atTargetRpm),
                new InstantCommand(() -> {
                    feeder.setIntakeOpenLoop(POWER);
                    feeder.setOuttakeOpenLoop(POWER);
                }, feeder),
                new WaitCommand(175),
                new InstantCommand(shooter::closeLatch, shooter)
        );
    }
}


