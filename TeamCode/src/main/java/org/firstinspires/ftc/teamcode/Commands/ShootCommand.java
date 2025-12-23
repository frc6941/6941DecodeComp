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
                    shooter.openLatch();
                    shooter.setVelocityClosedLoopEnabled(true);
                    shooter.setTargetRpm(ShooterPidTuning.TARGET_RPM);
                }, shooter),
                new RepeatCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    feeder.setIntakeOpenLoop(POWER);
                                    feeder.setOuttakeOpenLoop(0.5);
                                }, feeder),
                                new WaitUntilCommand(shooter::atTargetRpm),
                                new InstantCommand(() -> {
                                    feeder.setIntakeOpenLoop(POWER);
                                    feeder.setOuttakeOpenLoop(POWER);
                                }, feeder),
                                new WaitCommand(125)
                        )
                )
//                        .andThen(
//                        new InstantCommand(() -> {
//                            feeder.setIntakeOpenLoop(0);
//                            feeder.setOuttakeOpenLoop(0);
//                        }, feeder)
//                )
        );
    }
}