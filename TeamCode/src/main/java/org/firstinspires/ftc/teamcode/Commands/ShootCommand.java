package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup {

    private static final double POWER = 0.8;
    private static final long LATCH_LEAD_TIME_MS = 100;

    public ShootCommand(final ShooterSubsystem shooter, final FeederSubsystem feeder) {
        super(
                new InstantCommand(() -> shooter.openLatch(), shooter),
                new InstantCommand(() -> shooter.setOpenLoop(POWER), shooter),
                new WaitCommand(LATCH_LEAD_TIME_MS),
                new InstantCommand(() -> {
                    feeder.setIntakeOpenLoop(POWER);
                    feeder.setOuttakeOpenLoop(POWER);
                }, feeder)
        );
    }
}


