package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class CloseShootCommand extends SequentialCommandGroup {


    public CloseShootCommand(final ShooterSubsystem shooter, final FeederSubsystem feeder, Trigger trigger) {
        super(
                new InstantCommand(() -> {
                    shooter.setOpenLoop(1);
                }, shooter),

                new RepeatCommand(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(trigger::get),
                                new InstantCommand(() -> {
                                    shooter.openLatch();
                                    feeder.setIntakeOpenLoop(0.8);
                                    feeder.setOuttakeOpenLoop(0.7);
                                }),
                                new WaitUntilCommand(() -> !trigger.get()),
                                new InstantCommand(() -> {
                                    shooter.closeLatch();
                                    feeder.setIntakeOpenLoop(0);
                                    feeder.setOuttakeOpenLoop(0);
                                })
                        )).perpetually()
        );
        addRequirements(feeder);
    }
}