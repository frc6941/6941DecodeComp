package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class CloseShootCommand extends SequentialCommandGroup {

    public CloseShootCommand(final ShooterSubsystem shooter, final FeederSubsystem feeder, Trigger trigger) {
        super(
                new InstantCommand(() ->
                {
                    shooter.setOpenLoop(1);
                    feeder.setIntakeOpenLoop(Constants.Feeder.DEFAULT_INTAKE_POWER);
                }),
                new RepeatCommand(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(trigger::get),
                                new InstantCommand(() -> feeder.setIndexOpenLoop(Constants.Feeder.DEFAULT_INDEX_POWER)),
                                new WaitUntilCommand(() -> !trigger.get()),
                                new InstantCommand(() -> feeder.setIndexOpenLoop(0))
                        )).perpetually()
        );
        addRequirements(feeder, shooter);
    }
}