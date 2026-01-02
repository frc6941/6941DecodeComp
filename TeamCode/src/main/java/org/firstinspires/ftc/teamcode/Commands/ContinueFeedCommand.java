package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;

public class ContinueFeedCommand extends SequentialCommandGroup {

    public ContinueFeedCommand(final FeederSubsystem feeder) {
        super(

                new InstantCommand(() ->
                {
                    feeder.setIntakeOpenLoop(Constants.Feeder.SHOOT_INTAKE_POWER);
                }),
                new RepeatCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> feeder.setIndexOpenLoop(Constants.Feeder.DEFAULT_INDEX_POWER)),
                                new WaitCommand(120),
                                new InstantCommand(() -> feeder.setIndexOpenLoop(0))))
        );
        addRequirements(feeder);
    }
}