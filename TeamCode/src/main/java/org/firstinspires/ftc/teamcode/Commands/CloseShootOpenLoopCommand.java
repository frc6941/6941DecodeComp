package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class CloseShootOpenLoopCommand extends SequentialCommandGroup {

    public CloseShootOpenLoopCommand(
            final ShooterSubsystem shooter,
            final FeederSubsystem feeder,
            Trigger trigger,
            double openLoop) {
        super(
                new InstantCommand(() ->
                {
                    shooter.setOpenLoop(openLoop);
                    feeder.setIntakeOpenLoop(Constants.Feeder.DEFAULT_INTAKE_POWER);
                }),
                new RepeatCommand(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> trigger.get()),
                                new InstantCommand(() -> feeder.setIndexOpenLoop(Constants.Feeder.DEFAULT_INDEX_POWER)),
                                new WaitCommand(20),
                                new WaitUntilCommand(() -> !trigger.get()),
                                new InstantCommand(() -> feeder.setIndexOpenLoop(0))
                        )).perpetually()
        );
        addRequirements(feeder, shooter);
    }
}