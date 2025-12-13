package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import java.util.function.DoubleSupplier;

public class LockHeadingCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final DoubleSupplier strafeX;
    private final DoubleSupplier strafeY;
    private final DoubleSupplier targetHeadingSupplier;
    private final PIDController headingController;
    private final double toleranceDeg;
    private double targetHeadingDeg;

    public LockHeadingCommand(final DriveSubsystem drive,
                              final DoubleSupplier strafeX,
                              final DoubleSupplier strafeY,
                              final DoubleSupplier targetHeadingSupplier,
                              final double toleranceDeg) {
        this.drive = drive;
        this.strafeX = strafeX;
        this.strafeY = strafeY;
        this.targetHeadingSupplier = targetHeadingSupplier;
        this.toleranceDeg = toleranceDeg;
        headingController = new PIDController(
                Constants.Drive.TURN_kP,
                Constants.Drive.TURN_kI,
                Constants.Drive.TURN_kD
        );
        headingController.setTolerance(toleranceDeg);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        headingController.reset();

    }

    @Override
    public void execute() {
        final double currentHeading = drive.getHeadingDegrees();
        targetHeadingDeg = targetHeadingSupplier.getAsDouble();
        headingController.setSetPoint(targetHeadingDeg);
        double rotation = headingController.calculate(currentHeading);
        rotation = clamp(rotation, -Constants.Drive.TURN_MAX_OUTPUT, Constants.Drive.TURN_MAX_OUTPUT);
        drive.drive(strafeX.getAsDouble(), strafeY.getAsDouble(), rotation);
    }

    @Override
    public void end(final boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return headingController.atSetPoint();
    }

    private double clamp(final double value, final double min, final double max) {
        if (value < min) {
            return min;
        }
        if (value > max) {
            return max;
        }
        return value;
    }
}

