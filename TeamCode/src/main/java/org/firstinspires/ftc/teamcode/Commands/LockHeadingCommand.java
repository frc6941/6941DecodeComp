package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.tuning.LockHeadingTuning;

import java.util.function.DoubleSupplier;

public class LockHeadingCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final DoubleSupplier strafeX;
    private final DoubleSupplier strafeY;
    private final DoubleSupplier targetHeadingSupplier;
    private final PIDController headingController;
    private final double toleranceDeg;
    private final Telemetry telemetry;
    private double targetHeadingDeg;

    public LockHeadingCommand(final DriveSubsystem drive,
                              final DoubleSupplier strafeX,
                              final DoubleSupplier strafeY,
                              final DoubleSupplier targetHeadingSupplier,
                              final double toleranceDeg,
                              final Telemetry telemetry) {
        this.drive = drive;
        this.strafeX = strafeX;
        this.strafeY = strafeY;
        this.targetHeadingSupplier = targetHeadingSupplier;
        this.toleranceDeg = toleranceDeg;
        this.telemetry = telemetry;
        headingController = new PIDController(
                LockHeadingTuning.TURN_kP,
                LockHeadingTuning.TURN_kI,
                LockHeadingTuning.TURN_kD
        );
        headingController.setTolerance(toleranceDeg);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        headingController.reset();
        headingController.setSetPoint(0.0);
    }

    @Override
    public void execute() {
        // Dashboard 调参：每次循环都刷新 PID 系数（参考 GoToPoseCommand）
        headingController.setPID(
                LockHeadingTuning.TURN_kP,
                LockHeadingTuning.TURN_kI,
                LockHeadingTuning.TURN_kD
        );
        targetHeadingDeg = targetHeadingSupplier.getAsDouble();
        double currentHeading = drive.getHeadingDegrees();
        double error = shortestDelta360(targetHeadingDeg, currentHeading);
        double rotation = headingController.calculate(-error);
        rotation = clamp(rotation, -Math.abs(LockHeadingTuning.TURN_MAX_OUTPUT), Math.abs(LockHeadingTuning.TURN_MAX_OUTPUT));
        drive.drive(strafeX.getAsDouble(), strafeY.getAsDouble(), rotation);
    }

    @Override
    public void end(final boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return headingController.atSetPoint();
    }

    private double shortestDelta360(final double targetDeg, final double currentDeg) {
        double error = targetDeg - currentDeg;
        error = error % 360.0;
        if (error > 180.0) {
            error -= 360.0;
        } else if (error < -180.0) {
            error += 360.0;
        }
        return error;
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

