package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class FeederSubsystem extends SubsystemBase {

    private final Motor intakeRoller;
    private final Motor indexRoller;
    private final DcMotorEx intakeEx;
    private final DcMotorEx indexEx;

    public FeederSubsystem(final HardwareMap hardwareMap) {
        intakeRoller = buildRollerMotor(
                hardwareMap,
                Constants.Feeder.INTAKE_ROLLER_NAME,
                Constants.Feeder.INTAKE_INVERTED
        );
        indexRoller = buildRollerMotor(
                hardwareMap,
                Constants.Feeder.INDEX_ROLLER_NAME,
                Constants.Feeder.INDEX_INVERTED
        );

        DcMotorEx tmpIntake;
        DcMotorEx tmpIndex;
        try {
            tmpIntake = hardwareMap.get(DcMotorEx.class, Constants.Feeder.INTAKE_ROLLER_NAME);
        } catch (Exception ignored) {
            tmpIntake = null;
        }
        try {
            tmpIndex = hardwareMap.get(DcMotorEx.class, Constants.Feeder.INDEX_ROLLER_NAME);
        } catch (Exception ignored) {
            tmpIndex = null;
        }
        intakeEx = tmpIntake;
        indexEx = tmpIndex;
    }

    public void setIntakeOpenLoop(final double power) {
        intakeRoller.setRunMode(Motor.RunMode.RawPower);
        intakeRoller.set(clamp(power, -1.0, 1.0));
    }

    public void setIndexOpenLoop(final double power) {
        indexRoller.setRunMode(Motor.RunMode.RawPower);
        indexRoller.set(clamp(power, -1.0, 1.0));
    }

    public double getIntakeVelocityRpm() {
        if (intakeEx != null) {
            final double ticksPerSec = intakeEx.getVelocity();
            return (ticksPerSec * 60.0) / Constants.Shooter.TICKS_PER_REV_OUTPUT;
        }
        try {
            return intakeRoller.getCorrectedVelocity();
        } catch (Exception ignored) {
            return 0.0;
        }
    }

    public void setIntakeVelocityRpm(final double rpm) {
        intakeRoller.setRunMode(Motor.RunMode.VelocityControl);
        intakeRoller.set(Math.max(0.0, rpm));
    }

    public double getIndexVelocityRpm() {
        if (indexEx != null) {
            final double ticksPerSec = indexEx.getVelocity();
            return (ticksPerSec * 60.0) / Constants.Shooter.TICKS_PER_REV_OUTPUT;
        }
        try {
            return indexRoller.getCorrectedVelocity();
        } catch (Exception ignored) {
            return 0.0;
        }
    }

    public void setIndexVelocityRpm(final double rpm) {
        indexRoller.setRunMode(Motor.RunMode.VelocityControl);
        indexRoller.set(Math.max(0.0, rpm));
    }

    public void stop() {
        intakeRoller.set(0.0);
        indexRoller.set(0.0);
        intakeRoller.setRunMode(Motor.RunMode.RawPower);
        indexRoller.setRunMode(Motor.RunMode.RawPower);
    }

    private Motor buildRollerMotor(final HardwareMap hardwareMap,
                                   final String name,
                                   final boolean inverted) {
        final Motor motor = new Motor(hardwareMap, name, Constants.Feeder.ROLLER_GEARING);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setInverted(inverted);
        return motor;
    }

    private double clamp(final double value, final double min, final double max) {
        return Math.max(min, Math.min(max, value));
    }
}

