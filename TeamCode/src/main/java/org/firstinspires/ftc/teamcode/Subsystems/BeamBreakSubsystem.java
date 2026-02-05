//package org.firstinspires.ftc.teamcode.Subsystems;

//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

//public class BeamBreakSubsystem extends SubsystemBase {

    //private final AnalogInput beamBreak;
    //private final double defaultToleranceVoltage;

    //public BeamBreakSubsystem(HardwareMap hardwareMap, String name, double defaultToleranceVoltage) {
        //beamBreak = hardwareMap.get(AnalogInput.class, name);
        //this.defaultToleranceVoltage = defaultToleranceVoltage;
    //}


    //public boolean isBeamBreakOn() {
        //return isBeamBreakOn(defaultToleranceVoltage);
    //}

    //public boolean isBeamBreakOn(double toleranceVoltage) {
       // return beamBreak.getVoltage() >= toleranceVoltage;
    //}

    //public double getVoltage() {
        //return beamBreak.getVoltage();
    //}
//}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Beam Break Debug", group = "Debug")
public class debug_beam_break extends LinearOpMode {

    private AnalogInput beamBreak;

    @Override
    public void runOpMode() {
        // 初始化光电门
        beamBreak = hardwareMap.get(AnalogInput.class, "shooterBB");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 读取电压值
            double voltage = beamBreak.getVoltage();

            telemetry.addData("Beam Break Voltage", "%.3f V", voltage);
            telemetry.addData("Beam Break Status", voltage >= 1.0 ? "BLOCKED" : "CLEAR");
            telemetry.addData("Expected Range", "0.0V - 3.3V");
            telemetry.update();

            sleep(100);
        }
    }
}
