package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem implements Subsystem {
    DcMotor leftIntake, rightIntake;
    Gamepad driverGameapd;
    HardwareMap hardwareMap;
    public IntakeSubsystem(HardwareMap hardwareMap, Gamepad driverGamepad) {
        this.hardwareMap = hardwareMap;
        this.driverGameapd = driverGamepad;

        leftIntake = hardwareMap.dcMotor.get("leftIntake"); // HUB 1 Port 1
        rightIntake = hardwareMap.dcMotor.get("rightIntake"); // HUB 1 Port 0
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void init() {

    }

    public void reset() {

    }

    public void update() {
        if(driverGameapd.right_trigger > 0) {
            leftIntake.setPower(1);
            rightIntake.setPower(1);
        } else if(driverGameapd.b) {
            leftIntake.setPower(-1);
            rightIntake.setPower(-1);
        } else {
            stop();
        }
    }

    public void stop() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }
}
