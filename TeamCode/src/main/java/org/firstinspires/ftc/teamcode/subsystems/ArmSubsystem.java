package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem implements Subsystem {
    Servo leftIntake, rightIntake;
    DcMotorEx leftArmMotor, rightArmMotor;
    Gamepad driverGameapd;
    HardwareMap hardwareMap;
    boolean toggleIntake;
    boolean previousTriggerValue;
    boolean currentTriggerValue;

    public ArmSubsystem(Gamepad driverGamepad, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.driverGameapd = driverGamepad;

        leftIntake = hardwareMap.servo.get("leftIntake"); // HUB 1 Port 1
        rightIntake = hardwareMap.servo.get("rightIntake"); // HUB 1 Port 0

        leftArmMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftArmMotor");
        rightArmMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightArmMotor");
        leftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        toggleIntake = false;
        previousTriggerValue = false;
        currentTriggerValue = false;
    }

    public void init() {

    }

    public void reset() {

    }

    public void update() {
        if(driverGameapd.right_trigger > 0) {
            currentTriggerValue = true;
        } else {
            currentTriggerValue = false;
        }

        if(currentTriggerValue != previousTriggerValue) {
            toggleIntake = !toggleIntake;
        }


        if(toggleIntake) {
            leftIntake.setPosition(90);
            rightIntake.setPosition(90);
        } else  {
            leftIntake.setPosition(0);
            rightIntake.setPosition(0);
        }

        if(driverGameapd.left_trigger > 0) {
            leftArmMotor.setPower(-0.5);
            rightArmMotor.setPower(-0.5);

        } else if(driverGameapd.left_bumper) {
            leftArmMotor.setPower(0.75);
            rightArmMotor.setPower(0.75);
        } else {
            leftArmMotor.setPower(0);
            rightArmMotor.setPower(-0);

        }

        previousTriggerValue = currentTriggerValue;
    }

    public void stop() {
        leftArmMotor.setPower(0);
        rightArmMotor.setPower(0);
    }
}
