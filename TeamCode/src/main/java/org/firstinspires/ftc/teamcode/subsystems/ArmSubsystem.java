package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libs.ButtonReader;
import org.firstinspires.ftc.teamcode.libs.GamepadKeys;
import org.firstinspires.ftc.teamcode.libs.SuperGamepad;
import org.firstinspires.ftc.teamcode.libs.ToggleButtonReader;
import org.firstinspires.ftc.teamcode.libs.TriggerReader;

public class ArmSubsystem implements Subsystem {
    Servo leftIntake, rightIntake, zoneServo;
    DcMotorEx leftArmMotor, rightArmMotor;
    SuperGamepad driverGameapd;
    HardwareMap hardwareMap;

    private ToggleButtonReader intakeToggle;
    private TriggerReader armDownButton;
    private ButtonReader armUpButton;
    public ArmSubsystem(SuperGamepad driverGamepad, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.driverGameapd = driverGamepad;

        leftIntake = hardwareMap.servo.get("leftIntake"); // HUB 1 Port 1
        rightIntake = hardwareMap.servo.get("rightIntake"); // HUB 1 Port 0

        leftArmMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftArmMotor");
        rightArmMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightArmMotor");

        rightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeToggle = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        armDownButton = new TriggerReader(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER);
        armUpButton = new ButtonReader(driverGamepad, GamepadKeys.Button.LEFT_BUMPER);
    }

    public void init() {

    }

    public void reset() {

    }

    public void update() {
        intakeToggle.readValue();
        armDownButton.readValue();
        armUpButton.readValue();


        if(intakeToggle.getState()) {
            leftIntake.setPosition(.44);
            rightIntake.setPosition(.85);
        } else {
            leftIntake.setPosition(.7);
            rightIntake.setPosition(.58);
        }

        if(armDownButton.isDown()) {
            leftArmMotor.setPower(-0.25);
            rightArmMotor.setPower(-0.25);

        } else if(armUpButton.isDown()) {
            leftArmMotor.setPower(0.4);
            rightArmMotor.setPower(0.4);
        } else {
            leftArmMotor.setPower(0);
            rightArmMotor.setPower(-0);

        }

    }

    public void stop() {
        leftArmMotor.setPower(0);
        rightArmMotor.setPower(0);
    }
}
