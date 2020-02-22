package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.libs.ButtonReader;
import org.firstinspires.ftc.teamcode.libs.GamepadKeys;
import org.firstinspires.ftc.teamcode.libs.SuperGamepad;
import org.firstinspires.ftc.teamcode.libs.ToggleButtonReader;
import org.firstinspires.ftc.teamcode.libs.TriggerReader;

public class ArmSubsystem implements Subsystem {
    Servo leftIntake, rightIntake, zoneServo;
    public DcMotorEx leftArmMotor, rightArmMotor;
    SuperGamepad driverGameapd;
    HardwareMap hardwareMap;

    private ToggleButtonReader intakeToggle;
    private TriggerReader armDownButton;
    private ButtonReader armUpButton;
    private ToggleButtonReader foldIntakeButton;
    private ToggleButtonReader slowMode;

    //311
    public ArmSubsystem(SuperGamepad driverGamepad, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.driverGameapd = driverGamepad;

        leftIntake = hardwareMap.servo.get("leftIntake"); // HUB 1 Port 1
        rightIntake = hardwareMap.servo.get("rightIntake"); // HUB 1 Port 0

        leftArmMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftArmMotor");
        rightArmMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightArmMotor");

        leftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeToggle = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        armDownButton = new TriggerReader(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER);
        armUpButton = new ButtonReader(driverGamepad, GamepadKeys.Button.LEFT_BUMPER);
        foldIntakeButton = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.X);
        slowMode = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.A);
    }

    public void init() {
        reset();
    }

    public void reset() {
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        intakeToggle.readValue();
        armDownButton.readValue();
        armUpButton.readValue();
        foldIntakeButton.readValue();
        slowMode.readValue();

        if(!foldIntakeButton.getState()) {
            if (intakeToggle.getState()) {
                openClaw();
            } else {
                closeClaw();
            }
        } else {
            stowClaws();
        }

        double multiplier = 1;
        if(slowMode.getState()) {
            leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            multiplier *= 0.5;
        } else {
            leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if(armDownButton.isDown()) {
            leftArmMotor.setPower(-0.45);
            rightArmMotor.setPower(-0.45);
        } else if(armUpButton.isDown()) {
            leftArmMotor.setPower(0.65 * multiplier);
            rightArmMotor.setPower(0.65 * multiplier);
        } else {
            leftArmMotor.setPower(0);
            rightArmMotor.setPower(0);
        }
    }

    public void stop() {
        leftArmMotor.setPower(0);
        rightArmMotor.setPower(0);
    }

    public void closeClaw() {
        //left low is closed
        leftIntake.setPosition(.33);
        // right high is open
        rightIntake.setPosition(0.915);
    }

    public void openClaw() {
        leftIntake.setPosition(.5);
        rightIntake.setPosition(.72);
    }

    public void stowClaws() {
        leftIntake.setPosition(1);
        rightIntake.setPosition(.23);
    }
}
