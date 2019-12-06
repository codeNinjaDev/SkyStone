package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class MoveArmCommand implements Command {
    ArmSubsystem arm;
    int position;
    double timeout;
    public MoveArmCommand(ArmSubsystem arm, int position, double timeout) {
        this.arm = arm;
        this.position = position;
        this.timeout = timeout;
    }

    public void init() {
        reset();
        arm.leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void reset() {

    }
    public void update(Telemetry tl) {
        arm.leftArmMotor.setPower(0.4);
        arm.rightArmMotor.setPower(0.4);
    }

    public boolean isFinished() {
        return (Math.abs(arm.leftArmMotor.getCurrentPosition()) > Math.abs(position)) || (Math.abs(arm.rightArmMotor.getCurrentPosition()) > Math.abs(position));
    }

    public void finish() {
        arm.leftArmMotor.setPower(0);
        arm.rightArmMotor.setPower(0);
        arm.reset();
    }
}
