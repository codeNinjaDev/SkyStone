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
        arm.leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update(Telemetry tl) {
        if(position > 0) {
            arm.leftArmMotor.setPower(0.3);
            arm.rightArmMotor.setPower(0.3);
        } else {
            arm.leftArmMotor.setPower(-0.2);
            arm.rightArmMotor.setPower(-0.2);
        }
    }

    public boolean isFinished() {
        return (Math.abs(arm.leftArmMotor.getCurrentPosition() - position) < 50) || (Math.abs(arm.rightArmMotor.getCurrentPosition() - position) < 50);
    }

    public void finish() {
        arm.leftArmMotor.setPower(0);
        arm.rightArmMotor.setPower(0);
        arm.reset();
    }
}
