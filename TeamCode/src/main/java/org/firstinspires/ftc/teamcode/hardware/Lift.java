package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    public Telemetry telemetry;
    public LinearOpMode opmode;

    public DcMotor motor;
    public final static int tickInterval = 6215;

    public Lift(DcMotor _motor) {
        motor = _motor;
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void afterStartInit() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move(double speed) {
        if(Math.abs(speed) < 0.1)
        {
            if(motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)  return;
            motor.setPower(speed);
            return;
        }
        if(motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        motor.setPower(speed);
    }

    public void goToPositionNoWait(int ticks, double power)
    {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(ticks);
        motor.setPower(power);
    }

    public void goToPosition(int ticks, double power) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(ticks);

        while (motor.isBusy()) {
            if (!opmode.opModeIsActive()) {
                motor.setPower(0);
                return;
            }

            motor.setPower(power);
        }
        motor.setPower(0.0);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void land() {
        goToPosition(tickInterval, 1.0);
    }
    public void hook() { goToPosition(-tickInterval, 1.0); }

    public void stay() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(0);
        motor.setPower(1.0);
    }

    public void setTelemetry(Telemetry _t) { telemetry = _t; }
    public void setOpmode(LinearOpMode _o) { opmode = _o; }
}
