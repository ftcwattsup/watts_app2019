package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Collector {

    public Telemetry telemetry;
    public LinearOpMode opmode;

    public DcMotor rot, extender, mat;

    public int rotTicks = 90;
    public double initialPower = 0.33;
    public double newratio = 1.0;
    public boolean lastTicksWithPower = false;

    public double matRevolution = 100;

    Collector(DcMotor _rot, DcMotor _extend, DcMotor _maturique) {
        rot = _rot;
        extender = _extend;
        mat = _maturique;

        mat.setDirection(DcMotorSimple.Direction.FORWARD);
        rot.setDirection(DcMotorSimple.Direction.REVERSE);
        extender.setDirection(DcMotorSimple.Direction.FORWARD);

        rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void afterInitStart() {
        double power = initialPower * newratio;
        rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rot.setTargetPosition(0);
        rot.setPower(power);

        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Rotation
     */
    public void addTicks(double y) {
        addTicks((int) (y * rotTicks));
    }

    public void addTicks(int ticks) {
        addTicksint(ticks);
        //rotLeft.setTargetPosition(rotLeft.getTargetPosition() + ticks);
        //rot.setTargetPosition(rot.getTargetPosition() + ticks);
    }

    public void addTicksint(int ticks) {
        if(Math.abs(ticks) < 10)  return;
        if(lastTicksWithPower)
        {
            rot.setPower(initialPower * newratio);
        }
        lastTicksWithPower = false;
        rot.setTargetPosition(rot.getTargetPosition() + ticks);
    }

    public void addTicksintWithPower(int ticks, double power) {
        lastTicksWithPower = true;
        rot.setPower(initialPower * power * newratio);
        rot.setTargetPosition(rot.getTargetPosition() + ticks);
        /*while(rotLeft.isBusy() && rot.isBusy())
        {
            if(!opmode.opModeIsActive())
            {
                rotLeft.setPower(0);
                rot.setPower(0);
                return;
            }
        }
        rotLeft.setPower(initialPower);
        rot.setPower(initialPower);*/
    }

    public void rotateTicks(int ticks) {
        rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rot.setTargetPosition(ticks);
        rot.setPower(1.0);
        //while(rotLeft.isBusy() && rot.isBusy()) {
        //    if(!opmode.opModeIsActive())    return;
        //}
    }

    public void rotate(double speed) {
        rot.setPower(speed);
    }

    public void stopRotation()
    {
        rot.setTargetPosition(rot.getCurrentPosition());
    }

    /**
     * Extension
     */
    public void extend(double speed) {
        extender.setPower(speed);
    }

    /**
     * Maturique
     */
    public void startMaturique(double power) {
        if(mat.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)    mat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mat.setPower(power);
    }

    public void stopMaturique() {
        mat.setPower(0);
        int ticks = mat.getCurrentPosition();
        mat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int period = (int)(matRevolution) / 4;
        int newpos = (int)(ticks / period) * period;
        mat.setTargetPosition(newpos);
        mat.setPower(1.0);
    }

    public void setTelemetry(Telemetry _t) { telemetry = _t; }
    public void setOpmode(LinearOpMode _o) { opmode = _o; }
}
