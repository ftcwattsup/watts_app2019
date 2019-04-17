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

    public int rotTicks = 50;
    public double defaultPower = 0.9;
    public double minPower = 0.2;

    public double matRevolution = 100;

    Collector(DcMotor _rot, DcMotor _extend, DcMotor _maturique) {
        rot = _rot;
        extender = _extend;
        mat = _maturique;

        mat.setDirection(DcMotorSimple.Direction.REVERSE);
        rot.setDirection(DcMotorSimple.Direction.FORWARD);
        extender.setDirection(DcMotorSimple.Direction.REVERSE);

        rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        matRevolution = mat.getMotorType().getTicksPerRev();
    }

    public void afterInitStart() {
        double power = defaultPower;
        rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rot.setTargetPosition(0);
        rot.setPower(power);

        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Rotation
     */

    public void addTicksGamepad(double y) {
        if(Math.abs(y) < 0.001) return;
        addTicks((int)(y * rotTicks));
    }

    public void addTicks(int ticks) {
        addTicksWithPower(ticks, defaultPower);
    }

    public void addTicksWithPower(int ticks, double power) {
        rot.setPower(power);
        rot.setTargetPosition(rot.getCurrentPosition() + ticks);
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

    public void stopRotation() {
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
    public void collect(double power) {
        if(Math.abs(power) < 0.01)  stopMaturique();
        else    startMaturique(power);
    }

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

        int pos = newpos;
        int dst = Math.abs(mat.getCurrentPosition() - newpos);

        newpos -= period;
        if(Math.abs(mat.getCurrentPosition() - newpos) < dst) {
            dst = Math.abs(mat.getCurrentPosition() - newpos);
            pos = newpos;
        }

        newpos += 2 * period;
        if(Math.abs(mat.getCurrentPosition() - newpos) < dst) {
            dst = Math.abs(mat.getCurrentPosition() - newpos);
            pos = newpos;
        }

        mat.setTargetPosition(pos);
        mat.setPower(1.0);
    }

    public void showTelemetry() {
        telemetry.addData("rot current", rot.getCurrentPosition());
        telemetry.addData("rot target", rot.getTargetPosition());
        telemetry.update();
    }

    public void setTelemetry(Telemetry _t) { telemetry = _t; }
    public void setOpmode(LinearOpMode _o) { opmode = _o; }
}
