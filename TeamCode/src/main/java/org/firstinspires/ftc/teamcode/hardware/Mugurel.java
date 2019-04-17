package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mugurel {
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public LinearOpMode opmode;

    public Runner runner;
    public Collector collector;
    public Lift lift;

    public Mugurel() { ; }

    public Mugurel(HardwareMap hm) {
        hardwareMap = hm;
        runner = new Runner(
                hm.get(DcMotor.class, Config.leftFront),
                hm.get(DcMotor.class, Config.rightFront),
                hm.get(DcMotor.class, Config.leftBack),
                hm.get(DcMotor.class, Config.rightBack)
        );
        collector = new Collector(
                hm.get(DcMotor.class, Config.rot),
                hm.get(DcMotor.class, Config.extend),
                hm.get(DcMotor.class, Config.maturique)
        );
        lift = new Lift(hm.get(DcMotor.class, Config.lift));
    }

    public void setTelemetry(Telemetry _t) {
        telemetry = _t;
        runner.setTelemetry(_t);
        collector.setTelemetry(_t);
        lift.setTelemetry(_t);
    }

    public void setOpmode(LinearOpMode _o) {
        opmode = _o;
        runner.setOpmode(_o);
        collector.setOpmode(_o);
        lift.setOpmode(_o);
    }

    public void afterStartInit() {
        runner.afterStartInit();
        collector.afterInitStart();
        lift.afterStartInit();
    }
}




