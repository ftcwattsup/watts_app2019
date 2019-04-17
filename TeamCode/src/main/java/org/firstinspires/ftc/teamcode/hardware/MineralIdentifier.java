package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

public class MineralIdentifier {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public LinearOpMode opmode;

    public enum IdentifierType {ALL, LEFT_MID, MID_RIGHT}

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public static final String VUFORIA_KEY = Config.vuforiaKey;
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public static final double cameraX = 1280;
    public static final double cameraY = 720;
    public double middleX = cameraX / 2.0;
    public double TOP = 250;
    public double HEIGHT = 75;

    public int last = -1;
    public IdentifierType type = IdentifierType.ALL;

   public MineralIdentifier(HardwareMap _hm) {
        hardwareMap = _hm;
    }

    public void setType(IdentifierType _type) {
        type = _type;
    }
    public void setMid(double x) { middleX = x; }
    public void setTop(double x) { TOP = x; }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void init() {
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.addData("Vuforia intialized!", " ");
        start();
    }

    public void start() {
        tfod.activate();
    }

    public void stop() {
        tfod.shutdown();
    }

    public boolean valid(Recognition r) {
        double ypos = ((r.getTop() + r.getBottom()) / 2.0);
        if (ypos < TOP) return false;
        if (getHeight(r) < HEIGHT)   return false;
        return true;
    }

    public List<Recognition> validRecognitions(List<Recognition> rec) {
        ArrayList<Recognition> list = new ArrayList<Recognition>();
        for (Recognition r : rec)
            if (valid(r))
                list.add(r);
        return list;
    }

    public double getHeight(Recognition rec)
    {
        return ((rec.getBottom() - rec.getTop()));
    }

    public double getPosition(Recognition rec) {
        return ((rec.getLeft() + rec.getRight()) / 2.0);
    }

    public int getGoldMineral() {
        /**
         * -1 = invalid
         * 0 = left
         * 1 = middle
         * 2 = right
         */
        //List<Recognition> updatedRecognitions = tfod.getRecognitions();
        //if(updatedRecognitions == null) return last;
        List<Recognition> recognitions = validRecognitions(tfod.getRecognitions());

        telemetry.addData("Size", recognitions.size());

        if (type == IdentifierType.ALL) {
            int allcnt = recognitions.size(), goldcnt = 0;
            for (Recognition rec : recognitions)
                if (rec.getLabel().equals(LABEL_GOLD_MINERAL)) goldcnt++;

            if (allcnt != 3 && goldcnt != 1) return -1;

            double goldx = 0, silverx1 = 0, silverx2 = 0;
            for (Recognition rec : recognitions) {
                if (rec.getLabel().equals(LABEL_GOLD_MINERAL))
                    goldx = getPosition(rec);
                else if (rec.getLabel().equals(LABEL_SILVER_MINERAL)) {
                    if (silverx1 == 0) silverx1 = getPosition(rec);
                    else silverx2 = getPosition(rec);
                }
            }

            if (silverx1 > silverx2) {
                double aux = silverx1;
                silverx1 = silverx2;
                silverx2 = aux;
            }

            if (goldx < silverx1) return 0;
            if (goldx < silverx2) return 1;
            return 2;
        } else if (type == IdentifierType.LEFT_MID) {
            int goldcnt = 0;
            for (Recognition rec : recognitions)
                if (rec.getLabel().equals(LABEL_GOLD_MINERAL)) goldcnt++;
            if (goldcnt == 0) return 2;
            if (goldcnt > 1) return -1;
            for (Recognition rec : recognitions)
                if (rec.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    double pos = getPosition(rec);
                    if (pos > middleX) return 1;
                    else return 0;
                }
            return -1;
        } else if (type == IdentifierType.MID_RIGHT) {
            //telemetry.setAutoClear(false);
            int goldcnt = 0;
            for (Recognition rec : recognitions)
                if (rec.getLabel().equals(LABEL_GOLD_MINERAL))
                {
                    goldcnt++;
                    //telemetry.addData("Gold height", getHeight(rec));

                }
            if (goldcnt == 0) return 0;
            if (goldcnt > 1) return -1;
            for (Recognition rec : recognitions)
                if (rec.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    double pos = getPosition(rec);
                    if (pos > middleX) return 2;
                    else return 1;
                }
            return -1;
        }
        return -1;
    }

    public int findGold() {
        //telemetry.setAutoClear(false);
        //start();
        //opmode.sleep(1000);
        int count = 10;
        int[] fq = new int[3];
        for (int i = 0; i < count; i++) {
            int where = getGoldMineral();
            telemetry.addData(new Integer(i).toString(), where);
            telemetry.update();
            if (where != -1)
                fq[where]++;
            last = where;
        }
        telemetry.addData("fq 0", fq[0]);
        telemetry.addData("fq 1", fq[1]);
        telemetry.addData("fq 2", fq[2]);
        telemetry.update();
        int mx = -1, id = -1;
        for (int i = 0; i < 3; i++)
            if (fq[i] > mx) {
                mx = fq[i];
                id = i;
            }
        stop();
        return id;
    }

    public void setTelemetry(Telemetry _t) { telemetry = _t; }
    public void setOpmode(LinearOpMode _o) { opmode = _o; }
}
