package org.firstinspires.ftc.teamcode.hardware;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {

    //TODO ----------------------------INTAKE ACTUATORS--------------------------------

    public ServoImplEx intake=null;
    public DcMotorEx intakeRoller=null;

    //TODO -----------------------------OUTAKE ACTUATORS--------------------------------

    public CRServo turret1 =null;
    public CRServo turret2 =null;
    public ServoImplEx hood=null;
    public ServoImplEx endgame=null;
    public DcMotorEx shooter=null;
    public DcMotorEx turretEncoder=null;

    //TODO -----------------------------FEEDER ACTUATORS--------------------------------

    public DcMotorEx lowerFeeder=null;
    public DcMotorEx upperFeeder=null;

    //TODO -----------------------------SENSORS--------------------------------

    public RevColorSensorV3 c1 = null, c2 = null;

    public float[] hsvValues;
    public DigitalChannel intakeBeam=null;
    public DigitalChannel feederBeam=null;
    public DigitalChannel outtakeBeam=null;

    //TODO --------------------------------------------------------------------

    public static boolean outtakeBeamStore,intakeBeamStore,feederBeamState;

    // Static instance to be used across all instances
    private static RobotHardware instance;
    public boolean enabled;
    private HardwareMap hardwareMap;  // Linking to hardware map with robot hardware.

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;

        //TODO============================= MAPPING INTAKE ACTUATORS ===============================

        intakeRoller=hardwareMap.get(DcMotorEx.class,"roller");
        intake=hardwareMap.get(ServoImplEx.class,"intake");

        //TODO============================= MAPPING OUTAKE ACTUATORS ===============================

        turret1=hardwareMap.get(CRServo.class ,"turret1");
        turret2=hardwareMap.get(CRServo.class ,"turret2");
        hood=hardwareMap.get(ServoImplEx.class,"hood");
        shooter=hardwareMap.get(DcMotorEx.class,"shooter");
        turretEncoder=hardwareMap.get(DcMotorEx.class,"upperFeeder");

        //TODO============================= MAPPING FEEDER ACTUATORS ===============================

        lowerFeeder = hardwareMap.get(DcMotorEx.class, "lowerFeeder");
        upperFeeder = hardwareMap.get(DcMotorEx.class, "upperFeeder");

        //TODO============================= MAPPING SENSORS ===============================

        c1 = hardwareMap.get(RevColorSensorV3.class, "c1");
        c2 = hardwareMap.get(RevColorSensorV3.class, "c2");
//        csFeed=hardwareMap.get(RevColorSensorV3.class,"feed");
        intakeBeam=hardwareMap.get(DigitalChannel.class,"ib");
        feederBeam=hardwareMap.get(DigitalChannel.class,"fb");
        outtakeBeam=hardwareMap.get(DigitalChannel.class,"ob");

////        csFeed.setGain(50);
        intakeBeam.setMode(DigitalChannel.Mode.INPUT);
        feederBeam.setMode(DigitalChannel.Mode.INPUT);
        outtakeBeam.setMode(DigitalChannel.Mode.INPUT);

        //TODO============================= OTHER INITIALIZATION ===============================

        endgame =hardwareMap.get(ServoImplEx.class,"endgame");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        upperFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        lowerFeeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperFeeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public float[]  rgbToHsv(float rNorm, float gNorm, float bNorm) {
        float[] hsv = new float[3];

        float max = Math.max(rNorm, Math.max(gNorm, bNorm));
        float min = Math.min(rNorm, Math.min(gNorm, bNorm));
        float delta = max - min;
        // Value
        hsv[2] = max;

        // Saturation
        hsv[1] = max == 0 ? 0 : delta / max;

        // Hue
        if (delta == 0) {
            hsv[0] = 0;
        } else {
            if (max == rNorm) {
                hsv[0] = (60 * ((gNorm - bNorm) / delta) + 360) % 360;
            } else if (max == gNorm) {
                hsv[0] = (60 * ((bNorm - rNorm) / delta) + 120) % 360;
            } else if (max == bNorm) {
                hsv[0] = (60 * ((rNorm - gNorm) / delta) + 240) % 360;
            }
        }

        return hsv;
    }

    public String getColor(RevColorSensorV3 color) {
        String colorS = "";
        hsvValues = rgbToHsv(color.getNormalizedColors().red, color.getNormalizedColors().green, color.getNormalizedColors().blue);
        if (hsvValues[1] >= 0.65 && hsvValues[1] <= 0.75) {
            colorS = "GREEN";
        } else if (hsvValues[0] >= 180 && hsvValues[0] <= 230) {
            colorS = "PURPLE";
        }
        return colorS;
    }
    public void resetEncoder() {
        turretEncoder.setMode(STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

}