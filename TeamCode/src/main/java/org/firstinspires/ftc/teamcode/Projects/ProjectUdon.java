// Project Udon

package org.firstinspires.ftc.teamcode.Projects;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class ProjectUdon extends Project{
    //Setup motors
    public DcMotor frontright = null;
    public DcMotor frontleft = null;
    public DcMotor backright = null;
    public DcMotor backleft = null;
//    public DistanceSensor distancesensor = null;
    public DcMotor roller = null;
    public DigitalChannel rollerSwitch = null;
    public DcMotor lift = null;

    @Override
    public void init(HardwareMap ahwMap) {

        //save reference to Hardware map    AINT NO WAY
        hwMap = ahwMap;

        //Define and Initialize Motors
        frontright = hwMap.dcMotor.get("frontright"); //port c3
        frontleft = hwMap.dcMotor.get("frontleft"); //port e0
        backright = hwMap.dcMotor.get("backright"); //port c0
        backleft = hwMap.dcMotor.get("backleft"); //port e3
        roller = hwMap.dcMotor.get("roller");
        lift = hwMap.dcMotor.get("lift");

        rollerSwitch = hwMap.digitalChannel.get("rollerSwitch");

        //Setup Motor directions and Encoder settings
        frontright.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        roller.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);

        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //2M Distance Sensor
//        distancesensor = hwMap.get(DistanceSensor.class, "distancesensor");

        Stop();
    }
    public void Stop(){
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        roller.setPower(0);
        lift.setPower(0);
    }
}

