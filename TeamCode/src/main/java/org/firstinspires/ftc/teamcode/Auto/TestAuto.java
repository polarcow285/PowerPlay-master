// TestAuto


package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.Projects.ProjectUdon;

@Autonomous
public class TestAuto extends LinearOpMode {
    public ProjectUdon robot = new ProjectUdon();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
//        robot.frontleft.setPower(1);
//        robot.frontright.setPower(-1);
//        sleep(3000);
//        robot.frontleft.setPower(0);
//        robot.frontright.setPower(0);

        // generic DistanceSensor methods.
        telemetry.addData("range", String.format("%.01f mm", robot.distancesensor.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", robot.distancesensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", robot.distancesensor.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", robot.distancesensor.getDistance(DistanceUnit.INCH)));

        telemetry.update();

    }
}