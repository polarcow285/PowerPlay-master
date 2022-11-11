package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Projects.ProjectUdon;

@Autonomous(name="Test Auto")
public class TestAuto extends LinearOpMode {
    ColorSensor color;
    public ProjectUdon robot = new ProjectUdon();


    @Override
    public void runOpMode() {
        color = hardwareMap.get(ColorSensor.class, "color");

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Alpha", color.alpha());
            //telemetry.addData("Alpha", ColorSensorV3.);
            if (color instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) color).getDistance(DistanceUnit.CM));
            }
            telemetry.update();
        }

    }

}
