package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Projects.ProjectUdon;

@TeleOp(name="DriveOnly", group="Mecanum")
public class DriveOnly extends LinearOpMode {
    private ProjectUdon robot = new ProjectUdon();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.setAutoClear(true);
        waitForStart();

        while (opModeIsActive()) {

            try {
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);
            }
            catch (RobotCoreException e) {

            }
            //Driving controls

            double y = 0; //back and forth
            double x = -gamepad1.right_stick_x * 1.1; //strafing
            double rx = gamepad1.left_stick_x; //turning

            //back and forth movement using triggers
            if (gamepad1.right_trigger > 0) {
                y = gamepad1.right_trigger;
                x = 0;
            } else if (gamepad1.left_trigger > 0) {
                y = -gamepad1.left_trigger;
                x = 0;
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            robot.frontleft.setPower(0.75 * frontLeftPower);
            robot.backleft.setPower(0.75 * backLeftPower);
            robot.frontright.setPower(0.75 * frontRightPower);
            robot.backright.setPower(0.75 * backRightPower);



            idle();

        }

    }

}
