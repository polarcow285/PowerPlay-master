package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Projects.ProjectUdon;

@TeleOp(name="LiftNoLimit", group="Mecanum")
public class LiftNoLimit extends LinearOpMode {
    private ProjectUdon robot = new ProjectUdon();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.setAutoClear(true);
        telemetry.addData("Lift Position", robot.lift.getCurrentPosition());

        int liftTarget = 0;
        double liftSpeed = 0;
        String liftCurrentDirection = "up";

        waitForStart();

        while (opModeIsActive()) {

            try {
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);
                throw new RobotCoreException("I am Exception Alpha!");
            } catch (RobotCoreException e) {

            }

            // lift code
            if (gamepad2.right_bumper) {
                //liftSpeed = 1;
                robot.lift.setPower(0.5);
            }
            else if (gamepad2.left_bumper) {
                robot.lift.setPower(-0.5);
            }
            else {
                robot.lift.setPower(0);
            }

            if(gamepad2.back){
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }


            telemetry.addData("Lift Position", robot.lift.getCurrentPosition());
            telemetry.update();
        }
    }
}