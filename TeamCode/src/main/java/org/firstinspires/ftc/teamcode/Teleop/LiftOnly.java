package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Projects.ProjectUdon;

@TeleOp(name="LiftOnly", group="Mecanum")
public class LiftOnly extends LinearOpMode {
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
            }
            catch (RobotCoreException e) {

            }

            // lift code


            //int high = 4100; //TEMP, 4100 is for medium poles
            int medium = 3800;
            int low = (int) (medium * 0.574);


            telemetry.addData("Lift Position", robot.lift.getCurrentPosition());
            telemetry.update();

            if (robot.lift.getCurrentPosition()>low && gamepad2.x) { // Arm UP
                liftTarget = low;
                liftSpeed = -0.5;
                liftCurrentDirection = "up";

                robot.lift.setPower(liftSpeed);
                robot.lift.setTargetPosition(liftTarget);
            }else if (robot.lift.getCurrentPosition()<low && gamepad2.x) { // Arm DOWN
                liftTarget = low;
                liftSpeed = 0.5;
                liftCurrentDirection = "down";

                robot.lift.setPower(liftSpeed);
                robot.lift.setTargetPosition(liftTarget);
            }else if (robot.lift.getCurrentPosition()>medium && gamepad2.b) { // Arm UP
                liftTarget = medium;
                liftSpeed = -0.5;
                liftCurrentDirection = "up";

                robot.lift.setPower(liftSpeed);
                robot.lift.setTargetPosition(liftTarget);
            }else if (robot.lift.getCurrentPosition()<medium && gamepad2.b) { // Arm DOWN
                liftTarget = medium;
                liftSpeed = 0.5;
                liftCurrentDirection = "down";

                robot.lift.setPower(liftSpeed);
                robot.lift.setTargetPosition(liftTarget);
            }
//            else if (robot.lift.getCurrentPosition()>high && gamepad2.y) { // Arm UP
//                liftTarget = high;
//                liftSpeed = -0.5;
//                liftCurrentDirection = "up";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }else if (robot.lift.getCurrentPosition()<high && gamepad2.y) { // Arm DOWN
//                liftTarget = high;
//                liftSpeed = 0.5;
//                liftCurrentDirection = "down";
//
//                robot.lift.setPower(liftSpeed);
//                robot.lift.setTargetPosition(liftTarget);
//            }
            else if (gamepad2.a){ // Arm DOWN
                liftTarget = 0;
                liftSpeed = -0.5;  // From my research, negative is ignore, so I don't understand why this *seemed* to work
                liftCurrentDirection = "down";

                robot.lift.setPower(liftSpeed);
                robot.lift.setTargetPosition(liftTarget);
            }

            //Remove Power from the Arm Motor if motor is close to 0 position, arm should drop 200
//            if ( liftCurrentDirection == "down" && ( robot.lift.getTargetPosition() < 5 ) ){
//                liftSpeed = 0;
//                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }

            /** END ARM LIFT **/


            //idle();

            // Arm Lift Telemetry
            /*if(robot.lift.isBusy() ){
                telemetry.addData("Lift Position", robot.lift.getCurrentPosition());
                telemetry.update();
            }*/
        }

    }

    }
