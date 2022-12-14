package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Projects.ProjectUdon;

@TeleOp(name="CONtrols", group="Mecanum")
public class CONtrols extends LinearOpMode {
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
        boolean rollerForwardToggle = false;
        boolean rollerBackwardToggle = false;

        waitForStart();

        while (opModeIsActive()) {

            try {
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);
                throw new RobotCoreException("I am Exception Alpha!");
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


            // Strafe Right at lower power
            if(frontLeftPower > 0 && backLeftPower < 0 && frontRightPower < 0 && backRightPower > 0){
                robot.frontleft.setPower(frontLeftPower*0.35);
                robot.backleft.setPower(backLeftPower*0.35);
                robot.frontright.setPower(frontRightPower*0.35);
                robot.backright.setPower(backRightPower*0.35);
            }
            // Strafe Left at lower power
            else if(frontLeftPower < 0 && backLeftPower > 0 && frontRightPower > 0 && backRightPower < 0){
                robot.frontleft.setPower(frontLeftPower*0.35);
                robot.backleft.setPower(backLeftPower*0.35);
                robot.frontright.setPower(frontRightPower*0.35);
                robot.backright.setPower(backRightPower*0.35);
            }
            //normal speed
            else{
                robot.frontleft.setPower(frontLeftPower*0.5);
                robot.backleft.setPower(backLeftPower*0.5);
                robot.frontright.setPower(frontRightPower*0.5);
                robot.backright.setPower(backRightPower*0.5);
            }

            // roller code
            if (gamepad2.right_bumper && !robot.rollerSwitch.getState()) {
                robot.roller.setPower(0.5);
            }
            else if (gamepad2.left_bumper) {
                robot.roller.setPower(-0.5);
            }
            else {
                robot.roller.setPower(0);
            }

            // lift code
            if (gamepad2.y && robot.lift.getCurrentPosition()<4100) {
                //liftSpeed = 1;
                robot.lift.setPower(0.5);
            }
            else if(gamepad2.a &&robot.lift.getCurrentPosition()>0){
                robot.lift.setPower(-0.5);
            }
            else{
                robot.lift.setPower(0);
            }



            telemetry.addData("Lift Position", robot.lift.getCurrentPosition());

            // lift code
            /*
            if (robot.lift.getCurrentPosition()>-2000 && gamepad2.x) { // Arm UP
                liftTarget = -2000;
                liftSpeed = 0.98;
                liftCurrentDirection = "up";

                robot.lift.setPower(liftSpeed);
                robot.lift.setTargetPosition(liftTarget);
            }else if (robot.lift.getCurrentPosition()<-2000 && gamepad2.x) { // Arm DOWN
                liftTarget = -2000;
                liftSpeed = -0.98;
                liftCurrentDirection = "down";

                robot.lift.setPower(liftSpeed);
                robot.lift.setTargetPosition(liftTarget);
            }else if (robot.lift.getCurrentPosition()>-3500 && gamepad2.b) { // Arm UP
                liftTarget = -3500;
                liftSpeed = 0.98;
                liftCurrentDirection = "up";

                robot.lift.setPower(liftSpeed);
                robot.lift.setTargetPosition(liftTarget);
            }else if (robot.lift.getCurrentPosition()<-3500 && gamepad2.b) { // Arm DOWN
                liftTarget = -3500;
                liftSpeed = -0.98;
                liftCurrentDirection = "down";

                robot.lift.setPower(liftSpeed);
                robot.lift.setTargetPosition(liftTarget);
            }else if (robot.lift.getCurrentPosition()>-4200 && gamepad2.y) { // Arm UP
                liftTarget = -4200;
                liftSpeed = 0.98;
                liftCurrentDirection = "up";

                robot.lift.setPower(liftSpeed);
                robot.lift.setTargetPosition(liftTarget);
            }else if (robot.lift.getCurrentPosition()<-4200 && gamepad2.y) { // Arm DOWN
                liftTarget = -4200;
                liftSpeed = -0.98;
                liftCurrentDirection = "down";

                robot.lift.setPower(liftSpeed);
                robot.lift.setTargetPosition(liftTarget);
            } else if (gamepad2.a){ // Arm DOWN
                liftTarget = 0;
                liftSpeed = -0.98;  // From my research, negative is ignore, so I don't understand why this *seemed* to work
                liftCurrentDirection = "down";

                robot.lift.setPower(liftSpeed);
                robot.lift.setTargetPosition(liftTarget);
            }

            // Remove Power from the Arm Motor if motor is close to 0 position, arm should drop
//            if ( liftCurrentDirection == "down" && ( lift.getTargetPosition() < 5 ) ){
//                liftSpeed = 0;
//                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }

            /** END ARM LIFT **/


            idle();




            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backRightPower", backRightPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.update();
        }

    }

    }
