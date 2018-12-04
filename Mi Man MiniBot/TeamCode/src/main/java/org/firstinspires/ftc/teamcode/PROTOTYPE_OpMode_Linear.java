/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="PROTOTYPE OpMode", group="Iterative Opmode")
//@Disabled
public class PROTOTYPE_OpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor strafeDrive = null;
    private DcMotor liftMotor = null;
    private DcMotor armMotor = null;
    private Servo armLeftServo = null;
    private Servo armRightServo = null;
    private Servo armMainServo = null;
    private ColorSensor middleBar = null;
    private  ColorSensor bottomBar = null;
    private TouchSensor liftTouchS= null;

    //Fields for setting power
    private double MOTOR_MAX = 1.0;
    private double MOTOR_OFF = 0.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing ...");

        leftDrive   = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive  = hardwareMap.get(DcMotor.class, "right_drive");
        strafeDrive = hardwareMap.get(DcMotor.class, "strafe_drive");
        liftMotor   = hardwareMap.get(DcMotor.class, "lift_motor");
        armMotor    = hardwareMap.get(DcMotor.class, "arm_motor");
        armRightServo = hardwareMap.get(Servo.class, "arm_right_servo");
        armLeftServo = hardwareMap.get(Servo.class, "arm_left_servo");
        armMainServo = hardwareMap.get(Servo.class, "arm_main_servo");
//        middleBar   = hardwareMap.get(ColorSensor.class, "middle");
//        bottomBar   = hardwareMap.get(ColorSensor.class, "bottom");
        liftTouchS  = hardwareMap.get(TouchSensor.class, "lift_touch");


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMainServo.scaleRange( -0.5, 1.0);
        armMainServo.setPosition(0.0);
        armLeftServo.setPosition(0);
        armRightServo.setPosition(0);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //region Wheels

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double horizontalPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive =  gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;

            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            horizontalPower = Range.clip( drive + strafe, -1.0,1.0);

            // Send calculated power to wheels

            // Normal Speed
            leftDrive.setPower(leftPower * 0.4);
            rightDrive.setPower(rightPower * 0.4);
            strafeDrive.setPower(horizontalPower * 0.4);

            // Turbo Speed
            if (gamepad1.left_stick_button){
                leftDrive.setPower(leftPower * 0.9);
                rightDrive.setPower(rightPower * 0.9);
                strafeDrive.setPower(horizontalPower * 0.9);
            }

            //endregion

            //region liftMotor

            if (liftTouchS.isPressed()) {
                liftMotor.setPower(MOTOR_OFF);
            }
            else {
                if (gamepad1.dpad_up) {
                    liftMotor.setPower(MOTOR_MAX);
                } else if (gamepad1.dpad_down) {
                    liftMotor.setPower(-MOTOR_MAX);
                } else {
                    liftMotor.setPower(MOTOR_OFF);
                }
            }

            //endregion

            //region armMotor
//
//        if(gamepad1.dpad_up){
//            armMotor.setPower(MOTOR_MAX);
//        }
//        else if(gamepad1.dpad_down){
//            armMotor.setPower(-MOTOR_MAX);
//        }
//        else {
//            armMotor.setPower(MOTOR_OFF);
//        }

            //endregion

            //region gripServoMotors

            double leftServoPosition = armLeftServo.getPosition();
            double rightServoPosition = armRightServo.getPosition();

            if(gamepad1.x){
                leftServoPosition += 0.1;
                rightServoPosition -= 0.1;
            }
            else if(gamepad1.b){
                leftServoPosition -= 0.1;
                rightServoPosition += 0.1;
            }

            armRightServo.setPosition(rightServoPosition);
            armLeftServo.setPosition(leftServoPosition);

            //endregion

            //region mainServoMotor
            double servoSpeed = gamepad1.right_stick_y;

            double mainServoPosition = armMainServo.getPosition();
            armMainServo.setPosition(servoContinuousMotion(servoSpeed, mainServoPosition));

            //endregion

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left_drive (%.2f), right_drive (%.2f), horizontal_drive (%.2f)", leftPower, rightPower, horizontalPower);
            telemetry.addData("Servos", "left_grip (%.2f), right_grip (%.2f), main_arm (%.4f)", leftServoPosition, rightServoPosition, mainServoPosition);
            //telemetry.addData("cageMotor", "Position: " + cageMotor.getCurrentPosition());
        }
    }

    private double servoContinuousMotion (double servoSpeed, double servoMovingPosition) {
            if (servoSpeed > 0.05){
                servoMovingPosition = servoSpeed * 0.006;
            } else if (-0.05 > servoSpeed){
                servoMovingPosition = servoSpeed * 0.006;
            }
            return servoMovingPosition;
    }
}
