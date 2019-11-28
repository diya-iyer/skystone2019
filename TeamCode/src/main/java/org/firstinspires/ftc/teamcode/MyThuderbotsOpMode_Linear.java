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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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
 * We took out @Disabled
 * name="Basic: Linear OpMode"  changed to name="Basic: Thunderbots OpMode"
 */

@TeleOp(name="Basic: Thunderbots OpMode", group="Thunderbots")

public class MyThuderbotsOpMode_Linear extends ThunsderbotVuforiaSkyStoneNavigationWebcamOpMode {

    // Declare OpMode members.
    // Added the two DcMotors for the arm
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private Servo elbow = null;
    private Servo claw = null;
   // private Servo grabber = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftArm = hardwareMap.get(DcMotor.class, "left_arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_arm");
        elbow = hardwareMap.get(Servo.class,"elbow");
        claw = hardwareMap.get(Servo.class,"claw");
        //grabber = hardwareMap.get(Servo.class, "grabber");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);
        elbow.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);

        initSkystoneCamera();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        targetsSkyStone.activate();

        //double tgtPower = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftForwardPower;
            double rightForwardPower;
            double leftBackwardPower;
            double rightBackwardPower;
            double leftArmPower;
            double rightArmPower;
            final double ELBOWINCREMENT   = 2.5;     // amount to slew servo each CYCLE_MS cycle
            final double CLAWINCREMENT    = 0.5 ;
            final double MAX_POS     =  3.0;     // Maximum rotational position
            final double MIN_POS     =  0.0;     // Minimum rotational position
            double  elbowposition = 0; // Starting position
            double clawposition = 0; // Starting position



            /*double grabberopenPower;
            double grabberclosePower; */

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double driveForward = -gamepad1.right_trigger;
            double driveBackward = gamepad1.left_trigger;
            double turn  =  gamepad1.right_stick_x;
            double uparm = gamepad2.left_stick_y;
            double downarm = gamepad2.left_stick_x;
            boolean upelbow = gamepad2.dpad_right;
            boolean downelbow = gamepad2.dpad_left;
            boolean clawopen = gamepad2.dpad_up;
            boolean clawclose = gamepad2.dpad_down;
            //double opengrabber = -gamepad2.right_stick_y;
           // double closegrabber = gamepad2.right_stick_x;

            leftForwardPower    = Range.clip(driveForward + turn, -1.0, 1.0) ;
            rightForwardPower   = Range.clip(driveForward - turn, -1.0, 1.0) ;
            rightBackwardPower = Range.clip(driveBackward + turn,-0.5, 1.0 );
            leftBackwardPower = Range.clip(driveBackward - turn, -0.5, 1.0);
            leftArmPower = Range.clip(uparm + downarm, -0.2, 0.5);
            rightArmPower = Range.clip(uparm - downarm, -0.2, 0.5);
            //grabberopenPower = Range.clip()



            if (upelbow) {

                elbowposition += ELBOWINCREMENT ;
                if (elbowposition >= MAX_POS ) {
                    elbowposition = MAX_POS;
                }
                elbow.setPosition(elbowposition);
            }
            if (downelbow){
                elbowposition -= ELBOWINCREMENT ;
                if (elbowposition <= MIN_POS ) {
                    elbowposition = MIN_POS;
                }
                elbow.setPosition(elbowposition);
                    
            }

            if (upelbow || downelbow) {
                telemetry.addData("Servo Position", "%5.2f", elbowposition);
                telemetry.update();
            }

            if (clawopen){

                clawposition += CLAWINCREMENT ;
                if (clawposition >= MAX_POS){
                    clawposition = MIN_POS;
                }

                claw.setPosition(clawposition);
                    

                

            }
            if (clawclose){

                clawposition -= CLAWINCREMENT ;
                if (clawposition <= MIN_POS) {
                    clawposition = MAX_POS;
                }
                claw.setPosition(clawposition);

            }


            /*telemetry.addData("Drive Key Pressed", "Forward:::" + driveForward + " ::: Backward ::: " + driveBackward);
            telemetry.update(); */

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            if (driveForward < 0) {
                leftDrive.setPower(leftForwardPower);
                rightDrive.setPower(rightForwardPower);
                // Show the elapsed game time and wheel power.
                /*telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftForwardPower, rightForwardPower);
                telemetry.update();*/
            } else if (driveForward == 0) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
            if (driveBackward > 0) {
                leftDrive.setPower(leftBackwardPower);
                rightDrive.setPower(rightBackwardPower);
                // Show the elapsed game time and wheel power.
                /*telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftBackwardPower, rightBackwardPower);
                telemetry.update();*/
            } else if (driveBackward == 0) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
                leftArm.setPower(leftArmPower);
                rightArm.setPower(rightArmPower);

            detectSksytoneImage();
            telemetry.update();
        }
        targetsSkyStone.deactivate();
    }

}
