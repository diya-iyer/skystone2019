package org.firstinspires.ftc.teamcode;
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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name="A: Mecanum Thunderbots RedSquareOneSkystone Autonomous", group="Thunderbots")

public class RedSquareOneSkystoneAutonomousPark extends MacThunderbotsSquareAutonomous {

    private NormalizedColorSensor colorSensor = null;
    private NormalizedRGBA colors = null;

    @Override
    public void runOpMode() {


         /* Initialize the drive system variables.
=======
        /*
         * Initialize the drive system variables.
>>>>>>> 6235cb25df2c16e128749ce502524b9a1b67d0a3
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        initSkystoneCamera();
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "Color_Back");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Init done");    //
        telemetry.update();

        robot.leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("SssssPath0", "Starting at %7d :%7d",
                robot.leftDrive1.getCurrentPosition(),
                robot.rightDrive1.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // activate the skystone target
        targetsSkyStone.activate();

        //sleep(300);

        // set the motor directions
        robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);


        // move forward for 2 seconds
        encoderDrive(0.1, -2500, -2500, 2.0);
        double powerMultiplier = 0.05;
        int count = 0;
        boolean stonefound = false;
        // until the skystone is detected
        // may need to change logic to stop somewhere
        while (count < 7) {

            // detect the skystone
            String capstone = detectSksytoneImage();
            // is it a skystpne?
            if ("Stone Target".equalsIgnoreCase(capstone)) {
                telemetry.addData("SKYSTONE VIEW", capstone);
                telemetry.addData("SKYSTONE STATUS", "FOUND!!!!");
                telemetry.update();
                // stop there and need to go out of the loop to pick the stone
                stonefound = true;
                break;
            } else {
                telemetry.addData("SKYSTONE VIEW", capstone);
                telemetry.addData("SKYSTONE STATUS", "NOT FOUND!!!!");
                telemetry.update();
                count++;
                // move right
                robot.leftDrive1.setPower(-powerMultiplier);
                robot.rightDrive1.setPower(powerMultiplier);
                robot.leftDrive2.setPower(powerMultiplier);
                robot.rightDrive2.setPower(-powerMultiplier);

                // wait briefly
                sleep(500); //500

                // Stop all motion;
                robot.leftDrive1.setPower(0);
                robot.rightDrive1.setPower(0);
                robot.leftDrive2.setPower(0);
                robot.rightDrive2.setPower(0);
                sleep(1000); //1000

            }
            // reset the power
            // this is the variable to play with to get the right speed for detection of the camera
            powerMultiplier = 0.08;

        }
        targetsSkyStone.deactivate();


        robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

        encoderDrive(0.1, -2500, -2500, 1.8);
        deliverbrick();

    }

    /*  telemetry.addData("Status", "Side_Arm");
      double sideArmposition = this.robot.sideArm.MAX_POSITION -.5;
      robot.rightClaw.setPosition(sideArmposition);
*/
    public void deliverbrick() {

        telemetry.addData("sideArm pos", String.valueOf(robot.sideArm.getPosition()));
        telemetry.update();
        // sleep(5000);
        robot.sideArm.setPosition(0.5);

        sleep(1000);
        // arm down
        double sideArmposition1 = this.robot.sideArm.MAX_POSITION - 1.0;
        robot.sideArm.setPosition(sideArmposition1);
        sleep(1000);


        double powerMultiplier = 0.5;
        //
        robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

        encoderDrive(DRIVE_SPEED, 64, 64, 0.9);


 /*           robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);

            encoderDrive(DRIVE_SPEED, 64, 64, 1.5);*/

        robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

        // loop until detect blue
        while (true) {

            robot.leftDrive1.setPower(powerMultiplier);
            robot.rightDrive1.setPower(powerMultiplier);
            robot.leftDrive2.setPower(powerMultiplier);
            robot.rightDrive2.setPower(powerMultiplier);

            colors = colorSensor.getNormalizedColors();
            if (colors.red > colors.green && colors.red > colors.blue) {
                robot.leftDrive1.setPower(0);
                robot.rightDrive1.setPower(0);
                robot.leftDrive2.setPower(0);
                robot.rightDrive2.setPower(0);

                sleep(1000);
                break;
            }
        }

        // move a little forward
        encoderDrive(DRIVE_SPEED, 64, 64, 0.1);

        sleep(2000);

        telemetry.addData("Status", "side_arm is here");
        telemetry.update();
        sideArmposition1 = this.robot.sideArm.MAX_POSITION;
        robot.sideArm.setPosition(sideArmposition1);
        sleep(1000);
        telemetry.addData("Status", "done");
        telemetry.update();
        sleep(1000);

       /* robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);

        encoderDrive(DRIVE_SPEED, 64, 64, 0.5); */

        }

    }





