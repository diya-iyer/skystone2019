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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Basic: Mecanum Thunderbots BlueSquareOneSkystone Autonomous", group="Thunderbots")

public class BlueSquareOneSkystoneAutonomous extends MacThunderbotsSquareAutonomous {

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
        targetsSkyStone.activate();
        //initSkystoneCamera();
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
        sleep(300);
        detectSksytoneImage();
        targetsSkyStone.deactivate();
        deliverbrick();

        /* telemetry.addData("sideArm pos", String.valueOf(robot.sideArm.getPosition()));
        telemetry.update();
        // sleep(5000);
        robot.sideArm.setPosition(0.5);
        robot.basepull1.setPosition(0.5);

        robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

        encoderDrive(DRIVE_SPEED, 64, 64, 1.5); */
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
            robot.basepull1.setPosition(0.5);

            robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);

            encoderDrive(DRIVE_SPEED, 64, 64, 1);

            sleep (2000);
            String Image = detectSksytoneImage();
            int count = 0;
            while (Image==null || !Image.equals("Stone Target")) {

                telemetry.addData("Status", "Skystone Not Identified..."+Image);    //
                telemetry.update();
                sleep(15000);
                

                robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

                encoderDrive(DRIVE_SPEED, 64, 64, .2);


                sleep(2000);
                int retry=0;
                while (retry <3) {
                    Image = detectSksytoneImage();
                    sleep(2000);
                    retry++;
                }
                count ++;
                if (count > 20) break;

            }
            if ((Image!=null)&&Image.equals("Stone Target")) {
                telemetry.addData("Status", "Skystone Identified..."+Image);    //
                telemetry.update();
            }
            robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);

            encoderDrive(DRIVE_SPEED, 64, 64, 0.6);
           // telemetry.addData("Status", "side_arm is here");
          //  telemetry.update();
            double sideArmposition1 = this.robot.sideArm.MAX_POSITION-1.0;
            robot.sideArm.setPosition(sideArmposition1);
            sleep(1000);
          //  telemetry.addData("Status", "done");
            //telemetry.update();
            sleep (1000);

            double powerMultiplier = 0.5;

            robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

            encoderDrive(DRIVE_SPEED, 64, 64, 1.2);


            robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);

            encoderDrive(DRIVE_SPEED, 64, 64, 1.5);

            sleep (2000);

            telemetry.addData("Status", "side_arm is here");
            telemetry.update();
            sideArmposition1 = this.robot.sideArm.MAX_POSITION;
            robot.sideArm.setPosition(sideArmposition1);
            sleep(1000);
            telemetry.addData("Status", "done");
            telemetry.update();
            sleep (1000);

            robot.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightDrive2.setDirection(DcMotorSimple.Direction.FORWARD);

            encoderDrive(DRIVE_SPEED, 64, 64, 0.4);


        }



        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderDrive(DRIVE_SPEED,  -3,   -3, 2.0);  // S1: Forward 3 Inches with 2 Sec timeout
        //TO DO: Step 2 Move arm to collect skystone
        // encoderDrive(TURN_SPEED,   1, -1, 2.0);  // S3: Turn Right 12 Inches with 4 Sec timeout

        //String imageDetectedName=this.detectSksytoneImage();
        //telemetry.addData("Image", imageDetectedName);



    /* public void parkunderbridge() {

        double powerMultiplier = 0.5;


        robot.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);

        encoderDrive(DRIVE_SPEED, 64, 64, 1.5); */




        //sideways must be quadrupled due to strafing needing more rotations
        //robot.leftDrive1.setPower(-powerMultiplier);
        //robot.rightDrive1.setPower(powerMultiplier);
        //robot.leftDrive2.setPower(powerMultiplier);
        //robot.rightDrive2.setPower(-powerMultiplier);

    }
