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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class MacHardwarePushbot
{
    /* Public OpMode members. */
    //Macenum has 4 wheels each with a motor
    public DcMotor  leftDrive1   = null;
    public DcMotor  rightDrive1  = null;
    public DcMotor  leftDrive2   = null;
    public DcMotor  rightDrive2  = null;

    //public DcMotor  leftArm     = null;
    public DcMotor  elbow       = null;
    public DcMotor  CenterRightArm    = null;
    public DcMotor  CenterLeftArm     = null;
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;
    public Servo    capstone    = null;
    public Servo    Wrist       = null;
    public NormalizedColorSensor colorFront = null;
    public NormalizedColorSensor colorBack = null;

    //public Servo    foundationarm = null;
    public DcMotor  tapemeasurer = null;

    public Servo basepull1 = null;
    public Servo basepull2 = null;

    public Servo sideArm = null;   //SHASHANK USE FOR DEMO

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.25 ;
    public static final double ARM_DOWN_POWER  = -0.25 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public MacHardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize 4 Motors
        leftDrive1  = hwMap.get(DcMotor.class, "left_drive1");
        rightDrive1 = hwMap.get(DcMotor.class, "right_drive1");
        leftDrive2  = hwMap.get(DcMotor.class, "left_drive2");
        rightDrive2 = hwMap.get(DcMotor.class, "right_drive2");

        CenterRightArm = hwMap.get(DcMotor.class, "CenterRightArm");
        CenterLeftArm = hwMap.get(DcMotor.class, "CenterLeftArm");
        rightClaw = hwMap.get(Servo.class, "right_claw");
        elbow     = hwMap.get(DcMotor.class,"elbow");
        basepull1 = hwMap.get(Servo.class, "base_pull1");
        basepull2 = hwMap.get(Servo.class, "base_pull2");
        sideArm = hwMap.get(Servo.class, "side_arm");
        //  foundationarm = hwMap.get(Servo.class, "foundation_arm");
        tapemeasurer = hwMap.get(DcMotor.class, "tape_measurer");
        capstone = hwMap.get(Servo.class, "capstone");
        Wrist = hwMap.get(Servo.class, "Wrist");
        colorFront = hwMap.get(NormalizedColorSensor.class, "Color_Front");
        colorBack = hwMap.get(NormalizedColorSensor.class, "Color_Back");


        leftDrive1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftDrive2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDrive1.setPower(0);
        rightDrive1.setPower(0);
        leftDrive2.setPower(0);
        rightDrive2.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        /* No Arms included yet in Macenum robot
        leftArm.setPower(0);
        rightArm.setPower(0);
        leftArm.se  tMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        // Define and initialize ALL installed servos.
        //@Thunderbots commented on Oct 11 as servos not included yet
       //leftClaw  = hwMap.get(Servo.class, "left_hand");
       // rightClaw = hwMap.get(Servo.class, "right_hand");
       // leftClaw.setPosition(MID_SERVO);
       // rightClaw.setPosition(MID_SERVO);

        basepull1.setDirection(Servo.Direction.FORWARD);
        basepull2.setDirection(Servo.Direction.FORWARD);
        Wrist.setDirection(Servo.Direction.FORWARD);
        sideArm.setDirection(Servo.Direction.FORWARD);  //SHASHANK USE FOR DEMO

    }
 }

