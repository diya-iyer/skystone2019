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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="Basic: Mecanum Thunderbots NEW TeleOp1", group="Thunderbots")

public class MacThuderbotsOpMode_FlippedNEWLinear extends LinearOpMode {

    // Declare OpMode members.
    MacHardwarePushbot robot = new MacHardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    double leftForwardPower;
    double rightForwardPower;
    double leftBackwardPower;
    double rightBackwardPower;
    double ArmDownUp;
    final double CLAWINCREMENT = 0.5;
    final double BASEINCREMENT = 3;

    final double CAPSTONEINCREMENT = 0.2;
    //final double CAPSTONE = 0.5;
    final double CAPSTONE_DROP_POS = 1.0;
    final double CAPSTONE_START_POS = 0.5;

    final double BASEPULL = 0.7;
    double basepullposition = 0;
    double foundationposition = 0;
    double capstoneposition = 0;
    double MAX_POS = 3.0;     // Maximum rotational position
    double MIN_POS = 0.0;     // Minimum rotational position


    double powerMultiplier = 1.0;
    double ParkpowerMultiplier = .5;
    double MAX_POWER = 1.0;
    double POWER_INCREMENT = 0.2;

    double powerMultiplierArm = -0.8;

    // private Servo grabber = null;
    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Init Done");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //double tgtPower = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry

            driveMacChasis();
            pickUpBrick();
            powerChange();
            tapemeasurepark();
            capstonedrop();
            telemetry.update();
        }

    }

    public void driveMacChasis() {
        // POV Mode uses left stick to go forward, backward, and turn
        // - This uses basic math to combine motions and is easier to drive straight.

        double driveForward = gamepad1.left_stick_y;
        double driveBackward = gamepad1.left_stick_y;
        double turnRight = gamepad1.right_stick_x;
        double turnLeft = gamepad1.right_stick_x;
        double strafeRight = gamepad1.left_stick_x;
        double strafeLeft = gamepad1.left_stick_x;
        double powerMultiplier = 0.5;

        boolean driveStop = false;


        if ((gamepad1.left_stick_y == 0) && (gamepad1.right_stick_y == 0) && (gamepad1.left_stick_x == 0) && (gamepad1.right_stick_x == 0))
            driveStop = true;

        //Mecanum wheels work well with full power
        if (turnRight < 0) {
            telemetry.addData("Status", "Moving right");
            telemetry.update();

            robot.leftDrive1.setPower(powerMultiplier);
            robot.rightDrive1.setPower(-powerMultiplier);
            robot.leftDrive2.setPower(powerMultiplier);
            robot.rightDrive2.setPower(-powerMultiplier);

        } else if (turnLeft > 0) {

            telemetry.addData("Status", "Moving left");
            telemetry.update();

            robot.leftDrive1.setPower(-powerMultiplier);
            robot.rightDrive1.setPower(powerMultiplier);
            robot.leftDrive2.setPower(-powerMultiplier);
            robot.rightDrive2.setPower(powerMultiplier);

        } else if (driveForward > 0) {

            telemetry.addData("Status", "Moving forward");
            telemetry.update();

            robot.leftDrive1.setPower(powerMultiplier);
            robot.rightDrive1.setPower(powerMultiplier);
            robot.leftDrive2.setPower(powerMultiplier);
            robot.rightDrive2.setPower(powerMultiplier);
        } else if (driveBackward < 0) {

            telemetry.addData("Status", "Moving backward");
            telemetry.update();

            robot.leftDrive1.setPower(-powerMultiplier);
            robot.rightDrive1.setPower(-powerMultiplier);
            robot.leftDrive2.setPower(-powerMultiplier);
            robot.rightDrive2.setPower(-powerMultiplier);
        } else if (driveStop) {
            //telemetry.addData("Status", "Stopping");
            //telemetry.update();

            robot.leftDrive1.setPower(0);
            robot.rightDrive1.setPower(0);
            robot.leftDrive2.setPower(0);
            robot.rightDrive2.setPower(0);
        } else if (strafeRight < 0) {
            telemetry.addData("Status", "Moving Right");
            telemetry.update();

            robot.leftDrive1.setPower(powerMultiplier);
            robot.rightDrive1.setPower(-powerMultiplier);
            robot.leftDrive2.setPower(-powerMultiplier);
            robot.rightDrive2.setPower(powerMultiplier);

        } else if (strafeLeft > 0) {
            telemetry.addData("Status", "Moving Left");
            telemetry.update();

            robot.leftDrive1.setPower(-powerMultiplier);
            robot.rightDrive1.setPower(powerMultiplier);
            robot.leftDrive2.setPower(powerMultiplier);
            robot.rightDrive2.setPower(-powerMultiplier);
        }

        leftForwardPower = this.robot.leftDrive1.getPower();
        rightForwardPower = this.robot.rightDrive1.getPower();
        leftBackwardPower = this.robot.leftDrive1.getPower();
        rightBackwardPower = this.robot.rightDrive1.getPower();
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Inputs received", "Drive Forward: " + driveForward + " Drive Backward: " + driveBackward + " Drive Right: " + turnRight + " Drive Left: " + turnLeft);
        telemetry.addData("Motors Forward", "left (%.2f), right (%.2f)", leftForwardPower, rightForwardPower);
        telemetry.addData("Motors Backward", "left (%.2f), right (%.2f)", leftBackwardPower, rightBackwardPower);


    }

    public void pickUpBrick() {
        boolean drivePickDown = gamepad2.dpad_down;
        boolean drivePickUp = gamepad2.dpad_up;
        boolean clawopen = gamepad2.dpad_right;
        boolean clawclose = gamepad2.dpad_left;
        double clawposition = robot.rightClaw.getPosition();
        boolean upfoundationarm = gamepad2.y;
        boolean downfoundationarm = gamepad2.a;
        double elbowUpDown = gamepad2.right_stick_y;

        MAX_POS = this.robot.rightClaw.MAX_POSITION;
        MIN_POS = this.robot.rightClaw.MIN_POSITION;


        boolean armStop = false;

        if ((gamepad2.right_stick_y == 0))
            armStop = true;

        double powerMultiplier = 0.6;
        double powerMultiplierArm = 0.8;


        if (!drivePickDown && !drivePickUp) {

            robot.CenterRightArm.setPower(0);
            robot.CenterLeftArm.setPower(0);
        }

        if (drivePickUp) {
            robot.CenterRightArm.setPower(powerMultiplier);
            robot.CenterLeftArm.setPower(-powerMultiplier);
        } else if (drivePickDown) {
            robot.CenterRightArm.setPower(-powerMultiplier);
            robot.CenterLeftArm.setPower(powerMultiplier);
        } else if (clawopen) {
            telemetry.addData("Claw open", clawposition);
            if (clawposition <= MAX_POS) {
                clawposition += CLAWINCREMENT;
            }
            robot.rightClaw.setPosition(clawposition);
        } else if (clawclose) {
            telemetry.addData("Claw close", clawposition);
            if (clawposition >= MIN_POS) {
                clawposition -= CLAWINCREMENT;
            }
            robot.rightClaw.setPosition(clawposition);

        }
        if (armStop) {

            robot.elbow.setPower(0);

        }
        if (elbowUpDown < 0) {

            telemetry.addData("Status", "ElbowMovingUp");
            robot.elbow.setPower(powerMultiplierArm);

        } else if (elbowUpDown > 0) {
            telemetry.addData("Status", "ElbowMovingDown");
            robot.elbow.setPower(-powerMultiplierArm);

        }
        if (upfoundationarm) {
            telemetry.addData("Status", "FoundationArmUp");

            basepullposition = this.robot.basepull1.MIN_POSITION+CAPSTONEINCREMENT;
            robot.basepull1.setPosition(basepullposition);
            robot.basepull2.setPosition(basepullposition);

        } else if (downfoundationarm) {
            telemetry.addData("Status", "FoundationArmDown");

            basepullposition = this.robot.capstone.MAX_POSITION-CAPSTONEINCREMENT;


            robot.basepull1.setPosition(basepullposition);
            robot.basepull2.setPosition(basepullposition);


        }

        telemetry.addData("Arms & Claw", "left (%.2f), right (%.2f)", robot.CenterRightArm.getPower(), robot.CenterLeftArm.getPower(), robot.rightClaw.getPosition());
        telemetry.addData("Elbow", "left (%.2f)", robot.elbow.getPower());
        telemetry.addData("Base Pull 1 & 2 ", "left (%.2f) left (%.2f)", robot.basepull1.getPosition(),robot.basepull2.getPosition());



}

    public void powerChange() {

        boolean powerDown = gamepad1.dpad_down;
        boolean powerUp = gamepad1.dpad_up;


        if (powerMultiplier < MAX_POWER && powerUp) {
            powerMultiplier = powerMultiplier + POWER_INCREMENT;
        } else if (powerMultiplier > 0 && powerDown) {
            powerMultiplier = powerMultiplier + POWER_INCREMENT;
        }


        telemetry.addData("Power Multiplier", "left (%.2f)", powerMultiplier);


    }

    public void tapemeasurepark() {

        //boolean releasecapstone = gamepad2.start;
        boolean extendtape = gamepad2.x;
        boolean reducetape = gamepad2.b;
        if (!extendtape && !reducetape)
            robot.tapemeasurer.setPower(0);
        if (extendtape) {
            telemetry.addData("Status", "TapeOut");
            robot.tapemeasurer.setPower(ParkpowerMultiplier);
        } else if (reducetape) {
            telemetry.addData("Status", "TapeIn");
            robot.tapemeasurer.setPower(-ParkpowerMultiplier);
        }
    }

    public void capstonedrop() {

        boolean dropcapstone = gamepad2.start;
        boolean liftcapstone = gamepad2.back;

        telemetry.addData("Capstone - Current position", "left (%.2f)", robot.capstone.getPosition());
        if (dropcapstone)
            capstoneposition = this.robot.capstone.MIN_POSITION+CAPSTONEINCREMENT;

        else if (liftcapstone)
            capstoneposition = this.robot.capstone.MAX_POSITION-CAPSTONEINCREMENT;

        robot.capstone.setPosition(capstoneposition);
        telemetry.addData("Capstone - New position", "left (%.2f)", robot.capstone.getPosition());


    }
}

