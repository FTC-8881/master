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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@Autonomous(name="Final: Just Land", group="Final")

public class AutoLeftDrop extends LinearOpMode {
    
    /* Declare OpMode members. */
    MasterHardware          robot   = new MasterHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final int LIFT = -20000;
    static final double LIFT_SPEED = 1.0;
    static final double CPD = 1.7357;
    static final double CPI = 8.4317;
    static final double ROT_SPEED = 0.5;
    static final double STRAFE_SPEED = 0.7;
    static final double DRIVE_SPEED = 0.7;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Lift",  "Running at %7d",
                          robot.liftMtr.getCurrentPosition());
        telemetry.addData("FrontDrive",  "Starting at %7d :%7d",
                          robot.flDriveMtr.getCurrentPosition(),
                          robot.frDriveMtr.getCurrentPosition());
        telemetry.addData("RearDrive",  "Starting at %7d :%7d",
                          robot.rlDriveMtr.getCurrentPosition(),
                          robot.rrDriveMtr.getCurrentPosition());
        telemetry.addData("Heading", 
                    String.format("%3d",-robot.gyro.getIntegratedZValue()));
        telemetry.update();
        robot.kickServo.setPosition(robot.KICK_IN);
    // Wait for the game to start (driver presses PLAY)
        waitForStart();
    //LOWER ROBOT
        encoderLift(LIFT_SPEED,  LIFT, 10); 
    //GET ALL 4 WHEELS ON THE GROUND
        driveToInches(DRIVE_SPEED, 1.25, 5);
    //UNHOOK THE ROBOT
        strafeRightInches(STRAFE_SPEED, 2.5, 5);
    //MOVE AWAY FROM LANDER    
        driveToInches(DRIVE_SPEED, 7, 5);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderLift(double speed, int liftTarget, double timeoutS) {

         //Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.liftSet(liftTarget);
            runtime.reset();
            robot.liftRun(speed);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                   robot.liftMtr.isBusy()  &&
                   robot.digitalTopLow.getState()  && 
                   robot.digitalTopLow.getState()) {

                // Display it for the driver.
                telemetry.addData("LiftT",  "Running to %7d", liftTarget);
                telemetry.addData("LiftA",  "Running at %7d",
                                            robot.liftMtr.getCurrentPosition());
//                telemetry.addData("Heading", 
//                    String.format("%3d",-robot.gyro.getIntegratedZValue()));
                telemetry.update();
            }

            robot.liftStop();
            //sleep(1000);   // optional pause after each move
        }
    }

    public void driveToDegrees(double speed,
                             int degrees, 
                             double timeoutS) {
        int flActual = robot.flDriveMtr.getCurrentPosition();
        int frActual = robot.frDriveMtr.getCurrentPosition();
        int rlActual = robot.rlDriveMtr.getCurrentPosition();
        int rrActual = robot.rrDriveMtr.getCurrentPosition();
        
        int flTarget = (int)(flActual + CPD*degrees);
        int frTarget = (int)(frActual - CPD*degrees);
        int rlTarget = (int)(rlActual + CPD*degrees);
        int rrTarget = (int)(rrActual - CPD*degrees);
        
        
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            robot.flDriveMtr.setTargetPosition(flTarget);
            robot.frDriveMtr.setTargetPosition(frTarget);
            robot.rlDriveMtr.setTargetPosition(rlTarget);
            robot.rrDriveMtr.setTargetPosition(rrTarget);
            
            runtime.reset();            
            robot.driveRun(speed);



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.flDriveMtr.isBusy() && robot.frDriveMtr.isBusy() &&
                   robot.rlDriveMtr.isBusy() && robot.rrDriveMtr.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Front T",  "Running to %7d :%7d ", flTarget,  frTarget);
                telemetry.addData("Front A",  "Running at %7d :%7d",
                                            robot.flDriveMtr.getCurrentPosition(),
                                            robot.frDriveMtr.getCurrentPosition());
                telemetry.addData("Rear T",  "Running to %7d :%7d ", rlTarget,  rrTarget);
                telemetry.addData("Rear A",  "Running at %7d :%7d",
                                            robot.rlDriveMtr.getCurrentPosition(),
                                            robot.rrDriveMtr.getCurrentPosition());
                telemetry.addData("Heading", 
                    String.format("%3d",-robot.gyro.getIntegratedZValue()));
                telemetry.update();
            }

            robot.driveStop();

            //sleep(1000);   // optional pause after each move
        }
    }

    public void driveToInches(double speed,
                             double inches, 
                             double timeoutS) {
        int flActual = robot.flDriveMtr.getCurrentPosition();
        int frActual = robot.frDriveMtr.getCurrentPosition();
        int rlActual = robot.rlDriveMtr.getCurrentPosition();
        int rrActual = robot.rrDriveMtr.getCurrentPosition();
        
        int flTarget = (int)(flActual + CPI*inches);
        int frTarget = (int)(frActual + CPI*inches);
        int rlTarget = (int)(rlActual + CPI*inches);
        int rrTarget = (int)(rrActual + CPI*inches);
        
        
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            robot.flDriveMtr.setTargetPosition(flTarget);
            robot.frDriveMtr.setTargetPosition(frTarget);
            robot.rlDriveMtr.setTargetPosition(rlTarget);
            robot.rrDriveMtr.setTargetPosition(rrTarget);
            
            runtime.reset();
            robot.driveRun(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.flDriveMtr.isBusy() && robot.frDriveMtr.isBusy() &&
                   robot.rlDriveMtr.isBusy() && robot.rrDriveMtr.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Front T",  "Running to %7d :%7d ", flTarget,  frTarget);
                telemetry.addData("Front A",  "Running at %7d :%7d",
                                            robot.flDriveMtr.getCurrentPosition(),
                                            robot.frDriveMtr.getCurrentPosition());
                telemetry.addData("Rear T",  "Running to %7d :%7d ", rlTarget,  rrTarget);
                telemetry.addData("Rear A",  "Running at %7d :%7d",
                                            robot.rlDriveMtr.getCurrentPosition(),
                                            robot.rrDriveMtr.getCurrentPosition());
                telemetry.addData("Heading", 
                    String.format("%3d",-robot.gyro.getIntegratedZValue()));
                telemetry.update();
            }

            robot.driveStop();

            //sleep(2500);   // optional pause after each move
        }
    }
    public void strafeRightInches(double speed,
                             double inches, 
                             double timeoutS) {
        int flActual = robot.flDriveMtr.getCurrentPosition();
        int frActual = robot.frDriveMtr.getCurrentPosition();
        int rlActual = robot.rlDriveMtr.getCurrentPosition();
        int rrActual = robot.rrDriveMtr.getCurrentPosition();
        
        int flTarget = (int)(flActual + CPI*inches);
        int frTarget = (int)(frActual - CPI*inches);
        int rlTarget = (int)(rlActual - CPI*inches);
        int rrTarget = (int)(rrActual + CPI*inches);
        
        
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            robot.flDriveMtr.setTargetPosition(flTarget);
            robot.frDriveMtr.setTargetPosition(frTarget);
            robot.rlDriveMtr.setTargetPosition(rlTarget);
            robot.rrDriveMtr.setTargetPosition(rrTarget);
            
            runtime.reset();
            robot.driveRun(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.flDriveMtr.isBusy() && robot.frDriveMtr.isBusy() &&
                   robot.rlDriveMtr.isBusy() && robot.rrDriveMtr.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Front T",  "Running to %7d :%7d ", flTarget,  frTarget);
                telemetry.addData("Front A",  "Running at %7d :%7d",
                                            robot.flDriveMtr.getCurrentPosition(),
                                            robot.frDriveMtr.getCurrentPosition());
                telemetry.addData("Rear T",  "Running to %7d :%7d ", rlTarget,  rrTarget);
                telemetry.addData("Rear A",  "Running at %7d :%7d",
                                            robot.rlDriveMtr.getCurrentPosition(),
                                            robot.rrDriveMtr.getCurrentPosition());
                telemetry.addData("Heading", 
                    String.format("%3d",-robot.gyro.getIntegratedZValue()));
                telemetry.update();
            }

            robot.driveStop();

            //sleep(2500);   // optional pause after each move
        }
    }

    
}
