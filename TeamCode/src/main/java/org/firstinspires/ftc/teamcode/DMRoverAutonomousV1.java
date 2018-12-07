package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "DMRoverAutonomousV1")
public class DMRoverAutonomousV1 extends DMRoverAbstract
{

    //Declare variables used for encoders later in the program
    float targetDrDistInch;
    double targetPower;
    float ENCODER_CNT_PER_IN_DRIVE = (float) (3.5*3.15159/288);
    //Set encoder counts per inch to the wheel circumference in inches divided by the number of counts per motor rotation
    double targetPosLeft;
    double targetPosRight;


    public void init()
    {
        super.init();
    }

    /*
    We used the cmdMoveR method to calculate a new distance target for our drive train (in encoder
    counts) and to begin the move to the target location.
    The cmdMoveR method uses the parameters (distance inches, encoder count per inch, power, motor)
    to calculate the number of encoder counts to turn the motor. The following lines of code are how we
    implemented the cmdMoveR method:

    targetPosLeft = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, leftDrive);
    targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, rightDrive);
    */


    public void loop()
    {

        switch(seqRobot)
        {

            case(1):
            {
            // Lowering the robot on to the playing field
            liftArm.setPower(1);
            sleep(1000);
            liftArm.setPower(0);

            seqRobot++;
            break;

            }


            case(2):
            {
                // Detach the hook on the landing arm from the lander
                targetDrDistInch = 2;
                targetPower = .4;
                targetPosLeft = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, leftDrive);
                targetDrDistInch = -2;
                targetPower = .4;
                targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, rightDrive);

                seqRobot++;
                break;
            }


            case (3):
            {
                targetDrDistInch = 2;
                targetPower = .5;
                targetPosLeft = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, leftDrive);
                targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, rightDrive);

                seqRobot++;
                break;
            }


            case (4):
            {
                targetDrDistInch = -2;
                targetPower = .4;
                targetPosLeft = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, leftDrive);
                targetDrDistInch = 2;
                targetPower = .4;
                targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, rightDrive);

                seqRobot++;
                break;
            }


            case (5):
            {
                // Driving forward to the minerals and lowering MCS arm to sample the gold mineral
                targetDrDistInch = 2;
                targetPower = .5;
                targetPosLeft = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, leftDrive);
                targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, rightDrive);

                seqRobot++;
                break;
            }


            case(6):
            {
                // TODO sample

                seqRobot++;
                break;
            }


            case(7):
            {
                targetDrDistInch = 2;
                targetPower = .5;
                targetPosLeft = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, leftDrive);
                targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, rightDrive);

                seqRobot++;
                break;
            }


            case(8):
            {
                // Back up and drive to the depot
                targetDrDistInch = -2;
                targetPower = .5;
                targetPosLeft = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, leftDrive);
                targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, rightDrive);

                seqRobot++;
                break;
            }


            case(9):
            {
                targetDrDistInch = -6;
                targetPower = .4;
                targetPosLeft = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, leftDrive);
                targetDrDistInch = 6;
                targetPower = .4;
                targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, rightDrive);

                seqRobot++;
                break;
            }


            case(10):
            {
                targetDrDistInch = 48;
                targetPower = .5;
                targetPosLeft = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, leftDrive);
                targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, rightDrive);

                seqRobot++;
                break;
            }


            case(11):
            {
                targetDrDistInch = -4;
                targetPower = .4;
                targetPosLeft = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, leftDrive);
                targetDrDistInch = 4;
                targetPower = .4;
                targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, rightDrive);

                seqRobot++;
                break;
            }


            case(12):
            {
                targetDrDistInch = 42;
                targetPower = .5;
                targetPosLeft = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, leftDrive);
                targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, rightDrive);

                seqRobot++;
                break;
            }


            case(13):
            {
                // Deposit marker by moving the MCS arm
                mineralArm.setDirection(DcMotor.Direction.FORWARD);
                mineralArm.setPower(1);
                sleep(400);
                mineralArm.setDirection(DcMotor.Direction.REVERSE);
                sleep(400);
                mineralArm.setPower(0);

                seqRobot++;
                break;
            }


            case(14):
            {
                // Drive to Crater to Park
                targetDrDistInch = -9;
                targetPower = .4;
                targetPosLeft = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, leftDrive);
                targetDrDistInch = 9;
                targetPower = .4;
                targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, rightDrive);

                seqRobot++;
                break;
            }


            case(15):
            {
                targetDrDistInch = 114;
                targetPower = 1;
                targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, rightDrive);
                targetPosRight = cmdMoveR(targetDrDistInch, ENCODER_CNT_PER_IN_DRIVE, targetPower, leftDrive);

                seqRobot++;
                break;
            }


            default:
            {
                //print case number to screen
                telemetry.addData("Case: ", seqRobot );
                telemetry.update();
                break;
            }
        }
    }


    public void stop()
    {
    super.stop();
    }
}
