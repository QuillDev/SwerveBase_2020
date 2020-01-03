package SKCommon.swerve;

import frc.robot.Map;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class SwerveDrivetrain {

    //enum for Drivemodes
    public enum Drivemode {
        kDriveSimple, kDriveByFeet;
    }

    //Create private versions of the modules to be used in the SwerveDrivetrain constructor
    private SwerveModule topLeft;
    private SwerveModule topRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    
    //Constructor for creating a Swerve Drivetrain
    public SwerveDrivetrain() {
        this.topRight  = new SwerveModule ( Map.flDrive, Map.flAngle, Map.flEncoder, Map.kflAngleOffset);
        this.topLeft   = new SwerveModule ( Map.frDrive, Map.frAngle, Map.frEncoder, Map.kfrAngleOffset );
        this.backLeft  = new SwerveModule ( Map.blDrive, Map.blAngle, Map.blEncoder, Map.kblAngleOffset );
        this.backRight = new SwerveModule ( Map.brDrive, Map.brAngle, Map.brEncoder, Map.kbrAngleOffset );

        //set module names
        this.topLeft.setName("Top Left");
        this.topRight.setName("Top Right");
        this.backLeft.setName("Back Left");
        this.backRight.setName("Back Right");
    }

    /**
     * 
     * @param fwd foward value from joystick
     * @param str strafe value from joystick
     * @param rcw rotation value from joystick
     * @param gyroOffset current gyroscope value
     * @param fieldOriented whether the bot is field oriented mode
     * @param mode drive control mode
     */
    public void drive( double fwd, double str, double rcw, double gyroOffset, boolean fieldOriented, Drivemode mode){

        //Check deadzones and turn values to zero if they're within the deadzone
        if(Math.abs(fwd) < Map.deadzone) { fwd = 0; } // zero fwd
        if(Math.abs(str) < Map.deadzone) { str = 0; } // zero str
        if(Math.abs(rcw) < Map.deadzone) { rcw = 0; } // zero rcw

        //Check if fieldoriented and if so add values
        if(fieldOriented){
            //Convert values to be field centric.
            double temp = fwd * Math.cos(gyroOffset) + str * Math.sin(gyroOffset);
            str = -fwd * Math.sin(gyroOffset) + str * Math.cos(gyroOffset);
            fwd = temp;
        }        

        //Calculate vectors and multiply them by Map.chassisMagnitudes
        double a = str - rcw * ( Map.WHEELBASE / Map.chassisMagnitude );
        double b = str + rcw * ( Map.WHEELBASE / Map.chassisMagnitude );
        double c = fwd - rcw * ( Map.TRACKWIDTH  / Map.chassisMagnitude );
        double d = fwd + rcw * ( Map.TRACKWIDTH  / Map.chassisMagnitude );

        //calcualte wheel drive speeds [tlSpeed, trSpeed, blSpeed, brSpeed]
        double trSpeed = new Vector2d(b, c).magnitude();
        double tlSpeed = new Vector2d(b, d).magnitude();
        double blSpeed = new Vector2d(a, d).magnitude();
        double brSpeed = new Vector2d(a, c).magnitude();

        //calculate wheel angles [tlAngle, trAngle, blAngle, brAngle]
        //These values are in RADIANS, we use radians because they are far more convenient for what we are doing
        double trAngle = Math.atan2(b, c);
        double tlAngle = Math.atan2(b, d);
        double blAngle = Math.atan2(a, d);
        double brAngle = Math.atan2(a, c);

        //set the max value to the maximum of all the speeds
        double max = Math.max(tlSpeed, Math.max(trSpeed, Math.max(blSpeed, brSpeed)));
        
        //if the max value is more than 1 normalize all of the values
        if(max > 1){

            //divide by max to normalize the speed values.
            tlSpeed /= max;
            trSpeed /= max;
            blSpeed /= max;
            brSpeed /= max;
        }

        //if the mode is kDriveSimple
        if(mode == Drivemode.kDriveSimple){

            //use driveSimple command to drive all of the modules in unison
            topLeft.driveSimple  ( tlSpeed, tlAngle );
            topRight.driveSimple ( trSpeed, trAngle );
            backLeft.driveSimple ( blSpeed, blAngle );
            backRight.driveSimple( brSpeed, brAngle );
        }
    }

    /**
     * @return the topLeft
     */
    public SwerveModule getTopLeft() {
        return topLeft;
    }

    /**
     * @return the topRight
     */
    public SwerveModule getTopRight() {
        return topRight;
    }
    
    /**
     * @return the backLeft
     */
    public SwerveModule getBackLeft() {
        return backLeft;
    }

    /**
     * @return the backRight
     */
    public SwerveModule getBackRight() {
        return backRight;
    }
}