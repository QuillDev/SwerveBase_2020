package SKCommon.hardware;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RioAccel {
    
    private BuiltInAccelerometer accel = new BuiltInAccelerometer();

    //constructor for the accelerometer
    public RioAccel(){ 
    }
    
    /**
     * @return returns the total accelerations added together
     */
    public double getAcceleration(){
        //create 2d vector
        Vector2d accelVector = new Vector2d(accel.getX(), accel.getY());
        
        //return total acceleration from the 2d plane
        return accelVector.magnitude();
    }

    /**
     * prints the data from the accelerometer to the smart dashboard
     */
    public void printDashboard(){
        SmartDashboard.putNumber("Acceleration ", getAcceleration());
    }
}