package SKCommon.hardware;

import com.analog.adis16448.frc.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class qGyro {
    
    private ADIS16448_IMU gyro = new ADIS16448_IMU();

    //adapt the file to be more general maybe.
    public qGyro(){ 
    }

    /**
     * @return return the gyro angle in degrees
     */
    public double readAngle(){
        return gyro.getAngle();
    }

    /**
     * @return return the gyro angle in radians
     */
    public double readAngleInRadians(){
        return Math.toRadians(gyro.getAngle());
    }

    /**
     * Reset the gyroscope (Current angle is set to zero)
     */
    public void reset(){
        gyro.reset();
    }

    /**
     * Calibrate the gyroscope
     */
    public void calibrate(){
        gyro.calibrate();
    }

    /**
     * Print angle in degrees to the gyro
     */
    public void printDashboard(){
        //print angle in degrees to the dashboard
        SmartDashboard.putNumber("Angle In Degrees", readAngle());

        //print angle in degrees to the dashboard
        SmartDashboard.putNumber("Angle In Radians", readAngleInRadians());
    }
    
}