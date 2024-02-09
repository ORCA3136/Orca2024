package frc.robot.subsystems;

//import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.DataLogManager;
//import edu.wpi.first.util.datalog.BooleanLogEntry;
//import edu.wpi.first.util.datalog.DataLog;
//import edu.wpi.first.util.datalog.DoubleLogEntry;
//import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.Timer;

//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;

public class SensorSubsystem extends SubsystemBase {

  private static DigitalInput DIO_0;
  private static DigitalInput DIO_2;
  private boolean lastOut0;
  private boolean lastOut2;

  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  /** Creates a new SensorSubsystem. */
  public SensorSubsystem() {
    if (DIO_0 == null) {
      System.out.println("DIO_0 init");
      DIO_0 = new DigitalInput(0);
    }
    if (DIO_2 == null) {
      System.out.println("DIO_2 init");
      DIO_2 = new DigitalInput(2);
    }

    System.out.println("Init");
  }

  @Override
  public void periodic() {
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("GyroX").setDouble(m_gyro.getAccelX());
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("GyroY").setDouble(m_gyro.getAccelY());
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("GyroZ").setDouble(m_gyro.getAccelZ());

    boolean output0 = DIO_0.get();
    boolean output2 = DIO_2.get();

    double time = Timer.getMatchTime();
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("MatchTime").setDouble(time);
    double totalTime = Timer.getFPGATimestamp();
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("TotalTime").setDouble(totalTime);

    if (output0 != lastOut0) {
      NetworkTableInstance.getDefault().getTable("Sensors").getEntry("DIO_0").setBoolean(output0);
      lastOut0 = output0;
    }
    if (output2 != lastOut2) {
      NetworkTableInstance.getDefault().getTable("Sensors").getEntry("DIO_2").setBoolean(output2);
      lastOut2 = output2;
    }
  }
}
