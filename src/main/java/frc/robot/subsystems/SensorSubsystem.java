// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SensorSubsystem extends SubsystemBase {

  private static DigitalInput DIO_0;
  private static DigitalInput DIO_2;
  private boolean lastOut0;
  private boolean lastOut2;

  /** Creates a new DriveSubsystem. */
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
  
    boolean output0 = DIO_0.get();
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("DIO_0").setBoolean(output0);
    boolean output2 = DIO_2.get();
    NetworkTableInstance.getDefault().getTable("Sensors").getEntry("DIO_2").setBoolean(output2);
    

    if (output0 != lastOut0) {
      System.out.println("DIO 0 is: " + output0);
      lastOut0 = output0;
    }
    if (output2 != lastOut2) {
      System.out.println("" + Timer.getFPGATimestamp());
      System.out.println("DIO 2 is: " + output2);
      lastOut2 = output2;
    }
  }

  /*
  Network Table
  get - Limelight helpers 408
  inbetween - Limelight helpers 412
  set - Limelight helpers 420

  NetworkTableInstance.getDefault().getTable(tableName).getEntry(entryName).getDouble(0.0)
  NetworkTableInstance.getDefault().getTable(tableName).getEntry(entryName).setDouble(0.0)
  Also works with stuff other than doubles
  setString() setDouble() setDoubleArray()
  */
}
