/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinLEDSubsystemConstants;

public class BlinkinLEDSubsystem extends SubsystemBase {

  private final static Spark blinkin = new Spark(BlinkinLEDSubsystemConstants.PWM_PORT);
 
 
  HashMap<String, Double> blinkinpwmHashMap = new HashMap<String, Double>(); 


  /**
   * Creates a new BlinkinLEDSubsystem for controlling and manipulating a Blin.
   * 
   */
  public BlinkinLEDSubsystem() {

  /** In future have freshman put key(color)-value (outpput number) into a CSV table or something to parse and have a complete 
   * Map<K,V> of possible pattters in future
   * For, now only putting what we need
   */

    blinkinpwmHashMap.put("Standby", -0.13);//0.67 solid gold
    blinkinpwmHashMap.put("CSEE Blue", 0.87);
    blinkinpwmHashMap.put("MAT Yellow", 0.69);
    blinkinpwmHashMap.put("MSET Red", 0.61);
    blinkinpwmHashMap.put("IDEA Green", 0.77);
    blinkinpwmHashMap.put("ACE Orange", 0.65);
  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void selectPattern(String pattern){
    blinkin.set(blinkinpwmHashMap.getOrDefault(pattern, blinkinpwmHashMap.get("Bosco Tech Gold")));
  }

  public void standby_mode(){
    blinkin.set(blinkinpwmHashMap.get("Standby"));
  }

  public void csee_blue(){
    blinkin.set(blinkinpwmHashMap.get("CSEE Blue"));
  }

  public void mat_yellow(){
    blinkin.set(blinkinpwmHashMap.get("MAT Yellow"));
  }

  public void mset_red(){
    blinkin.set(blinkinpwmHashMap.get("MSET Red"));
  }

  public void idea_green(){
    blinkin.set(blinkinpwmHashMap.get("IDEA Green"));
  }

  public void ace_orange(){
    blinkin.set(blinkinpwmHashMap.get("ACE Orange"));
  }

  //too much time to implement with a hahsmap

  //public String getSelectedColor(){

}



