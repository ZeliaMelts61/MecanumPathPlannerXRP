// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.TimeUnit;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import java.util.Map;
import java.util.Stack;
import java.util.HashMap;
//import java.util.EnumMap;

import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.lang.ModuleLayer.Controller;
//import java.util.Map;
import java.net.FileNameMap;


import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;

import com.fasterxml.jackson.annotation.JsonProperty;


public class SwappableController {


  public static class ControllerMappings {
      public Map<String, Controller> ControllerMappings;

      public static class Controller {
          @JsonProperty("ReadableName") // Map the "ReadableName" property in JSON to this field
          public String Name;
          
          public Map<String, Integer> Buttons;
          public Map<String, Integer> Axes;
      }
  }
  
  /** Creates a new SwappableController. */
  private int port;
  private int controllerConnectionTimeout = 5;
  private ControllerMappings.Controller currentControllerMappings;
  
  public static final int kControllerConnectionTimeout = 5;
  //private ConfigureBindings robotContainerConfigureBindings;

  private static ControllerMappings controllerMappings;
  private static Stack<EventLoop> myEventLoops = new Stack<>();
  private static ControllerSub controllerSub = null;
  


  public enum m_buttons {
    kA, kB, kX, kY, kLeftBumper, kRightBumper, kLeftTriggerButton, kRightTriggerButton,
    kMinus, kPlus, kLeftStick, kRightStick, kHome, kCapture, kMapableButton1, kMapableButton2,
    kMapableButton3, kMapableButton4, kPovUp, kPovDown, kPovLeft, kPovRight
  }
  public enum m_axes {
    kLeftX, kLeftY, kRightX, kRightY, kLeftTrigger, kRightTrigger, kThrottle
  }
  Map<m_buttons, Integer> m_ButtonsMap = new HashMap<>();
  Map<m_axes, Integer> m_AxesMap = new HashMap<>();
  


  

  private CommandGenericHID activeController;

  public SwappableController(int port, ConfigureBindings robotContainerConfigureBindings) {
    this(port, kControllerConnectionTimeout, robotContainerConfigureBindings);
  }

  public SwappableController(int port, int controllerConnectionTimeoutInSeconds, ConfigureBindings robotContainerConfigureBindings) {
    if (controllerSub == null) {
      controllerSub = new ControllerSub(robotContainerConfigureBindings);
    }
    //this.robotContainerConfigureBindings = robotContainerConfigureBindings;
    this.controllerConnectionTimeout=controllerConnectionTimeoutInSeconds;
    this.port = port;
    //load controller mappings if not already loaded
    if (controllerMappings == null) {
      try {
          ObjectMapper objectMapper = new ObjectMapper();
          File deployDirectory = Filesystem.getDeployDirectory();
          File mappingsFile = new File(deployDirectory, "ControllerMappingContants.json");
          
          controllerMappings = objectMapper.readValue(
              mappingsFile,
              ControllerMappings.class
          );
          System.out.println("Controller mappings loaded successfully.");
      } catch (Exception e) {
          e.printStackTrace();
      }
    }

    handleControllerChange();
    
    this.automaticallyConfigureController().asProxy().beforeStarting(Commands.waitUntil(() -> {
      Command command = CommandScheduler.getInstance().requiring(controllerSub);
      return command == null || !command.isScheduled();
    })
    ).schedule();
    
  }


  


      
    

  

  public void handleControllerChange() {
    // for (EventLoop eventLoop : myEventLoops) {
    //   eventLoop.clear();
    // }
    myEventLoops.clear();
    CommandScheduler.getInstance().getDefaultButtonLoop().clear();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    System.out.println("Driverstation connected: " +DriverStation.waitForDsConnection(10));
    /* 
    int sleepTime = 0;
    while (!DriverStation.isJoystickConnected(port)) {
      CommandScheduler.getInstance().getDefaultButtonLoop().poll();
      System.out.println("Waiting " + sleepTime + "s for joystick to connect on port: " + port);
      try {
        Thread.sleep(1000); // Wait for 1 second before checking again
        if (++sleepTime > this.controllerConnectionTimeout) {
          System.out.println("Joystick not connected after " + this.controllerConnectionTimeout + " seconds. Stopping wait.");
          break;
        }
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt(); // Restore interrupted status
      }
    }
    */
    System.out.println("Handling controller change for port: " + port);
    
    if (DriverStation.getJoystickIsXbox(port)){
      this.activeController = (CommandXboxController) new CommandXboxController(port);
      remapButtons("Xbox");
    } else if (DriverStation.getJoystickName(port).toLowerCase().contains("extreme 3d pro")) {
      this.activeController = (CommandJoystick) new CommandJoystick(port);
      remapButtons("extreme 3d pro");
    /* 
    } else if (DriverStation.getJoystickName(port).toLowerCase().contains("wireless gamepad")) {
      this.activeController = (CommandGenericHID) new CommandGenericHID(port);
      remapButtons("wireless gamepad");
    } else if (DriverStation.getJoystickName(port).toLowerCase().contains("vjoy device")) {
      this.activeController = (CommandGenericHID) new CommandGenericHID(port);
      remapButtons("vjoy device");
    */
    } else{
      this.activeController = (CommandGenericHID) new CommandGenericHID(port); // Handle other types or invalid cases
      try {
        remapButtons(DriverStation.getJoystickName(port).toLowerCase());
      } catch (Exception e) {
          if (e.getMessage().contains("Controller type not found")) {
              //CONTROLLER NOT FOUND
              System.out.println(e.getMessage());
              System.out.println("Controller type not recognized or unsupported. Using generic HID controller.");
          } else {
              throw e; // Rethrow if it's a different exception
          }
      }
    }
  }

  CommandGenericHID getCommandHid(){
    return activeController;
  }

  private void remapButtons(String controllerType) {
    ControllerMappings.Controller controller = null;
    // Get the specific controller mappings from the in-memory data
    if (controllerType == "Xbox"){
        controller = controllerMappings.ControllerMappings.get("Xbox");
        this.currentControllerMappings = controller;
    } else{
        controller = controllerMappings.ControllerMappings.get(controllerType);
        this.currentControllerMappings = controller;
        if (controller == null) {
            throw new IllegalArgumentException("Controller type not found: " + controllerType);
        }
    }
    

    // Clear existing mappings
    m_ButtonsMap.clear();
    m_AxesMap.clear();

    // Map buttons
    for (Map.Entry<String, Integer> entry : controller.Buttons.entrySet()) {
        try {
            m_ButtonsMap.put(m_buttons.valueOf(entry.getKey()), entry.getValue());
        } catch (IllegalArgumentException e) {
            System.out.println("Button " + entry.getKey() + " does not exist in this controller mapping.");
            // Handle the case where the button does not exist in the mapping
            //m_ButtonsMap.put(m_buttons.valueOf(entry.getKey()), null);
        }
    }

    // Map axes
    for (Map.Entry<String, Integer> entry : controller.Axes.entrySet()) {
        try {
            m_AxesMap.put(m_axes.valueOf(entry.getKey()), entry.getValue());
        } catch (IllegalArgumentException e) {
            System.out.println("Axis " + entry.getKey() + " does not exist in this controller mapping.");
            // Handle the case where the axis does not exist in the mapping
            //m_AxesMap.put(m_axes.valueOf(entry.getKey()), null);
        }
    }
    System.out.println("Buttons and axes sucessfully remapped for controller: " + controllerType + " on port: " + this.port);
}

  //only use the pov methods from this class, not the pov methods from the CommandGenericHID class as they won't work with all controllers
  public CommandGenericHID getActiveController() {
    return activeController;
  }

  public int getPort() {
    return port;
  }

  public boolean controlerIsValid(){
    return this.isConnected() && DriverStation.isJoystickConnected(this.port) && this.currentControllerMappings!=null;
  }

  public boolean isConnected(){
    if (activeController == null) {
      return false;
    }
    return activeController.isConnected();
  }

  

  public double getX() {
    
    return activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kLeftX, 25));
  }

  public double getY() {
    return activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kLeftY, 25));
  }

  public double getZ() {
    return activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kRightX, 25));
  }

  public Trigger a(){
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kA, 25));
  }

  public Trigger a(EventLoop eventLoop) {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kA, 25), eventLoop);
  }

  public Trigger b(){

    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kB, 25));
  }

  public Trigger b(EventLoop eventLoop) {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kB, 25), eventLoop);
  }

  public Trigger x(){
   return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kX, 25));
  }

  public Trigger x(EventLoop eventLoop) {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kX, 25), eventLoop);
  }

  public Trigger y(){
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kY, 25));
  }

  public Trigger y(EventLoop eventLoop) {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kY, 25), eventLoop);
  }

  public Trigger leftBumper() {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kLeftBumper, 25));
  }

  public Trigger leftBumper(EventLoop eventLoop) {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kLeftBumper, 25), eventLoop);
  }
  public Trigger rightBumper() {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kRightBumper, 25));
  }
  public Trigger rightBumper(EventLoop eventLoop) {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kRightBumper, 25), eventLoop);
  }
  public Trigger leftTrigger() {
    if (m_AxesMap.getOrDefault(m_axes.kLeftTrigger, null) == null) {
      return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kLeftTriggerButton, 25));
    } else {
      return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> 0.5<activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kLeftTrigger, 25)));
    }
  }

  public Trigger leftTrigger(double threshold) {
    if (m_AxesMap.getOrDefault(m_axes.kLeftTrigger, null) == null) {
      return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kLeftTriggerButton, 25));
    } else {
      return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> threshold<activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kLeftTrigger, 25)));
    }
  }

  public Trigger leftTrigger(double threshold, EventLoop eventLoop) {
    if (m_AxesMap.getOrDefault(m_axes.kLeftTrigger, null) == null) {
      return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kLeftTriggerButton, 25), eventLoop);
    } else {
      return new Trigger(eventLoop, () -> threshold<activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kLeftTrigger, 25)));
    }
  }

  public Trigger rightTrigger() {
    if (m_AxesMap.getOrDefault(m_axes.kRightTrigger, null) == null) {
      return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kRightTriggerButton, 25));
    } else {
      return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> 0.5<activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kRightTrigger, 25)));
    }
  }

  public Trigger rightTrigger(double threshold) {
    if (m_AxesMap.getOrDefault(m_axes.kRightTrigger, null) == null) {
      return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kRightTriggerButton, 25));
    } else {
      return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> threshold<activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kRightTrigger, 25)));
    }
  }

  public Trigger rightTrigger(double threshold, EventLoop eventLoop) {
    if (m_AxesMap.getOrDefault(m_axes.kRightTrigger, null) == null) {
      return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kRightTriggerButton, 25), eventLoop);
    } else {
      return new Trigger(eventLoop, () -> threshold<activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kRightTrigger, 25)));
    }
  }

  public Trigger minus(){
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kMinus, 25));
  }

  public Trigger minus(EventLoop eventLoop){
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kMinus, 25), eventLoop);
  }

  public Trigger plus(){
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kPlus, 25));
  }

  public Trigger plus(EventLoop eventLoop){
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kPlus, 25), eventLoop);
  }

  public Trigger leftStick(){
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kLeftStick, 25));
  }
  
  public Trigger leftStick(EventLoop eventLoop){
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kLeftStick, 25), eventLoop);
  }
  public Trigger rightStick(){
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kRightStick, 25));
  }
  public Trigger rightStick(EventLoop eventLoop){
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kRightStick, 25), eventLoop);
  }

  public double getRightX() {
    //Throttle on the joystick is the right X axis 
    if (m_AxesMap.getOrDefault(m_axes.kRightX, null) == null) {
      return activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kThrottle, 25));
    } else {
      return activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kRightX, 25));
    }
  }

  public double getRightTriggerAxis(){
    if (m_AxesMap.getOrDefault(m_axes.kRightTrigger, null) == null) {
      return (activeController.getHID().getRawButton(m_ButtonsMap.getOrDefault(m_buttons.kRightTriggerButton, 25)) ? 1.0 : 0.0);
    }
    return activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kRightTrigger, 25));
  }

  public double getLeftTriggerAxis(){
    if (m_AxesMap.getOrDefault(m_axes.kLeftTrigger, null) == null) {
      return (activeController.getHID().getRawButton(m_ButtonsMap.getOrDefault(m_buttons.kLeftTriggerButton, 25)) ? 1.0 : 0.0);
    }
    return activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kLeftTrigger, 25));
  }
  
  public Trigger home() {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kHome, 25));
  }
  public Trigger home(EventLoop eventLoop) {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kHome, 25), eventLoop);
  }
  public Trigger capture() {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kCapture, 25));
  }
  public Trigger capture(EventLoop eventLoop) {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kCapture, 25), eventLoop);
  }
  public Trigger mapableButton1() {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kMapableButton1, 25));
  }
  public Trigger mapableButton1(EventLoop eventLoop) {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kMapableButton1, 25), eventLoop);
  }
  public Trigger mapableButton2() {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kMapableButton2, 25));
  }
  public Trigger mapableButton2(EventLoop eventLoop) {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kMapableButton2, 25), eventLoop);
  }
  public Trigger mapableButton3() {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kMapableButton3, 25));
  }
  public Trigger mapableButton3(EventLoop eventLoop) {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kMapableButton3, 25), eventLoop);
  }
  public Trigger mapableButton4() {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kMapableButton4, 25));
  }
  public Trigger mapableButton4(EventLoop eventLoop) {
    return activeController.button(m_ButtonsMap.getOrDefault(m_buttons.kMapableButton4, 25), eventLoop);
  }
  public double getLeftX() {
    return activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kLeftX, 25));
  }
  public double getLeftY() {
    return activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kLeftY, 25));
  }
  public double getRightY() {
    return activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kRightY, 25));
  }

  public double getThrottle() {
    return activeController.getHID().getRawAxis(m_AxesMap.getOrDefault(m_axes.kThrottle, 25));
  }

  public Trigger povUp() {
    return pov(0);
  }

  public Trigger povUp(EventLoop eventLoop) {
    return pov(0, eventLoop);
  }

  public Trigger povLeft() {
    return pov(270);
  }

  public Trigger povLeft(EventLoop eventLoop) {
    return pov(270, eventLoop);
  }

  public Trigger povDown() {
    return pov(180);
  }

  public Trigger povDown(EventLoop eventLoop) {
    return pov(180, eventLoop);
  }

  public Trigger povRight() {
    return pov(90);
  }

  public Trigger povRight(EventLoop eventLoop) {
    return pov(90, eventLoop);
  }

  public Trigger povUpLeft(){
    return pov(315);
  }

  public Trigger povUpLeft(EventLoop eventLoop){
    return pov(315, eventLoop);
  }

  public Trigger povUpRight(){
    return pov(45);
  }

  public Trigger povUpRight(EventLoop eventLoop){
    return pov(45, eventLoop);
  }

  public Trigger povDownLeft(){
    return pov(225);
  }

  public Trigger povDownLeft(EventLoop eventLoop){
    return pov(225, eventLoop);
  }

  public Trigger povDownRight(){
    return pov(135);
  }

  public Trigger povDownRight(EventLoop eventLoop){
    return pov(135, eventLoop);
  }

  public Trigger povCenter() {
    return pov(-1);
  }

  public Trigger povCenter(EventLoop eventLoop) {
    return pov(-1, eventLoop);
  }
  
  public double pov(){
    //System.out.println("pov Count" + activeController.getHID().getPOVCount());
    if (activeController.getHID().getPOVCount() > 0 || !isConnected()){
      //System.out.println("has pov");
      return activeController.getHID().getPOV();
    }
    
    try {
        if (activeController.getHID().getRawButton(m_ButtonsMap.getOrDefault(m_buttons.kPovUp, null)) && 
            activeController.getHID().getRawButton(m_ButtonsMap.getOrDefault(m_buttons.kPovLeft, null))) {
        return 315;
        } else if (activeController.getHID().getRawButton(m_ButtonsMap.getOrDefault(m_buttons.kPovUp, null)) && 
                activeController.getHID().getRawButton(m_ButtonsMap.getOrDefault(m_buttons.kPovRight, null))) {
        return 45;
        } else if (activeController.getHID().getRawButton(m_ButtonsMap.getOrDefault(m_buttons.kPovDown, null)) && 
                activeController.getHID().getRawButton(m_ButtonsMap.getOrDefault(m_buttons.kPovLeft, null))) {
        return 225;
        } else if (activeController.getHID().getRawButton(m_ButtonsMap.getOrDefault(m_buttons.kPovDown, null)) && 
                activeController.getHID().getRawButton(m_ButtonsMap.getOrDefault(m_buttons.kPovRight, null))) {
        return 135;
        } else if (activeController.getHID().getRawButton(m_ButtonsMap.getOrDefault(m_buttons.kPovUp, null))) {
        return 0;
        } else if (activeController.getHID().getRawButton(m_ButtonsMap.getOrDefault(m_buttons.kPovDown, null))) {
        return 180;
        } else if (activeController.getHID().getRawButton(m_ButtonsMap.getOrDefault(m_buttons.kPovLeft, null))) {
        return 270;
        } else if (activeController.getHID().getRawButton(m_ButtonsMap.getOrDefault(m_buttons.kPovRight, null))) {
        return 90;
        }
        return -1;
    
    } catch (NullPointerException e) {
        System.out.println("Button mapping not found for POV buttons. Returning -1 (non-pressed state)");
        return -1; // If the button mapping is not found, return -1
    }
}

  public Trigger pov(int angle) {
    return pov(angle, CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger pov(int angle, EventLoop eventLoop) {
    return new Trigger(eventLoop, () -> pov() == angle);
  }





  public interface ConfigureBindings {
    void configureBindings();
  }

  public static final class ControllerSub extends SubsystemBase {
    public final ConfigureBindings robotContainerConfigureBindings;
    public ControllerSub(ConfigureBindings robotContainerConfigureBindings) {
      this.robotContainerConfigureBindings = robotContainerConfigureBindings;
    }
    @Override
    public void periodic() {}
    
    
    public Command ControllerChangeOnce(ControllerSub controllerSub, SwappableController controller) {
      return Commands.runOnce(() -> {
        controller.handleControllerChange();
        controllerSub.robotContainerConfigureBindings.configureBindings();
      }, controllerSub).ignoringDisable(true);
    }
  
    
  }
  private static final class automaticallyConfigureController extends Command {
    private SwappableController controller;
    private ControllerSub controllerSub;
    public automaticallyConfigureController(SwappableController controller, ControllerSub controllerSub) {
      this.controller = controller;
      this.controllerSub = controllerSub;
      addRequirements(controllerSub);
    }
    @Override
    public void initialize() {
      controller.handleControllerChange();
    }
    @Override
    public void execute() {
      if (!controller.controlerIsValid()) {
        controller.handleControllerChange();
      }
    }
    @Override
    public boolean isFinished() {
      return controller.controlerIsValid();
    }
    @Override
    public void end(boolean interrupted) {
      if (interrupted) {
        System.out.println("SwappableControllerAutoConfigure command was interrupted.");
      } else {
        controllerSub.robotContainerConfigureBindings.configureBindings();
      }
    }
  }

  //use this::ConfigureBindings in robotContainer for robotContainerConfigureBindings
  public Command automaticallyConfigureController() {
    return this.automaticallyConfigureController(this.controllerConnectionTimeout);
  }
  //use this::ConfigureBindings in robotContainer for robotContainerConfigureBindings
  public Command automaticallyConfigureController(Time timeout) {
    return this.automaticallyConfigureController(timeout.in(Seconds));
  }
  //use this::ConfigureBindings in robotContainer for robotContainerConfigureBindings
  public Command automaticallyConfigureController(double timeout) {
    return new automaticallyConfigureController(this, controllerSub).ignoringDisable(true).withTimeout(timeout);
  }
}
