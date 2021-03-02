/**
 **********************************************************************************************************************
 * @file       sketch_4_Wall_Physics.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V4.1.0
 * @date       08-January-2021
 * @brief      wall haptic example using 2D physics engine 
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                      = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                = new PVector(0, 0); 

/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 20.0;  
float             worldHeight                         = 20.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;


/* Initialization of wall */
FBox              wall, wall2, wall3, wall4, wall5, wall6, wall7, wall8, wall9,wall10; 
FCircle           circle1,circle2;
FBlob             blob1;


/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar,pac2;

/* end elements definition *********************************************************************************************/ 



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(800, 800);
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */ 
  haplyBoard          = new Board(this, "COM3", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  
  /* creation of wall */
  //wall                   = new FBox(10.0, 0.5);
  //wall.setPosition(edgeTopLeftX+worldWidth/6.0, edgeTopLeftY+2*worldHeight/3.0);
  //wall.setStatic(true);
  //wall.setFill(255,255,255);
  //world.add(wall);
  //wall.setNoStroke();
  //System.out.println(edgeTopLeftX+worldWidth); 
  //System.out.println(edgeTopLeftY+2*worldHeight); 
  
  //wall2                   = new FBox(4, 3);
  //wall2.setPosition(edgeTopLeftX+4, 6);
  //wall2.setStatic(true);
  //wall2.setFill(255,255,255);
  //  wall2.setNoStroke();
  //world.add(wall2);

  //wall3                   = new FBox(1, 0.8);
  //wall3.setPosition(13, 5);
  //wall3.setStatic(true);
  //wall3.setFill(255,255,255);
  //  wall3.setNoStroke();
  //world.add(wall3);
  
  wall4                   = new FBox(4, 8);
  wall4.setPosition(edgeTopLeftX+10, 15);
  wall4.setStatic(true);
  wall4.setFill(255,255,255);
    wall4.setNoStroke();
  world.add(wall4);
  
  //GOAL
  wall5                   = new FBox(0.8, 0.8);
  wall5.setPosition(edgeTopLeftX+1.5, 18);
  wall5.setStatic(true);
  wall5.setFriction(100);
  wall5.setFill(255,255,255);
    wall5.setNoStroke();
  world.add(wall5);
  
  //wall6                   = new FBox(0.5, 8);
  //wall6.setPosition(edgeTopLeftX+8, 5);
  //wall6.setStatic(true);
  //wall6.setFill(0,255,255);
  //  wall6.setNoStroke();
  //world.add(wall6);
  
  wall7                   = new FBox(2, 8);
  wall7.setPosition(edgeTopLeftX+18, 15);
  wall7.setStatic(true);
  wall7.setFill(255,255,255);
    wall7.setNoStroke();
  world.add(wall7);
  
  //wall8                   = new FBox(0.5, 2);
  //wall8.setPosition(edgeTopLeftX+4, 2);
  //wall8.setStatic(true);
  //wall8.setFill(0,255,255);
  //  wall8.setNoStroke();
  //world.add(wall8); 
  
  
  //wall9                   = new FBox(2, 8);
  //wall9.setPosition(5, 15);
  //wall9.setStatic(true);
  //wall9.setFill(0,255,255);
  //  wall9.setNoStroke();
  //world.add(wall9);
  
    
  //wall10                   = new FBox(3, 2);
  //wall10.setPosition(11, 8);
  //wall10.setStatic(true);
  //wall10.setFill(255,0,255);
  //  wall10.setNoStroke();
  //  world.add(wall10); 
    
    
  //circle1                   = new FCircle(4);
  //circle1.setPosition(15, 15);
  //circle1.setStatic(true);
  //circle1.setFill(0, 0, 0);
  //world.add(circle1);
  
      
  circle2                   = new FCircle(2);
  circle2.setPosition(13, 8);
  circle2.setStatic(false);
  circle2.setFill(250, 250, 250);
  circle2.setNoStroke();
  world.add(circle2);
  
  //FBlob blob1 = new FBlob();
  //blob1.setAsCircle(13);
  ////blob1.setPosition(7, 15);
  //blob1.setStatic(false);
  //blob1.setDensity(1);
  //blob1.setFriction(15);
  //blob1.setFill(128, 255, 0);
  //world.add(blob1);
  
  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4);  
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
 
  
  /* If you are developing on a Mac users must update the path below 
   * from "../img/Haply_avatar.png" to "./img/Haply_avatar.png" 
   */
  haplyAvatar = loadImage("../img/pac3.png"); 
  haplyAvatar.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1)));
  s.h_avatar.attachImage(haplyAvatar); 

  pac2 = loadImage("../img/pac2.png"); 
  pac2.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1)));
  wall5.attachImage(pac2); 

  /* world conditions setup */
  world.setGravity((0.0), (6000.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(0.4);
  world.setEdgesFriction(1.2);
  
  
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  

  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    
    
    s.updateCouplingForce();
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
