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
import processing.sound.*;
import ddf.minim.*;

Minim minim;
AudioPlayer song;
SoundFile file;
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

boolean done=false;

/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  file = new SoundFile(this, "pop1.wav");
  //file.play();
  
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
  
  wall4                   = new FBox(0.4, 1.4);
  wall4.setPosition(edgeTopLeftX+7.7, 9.2);
  wall4.setStatic(true);
  wall4.setFill(255,255,255);
  wall4.setNoStroke();
  world.add(wall4);
  
  wall6                   = new FBox(5, 0.5);
  wall6.setPosition(edgeTopLeftX+10, 10);
  wall6.setStatic(true);
  wall6.setFill(255,255,255);
  wall6.setNoStroke();
  world.add(wall6);
  
  wall7                   = new FBox(0.4, 1.4);
  wall7.setPosition(edgeTopLeftX+12.2, 9.2);
  wall7.setStatic(true);
  wall7.setFill(255,255,255);
  wall7.setNoStroke();
  world.add(wall7);
  
  circle1                   = new FCircle(3);
  circle1.setPosition(13, 5);
  circle1.setStatic(true);
  circle1.setFill(255, 255, 255);
  circle1.setNoStroke();
  world.add(circle1);
  
      
  circle2                   = new FCircle(1.7);
  circle2.setPosition(10, 7);
  circle2.setStatic(false);
  circle2.setFill(0, 0, 0);
  circle2.setNoStroke();
  world.add(circle2);

  
  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4);  
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
 
  
  /* If you are developing on a Mac users must update the path below 
   * from "../img/Haply_avatar.png" to "./img/Haply_avatar.png" 
   */
  haplyAvatar = loadImage("../img/tack.png"); 
  haplyAvatar.resize((int)(hAPI_Fisica.worldToScreen(1.5)), (int)(hAPI_Fisica.worldToScreen(1.5)));
  s.h_avatar.attachImage(haplyAvatar); 


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


 //void contactResult(FContactResult result) {
 //  // Draw an ellipse where the contact took place and as big as the normal impulse of the contact
 //  ellipse(result.getX(), result.getY(), result.getNormalImpulse(), result.getNormalImpulse());

 //  // Trigger your sound here
 //  // ...
 //  playAudio();
 //  done=true;
 //}



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    //file.play();
    
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
    
    
    if (s.h_avatar.isTouchingBody(wall4) || s.h_avatar.isTouchingBody(wall7) || s.h_avatar.isTouchingBody(wall6) && !file.isPlaying()){
      //file.play();
      playAudio();
      done=true;
      //delay(10);
      circle2.setNoFill();
    }
    else if (s.h_avatar.isTouchingBody(circle1)){
      circle2.setFill(0,0,0);
      circle2.setPosition(10, 8);
      done=false;
    }
    
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/
void playAudio(){
  if(done==false)
  {
    file.play();
    print("here");
  }
  
}
/* end helper functions section ****************************************************************************************/
