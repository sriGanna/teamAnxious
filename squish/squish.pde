/**
 **********************************************************************************************************************
 * @file       sketch_5_Shapes_Physics.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V5.0.0
 * @date       08-January-2021
 * @brief      Shapes haptic example using 2D physics engine 
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

/* virtual wall parameter  */
float             kWall                               = 450;
PVector           fWall                               = new PVector(0, 0);
PVector           penWall                             = new PVector(0, 0);
PVector           posWall                             = new PVector(0.01, 0.10);

/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;

/* pantagraph link parameters in meters */
float             l                                   = 0.07;
float             L                                   = 0.09;


/* end effector radius in meters */
float             rEE                                 = 0.006;


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
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 25.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;


/* Initialization of interative shapes */
FBox              b;
FPoly             t;
FCircle           g;
FCircle           e;
FBlob             f;


/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar;



/* end elements definition *********************************************************************************************/ 



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  file = new SoundFile(this, "squish.mp3");
  /* screen size definition */
  size(1000,1000);
  
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
  haplyBoard          = new Board(this, Serial.list()[0], 0);
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
  
  
  /* creation of square shape */
  //b                   = new FBox(3.0, 3.0);
  //b.setPosition(edgeTopLeftX+worldWidth/4.0, edgeTopLeftY+worldHeight/2.0);
  //b.setDensity(30);
  //b.setFill(random(255), random(255), random(255));
  //world.add(b);
  
  
  /* creation of T shape */
  //t                   = new FPoly(); 
  //t.vertex(-1.5, -1.0);
  //t.vertex( 1.5, -1.0);
  //t.vertex( 3.0/2.0, 0);
  //t.vertex( 1.0/2.0, 0);
  //t.vertex( 1.0/2.0, 4.0/2.0);
  //t.vertex(-1.0/2.0, 4.0/2.0);
  //t.vertex(-1.0/2.0, 0);
  //t.vertex(-3.0/2.0, 0);
  //t.setPosition(edgeTopLeftX+10, edgeTopLeftY+5); 
  //t.setDensity(50); 
  //t.setFill(random(255),random(255),random(255));
  //world.add(t);
  
  
  /* creation of small circle shape */
  //g                   = new FCircle(1.0);
  //g.setPosition(7, 7);
  //g.setFill(random(255),random(255),random(255));
  //g.setDensity(10);
  //world.add(g);
  
  
  ///* creation of large circle shape */
  //e                   = new FCircle(5);
  //e.setPosition(5, 3);
  //e.setFill(random(255), random(255), random(255));
  //e.setDensity(18); //60g/cm2
  //world.add(e);
  
  
  /* creation of blob shape, warning may slow down simulation */
  f                   = new FBlob();
  f.setAsCircle(50);
  f.setStroke(0);
  file.play();
  f.setPosition(200,200);
  f.setStrokeWeight(2);
  f.setFill(255);
  f.setStatic(false);
  f.setFriction(15);
  f.setDensity(1);
  //f.setDensity(30);
  f.setFill(random(255), random(255), random(255));
  world.add(f);
  
  
  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
 
  
  /* If you are developing on a Mac users must update the path below 
   * from "../img/Haply_avatar.png" to "./img/Haply_avatar.png" 
   */
  haplyAvatar = loadImage("../img/Haply_avatar.png"); 
  haplyAvatar.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1)));
  s.h_avatar.attachImage(haplyAvatar); 
  file.play();


  /* world conditions setup */
 // world.setGravity((0.0), (1000.0)); //1000 cm/(s^2)
  //world.setGravity((0.0), (0.0));
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  file.play();
  
  
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
      //posEE.set(device_to_graphics(posEE)); 
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    
    
    s.updateCouplingForce();
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
          /* haptic wall force calculation */
      fWall.set(0, 0);
      
      penWall.set(0, (posWall.y - (posEE.y + rEE)));
      
      if(penWall.y < 0){
        fWall = fWall.add(penWall.mult(-kWall));  
      }
      
      fEE = (fWall.copy()).mult(-1);
      fEE.set(graphics_to_device(fEE));
      /* end haptic wall force calculation */
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
    
    
    renderingForce = false;
    world.step(1.0f/1000.0f);
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/
PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}
/* end helper functions section ****************************************************************************************/
