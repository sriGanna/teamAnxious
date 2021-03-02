/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
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
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/
/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* end effector radius in meters */
float             rEE                                 = 0.006;

/* virtual wall parameter  */
float             kWall                               = 450;
PVector           fWall                               = new PVector(0, 0);
PVector           penWall                             = new PVector(0, 0);
PVector           posWall                             = new PVector(0.01, 0.10);


/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

final int         worldPixelWidth                     = 1000;
final int         worldPixelHeight                    = 650;

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;

/* define maze blocks */


FBox[] walls = new FBox[28];
PShape wall;

float wallW = 0.6;


/* define start and stop button */
FCircle           c1;
FCircle           c2;
FCircle           c3;



/* define game start */
boolean           gameStart                           = false;
int               mode                                =0;

/* text font */
PFont             f;

/* end elements definition *********************************************************************************************/  



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 400);
  
  /* set font type and size */
  f                   = createFont("Arial", 16, true);

  
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
  
  
  
  
  
  
  /* Mode 1 Drawings */
  for (int i = 0; i <10; i++){
     walls[i]                  = new FBox(1, wallW);
     
     walls[i].setStatic(true);
    walls[i].setFill(0, 0, 0);
    walls[i].setNoStroke();
    world.add(walls[i]);
    
  }
    for (int i = 10; i <28; i++){
     walls[i]                  = new FBox(wallW,1);
     
     walls[i].setStatic(true);
    walls[i].setFill(0, 0, 0);
    walls[i].setNoStroke();
    world.add(walls[i]);
    
  }
  walls[0].setPosition(11.18, 4);

  walls[1].setPosition(11.18, 4.9);

  walls[2].setPosition(18.8, 5.9);

  walls[3].setPosition(18.8, 6.9);

  walls[4].setPosition(18.8, 7.9);

  walls[5].setPosition(18.8, 4);

  walls[6].setPosition(18.8, 4.9);

  walls[7].setPosition(11.18, 5.9);

  walls[8].setPosition(11.18, 6.9);

  walls[9].setPosition(11.18, 7.9);

  walls[10].setPosition(11, 3);

  walls[11].setPosition(12, 3);

  walls[12].setPosition(13, 3);

  walls[13].setPosition(14, 3);

  walls[14].setPosition(15, 3);

  walls[15].setPosition(16, 3);

  walls[16].setPosition(17, 3);

  walls[17].setPosition(18, 3);

  walls[18].setPosition(19, 3);

  walls[19].setPosition(11, 9);

  walls[20].setPosition(12, 9);

  walls[21].setPosition(13, 9);

  walls[22].setPosition(14, 9);

  walls[23].setPosition(15, 9);

  walls[24].setPosition(16, 9);

  walls[25].setPosition(17, 9);

  walls[26].setPosition(18, 9);

  walls[27].setPosition(19, 9);

   /* End of Mode 1 Drawings */ 
   
  /* Insert Drawings for Mode 2 Here */ 
   deviceOrigin.add(worldPixelWidth/2, 0);
  
  /* create pantagraph graphics */
    wall = create_wall(posWall.x-0.2, posWall.y+rEE, posWall.x+0.2, posWall.y+rEE);
  wall.setStroke(color(0));
  /* End of Mode 2 Drawings */ 
  
  /* Insert Drawings for Mode 3 Here */ 
  
  /* End of Mode 3 Drawings */ 
   
   
 
  /* Mode 1 Button */
  c1                  = new FCircle(1.0); // diameter is 2
  c1.setPosition(edgeTopLeftX+1.5, edgeTopLeftY+worldHeight/2.0-3);
  c1.setFill(0, 255, 0);
  c1.setStaticBody(true);
  c1.setSensor(true);
  world.add(c1);
  
  /* Mode 2 Button */
  c2                  = new FCircle(1.0);
  c2.setPosition(edgeTopLeftX+3, edgeTopLeftY+worldHeight/2.0-3);
  c2.setFill(200,0,0);
  c2.setStaticBody(true);
  c2.setSensor(true);
  world.add(c2);
  
  /* Mode 3 Button */
  c3                  = new FCircle(1.0);
  c3.setPosition(edgeTopLeftX+4.5, edgeTopLeftY+worldHeight/2.0-3);
  c3.setFill(0,0,200);
  c3.setStaticBody(true);
  c3.setSensor(true);
  world.add(c3);
  
  
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(255,0,0); 
  s.h_avatar.setSensor(true);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  

 
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
    textFont(f, 22);
 
    if(mode ==1){
      fill(0, 0, 0);
      textAlign(CENTER);
      text("Touch a coloured circle to switch modes", width/2, 90);
      // mode 1 drawings visible 
      for(int i=0;i<28;i++){
       walls[i].setFill(0);
      }
      // all other mode drawings invisible
      for(int i=0;i<28;i++){
        walls[i].setFill(255,0,255);
      }
    
  
    }
    else if(mode ==2){
    }
    else if(mode ==3){
    }
    else{
      fill(128, 128, 128);
      textAlign(CENTER);
      text("Touch a coloured circle to switch modes", width/2, 60);
      
      for(int i=0;i<28;i++){
        walls[i].setFill(255,0,255);
      }
    
      
      
    }
  
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
    
    if (s.h_avatar.isTouchingBody(c1)){
      gameStart = true;
      mode =1;
      s.h_avatar.setSensor(false);
    }
    else if (s.h_avatar.isTouchingBody(c2)){
      
      mode =2;
      s.h_avatar.setSensor(false);
    }
    else if (s.h_avatar.isTouchingBody(c3)){
      
      mode =3;
      s.h_avatar.setSensor(false);
    }
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/
PShape create_wall(float x1, float y1, float x2, float y2){
  x1 = pixelsPerMeter * x1;
  y1 = pixelsPerMeter * y1;
  x2 = pixelsPerMeter * x2;
  y2 = pixelsPerMeter * y2;
  
  return createShape(LINE, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
}

 void contactStarted(FContact contact) {
   // Draw in green an ellipse where the contact took place
   fill(0, 170, 0);
   ellipse(contact.getX(), contact.getY(), 20, 20);
 }

 void contactPersisted(FContact contact) {
   // Draw in blue an ellipse where the contact took place
   fill(0, 0, 170);
   ellipse(contact.getX(), contact.getY(), 10, 10);
 }
