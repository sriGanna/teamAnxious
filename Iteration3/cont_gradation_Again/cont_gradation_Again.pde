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

/* text font */
PFont             f;

/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

float             kWall                               = 200;
PVector           fWall                               = new PVector(0, 0);
PVector           penWall                             = new PVector(0, 0);
PVector           posWall                             = new PVector(0.01, 0.10);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           posEELast                           = new PVector(0, 0);
PVector           fEE                                = new PVector(0, 0); 

float             rEE                                 = 0.006;
/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 35.0;  
float             worldHeight                         = 25.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;


/* Initialization of wall */
FBox              wall, wallPortal12, wallPortal11, wallHidePortal1, wallPortal5, wall4, wallPortal4, wallPortal3, wallHidePortal5, wallPortal2, wall9, wallPortal120; 
FCircle           circle1, circle2;
FBlob             blob1;

FBox[][] walls=new FBox[35][25];

/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar, pac2;

/* end elements definition *********************************************************************************************/

float threshold = 20;



/* Timer variables */
long currentMillis = millis();
long previousMillis = 0;
float interval = 5000;

float x=random(35);
float y=random(22.5);

/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */

  /* screen size definition */
  size(1400, 900);

  /* device setup */
  f                   = createFont("Arial", 30, true);

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



  //moved, one way 
  //wallPortal3                  = new FBox(2,3);
  //wallPortal3.setPosition(18, 5);
  //wallPortal3.setFill(255,0,0);
  ////wallPortal4.setDensity(800);
  //wallPortal3.setNoStroke();
  //wallPortal3.setSensor(true);
  //wallPortal3.setStatic(true);
  //world.add(wallPortal3); 


  for (int i=0; i<35; i++) {
    for (int j=0; j<25; j++) {
      walls[i][j] = new FBox(1, 1);
      walls[i][j].setPosition(i, j);
      //if (i>0 && i<=15 && j>0 && j<=13) {
      //  walls[i][j].setFill(0,0,0);
      //}
      //if (i>15 && i<35 && j>0 && j<=13) {
      //  walls[i][j].setFill(255,0,0);
      //}
      //if (i>15 && i<35 && j>13 && j<25) {
      //  walls[i][j].setFill(0,255,0);
      //}
      //if (i>0 && i<=15 && j>13 && j<25) {
      //  walls[i][j].setFill(0,0,255);
      //}
      //walls[i][j].setFill(random(255), random(255), random(255));
      walls[i][j].setNoStroke();
      walls[i][j].setNoFill();
      walls[i][j].setSensor(true);
      walls[i][j].setStatic(true);
      world.add(walls[i][j]);
    }
  }

  


  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setNoStroke();
  //s.h_avatar.setFill(255, 255, 255);
  s.h_avatar.setSensor(false);
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

  /* world conditions setup */
  world.setGravity((0.0), (0.0)); //1000 cm/(s^2)
  //world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);

  background(255);

  world.draw();


  /* setup framerate speed */
  frameRate(baseFrameRate);


  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw() {
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if (renderingForce == false) {
    //background(0);
    //world.setFill(color(0,0,0));
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */

    renderingForce = true;

    if (haplyBoard.data_available()) {
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


    ////angry
    //if (s.h_avatar.isTouchingBody(wallPortal3)){   
    //  s.h_avatar.adjustPosition(18,9);  
    //}
    //else {
    ////s.h_avatar.setVelocity(0,70);
    //s.h_avatar.setDamping(700);

    //for(int i=0;i<20;i++) {
    //   s.h_avatar.setVelocity(x,y);
    //   x=random(35);
    //   y=random(22.5);
    //   print(x,y);
    //}
    //}

    //retry1
    //ArrayList a = s.h_avatar.getTouching();

    //s.h_avatar.setDamping(700);
    //if (s.h_avatar.isTouchingBody(wallPortal3)) {   
    //s.h_avatar.setVelocity(50, 0); 
    //delay(1000);
    //s.h_avatar.setVelocity(0,50); 
    //delay(1000);
    //s.h_avatar.setVelocity(50,0); 
    //delay(1000);
    //s.h_avatar.adjustPosition(18,21); 
    //s.h_avatar.adjustPosition(18,25); 
    //s.h_avatar.adjustPosition(18,29); 
    //s.h_avatar.adjustPosition(18,34); 
    //s.h_avatar.adjustPosition(18,40);
    //}

    //try2
    for (int i=0; i<35; i++) {
      for (int j=0; j<25; j++) {
        if (s.h_avatar.isTouchingBody(walls[i][j])) {
          s.h_avatar.setDamping(600);
          if (i>0 && i<=20 && j>0 && j<=9) {
            s.h_avatar.setVelocity(50, 0);
            s.h_avatar.setFill(random(255),random(255),random(255));
          }
          else if (i>15 && i<35 && j>0 && j<=20) {
            s.h_avatar.setVelocity(0, 50);
            s.h_avatar.setFill(random(255),random(255),random(255));
          }
          else if (i>11 && i<35 && j>13 && j<25) {
            s.h_avatar.setVelocity(-50, 0);
            s.h_avatar.setFill(random(255),random(255),random(255));
          }
          else if (i>0 && i<=15 && j>8 && j<25) {
            s.h_avatar.setVelocity(0, -50);
            s.h_avatar.setFill(random(255),random(255),random(255));
          }
          //if(i>0 && i<35 && j>0 && j<25)
          //{
          //  //int randomX=int(random(-9,10));
          //  //int randomY=int(random(-9,10));
          //  //int c=0;
          //  int c=int(random(0,4));
          //  if(c==0) {
          //    s.h_avatar.setVelocity(50,0);
          //  }
          //  else if(c==1) {
          //    s.h_avatar.setVelocity(0,50);
          //  }
          //  else if(c==2) {
          //    s.h_avatar.setVelocity(-50,0);
          //  }
          //  else if(c==3) {
          //    s.h_avatar.setVelocity(0,-50);
          //  }
          //  //print(randomX, randomY);
            
          //}
          //else {
          //  randomX=0;
          //  randomY=0;
          //}
        }
      }
    }
    
    ////try3
    //currentMillis = millis();
    //if (currentMillis - previousMillis > interval) {
    //      s.h_avatar.setVelocity(50,0);
    //      previousMillis = currentMillis; 
    //}

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();

    world.step(1.0f/1000.0f);

    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/
PVector device_to_graphics(PVector deviceFrame) {
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}

PVector graphics_to_device(PVector graphicsFrame) {
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}
/* end helper functions section ****************************************************************************************/
