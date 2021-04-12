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
float             worldWidth                          = 32.0;  
float             worldHeight                         = 21.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;


/* Initialization of wall */
FBox[][]              walls=new FBox[4][12];
FBox wall;
float wallW = 0.6;

float w1=4;
float w2=10.2;
float w3=15;

FCircle           circle2;


/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar;


//curve elements
float beginX = 20.0;  // Initial x-coordinate
float beginY = 10.0;  // Initial y-coordinate
float endX = 570.0;   // Final x-coordinate
float endY = 320.0;   // Final y-coordinate
float distX;          // X-axis distance to move
float distY;          // Y-axis distance to move
float exponent = 4;   // Determines the curve
float x = 0.0;        // Current x-coordinate
float y = 0.0;        // Current y-coordinate
float step = 0.01;    // Size of each step along the path
float pct = 0.0;      // Percentage traveled (0.0 to 1.0)


/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */

  /* screen size definition */
  size(1280, 820);
  
  //noStroke();
  distX = endX - beginX;
  distY = endY - beginY;

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


  float wx1=7.18;
  float wx2=23.2;
  float wy1=3;
  float wy2=19;

  createWall(wx1, false, 0);
  resetW();
  createWall(wy1, true, 1);
  resetW();
  createWall(wx2, false, 2);
  resetW();
  createWall(wy2, true, 3);

  for (int i=0; i<4; i++)
  {
    for (int j=0; j<12; j++)
    {
      walls[i][j].setStatic(true);
      walls[i][j].setFill(0, 255, 255);
      walls[i][j].setNoStroke();
      walls[i][j].setForce(0, 0);
      world.add(walls[i][j]);
    }
  }   



  //circle
  circle2                   = new FCircle(1);
  circle2.setPosition(12, 7);
  circle2.setStatic(false);
  circle2.setFill(255, 0, 0);
  world.add(circle2);



  print(edgeTopLeftX+worldWidth/2.0);
  print(" ,");
  print(edgeTopLeftY+2*worldHeight/3.0);



  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4);  
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+5); 


  /* If you are developing on a Mac users must update the path below 
   * from "../img/Haply_avatar.png" to "./img/Haply_avatar.png" 
   */
  haplyAvatar = loadImage("../img/Haply_avatar.png"); 
  haplyAvatar.resize((int)(hAPI_Fisica.worldToScreen(1))/2, (int)(hAPI_Fisica.worldToScreen(1))/2);
  s.h_avatar.attachImage(haplyAvatar); 


  /* world conditions setup */
  world.setGravity((0.0), (10.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(0.4);
  world.setEdgesFriction(1.8);


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
    background(255);
    fill(0, 2);
    rect(0, 0, width, height);
    pct += step;
    if (pct < 1.0) {
      x = beginX + (pct * distX);
      y = beginY + (pow(pct, exponent) * distY);
    }
    fill(255);
    ellipse(x, y, 20, 20);
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

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();

    world.step(1.0f/1000.0f);

    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/
void createWall(float w, boolean isHorizontal, int wallNumber) {
  for (int j=0; j<12; j++)
  {
    if (isHorizontal==true && j<4)
    {
      walls[wallNumber][j]=new FBox(wallW+0.2, 1);
      walls[wallNumber][j].setPosition(w1+4.2, w);      
      w1=w1+1.6;
    } else if (isHorizontal==true && j>=4 && j<8)
    {
      walls[wallNumber][j]=new FBox(wallW, 1);
      walls[wallNumber][j].setPosition(w2+4.2, w);     
      w2=w2+1.2;
    } else if (isHorizontal==true && j>=8 && j<12)
    {
      walls[wallNumber][j]=new FBox(wallW-0.2, 1);
      walls[wallNumber][j].setPosition(w3+4.2, w);      
      w3=w3+1;
    } else if (isHorizontal==false && j<4)
    {
      walls[wallNumber][j]=new FBox(1, wallW+0.2);
      walls[wallNumber][j].setPosition(w, w1);      
      //println(w,w1);
      w1=w1+1.6;
    } else if (isHorizontal==false && j>=4 && j<8)
    {
      walls[wallNumber][j]=new FBox(1, wallW);
      walls[wallNumber][j].setPosition(w, w2);      
      //println(w,w2);
      w2=w2+1.2;
    } else if (isHorizontal==false && j>=8 && j<12)
    {
      walls[wallNumber][j]=new FBox(1, wallW-0.2);
      walls[wallNumber][j].setPosition(w, w3);     
      //println(w,w3);
      w3=w3+1;
    }    
  }
}

void resetW() {
   w1=4;
   w2=10.2;
   w3=15;
   //print("here" + w1+" "+w2+" "+w3);
}

void mousePressed() {
  pct = 0.0;
  beginX = x;
  beginY = y;
  endX = mouseX;
  endY = mouseY;
  distX = endX - beginX;
  distY = endY - beginY;
}
/* end helper functions section ****************************************************************************************/
