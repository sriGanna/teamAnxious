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
import controlP5.*;
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
float             worldWidth                          = 27.0;  
float             worldHeight                         = 20.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;


/* Initialization of interative shapes */
FCircle           c, c2;
FPoly             t;
FCircle           ge;
FCircle           e;
FBlob             f, f2;
FBox              menu;


/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar;
public PGraphics         blobs;
ControlP5 cp5,cp6, cp7;

/* end elements definition *********************************************************************************************/ 



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  //file = new SoundFile(this, "squish.mp3");
  /* screen size definition */
  size(1200,800);
  PGraphics blobs  = createGraphics(1200, 800);
   smooth();
  cp5 = new ControlP5(this);
  cp6 = new ControlP5(this);


  PFont p = createFont("Verdana", 17); 
  ControlFont font = new ControlFont(p);

  // change the original colors
 
  cp5.setColorForeground(0xffaa0000);
  cp5.setColorBackground(color(255,0,0));
  cp5.setFont(font);
  cp5.setColorActive(0xffff0000);
  
    // change the original colors
  cp6.setColorForeground(0xffaa0000);
  cp6.setColorBackground(color(0,0,255));
  cp6.setFont(font);
  cp6.setColorActive(0xffff0000);
  
  

  
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
  
 
    //buttons
  cp5.addButton("Save")
    .setValue(0)
    .setPosition(1075,  110)
    .setSize(90, 30)
    ;
  cp6.addButton("Return")
    .setValue(0)
    .setPosition(1075, 170 )
    .setSize(90, 30)
    ;

blobs.beginDraw();
    f                   = new FBlob();
  //f.setAsCircle(16, 7, 20, 70);
  f.setAsCircle(25, 20, 21, 70);
  f.setStroke(0);
  f.setStrokeWeight(2);
  //f.setFill(255);
  f.setStatic(true);
  f.setFriction(20);
  f.setDensity(100);
  f.setSensor(true);
  f.setFill(random(255), random(255), random(255));
  world.add(f);
  
    f2                   = new FBlob();
  //f.setAsCircle(16, 7, 20, 70);
  f2.setAsCircle(10, 20, 21, 70);
  f2.setStroke(0);
  f2.setStrokeWeight(2);
  //f.setFill(255);
  f2.setStatic(true);
  f2.setFriction(20);
  f2.setDensity(100);
  f2.setSensor(true);
  f2.setFill(random(255), random(255), random(255));
  world.add(f2);
  
  c                   = new FCircle(20.0);
  c.setPosition(edgeTopLeftX+worldWidth/1.3-3, edgeTopLeftY+2*worldHeight/6.0+11);
  c.setStatic(true);
  c.setSensor(true);
  c.setNoFill();
  c.setNoStroke();
  world.add(c);
  
    c2                   = new FCircle(22.0);
  c2.setPosition(edgeTopLeftX+worldWidth/1.3-16, edgeTopLeftY+2*worldHeight/6.0+12);
  c2.setStatic(true);
  c2.setSensor(true);
  c2.setNoFill();
  c2.setNoStroke();
  world.add(c2);
  blobs.endDraw();
  createObjects();
  createMenu();
  
  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setNoStroke();
  s.h_avatar.setFill(0,0,0);
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

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
    //background(255);
    //image(blobs, 0, 0);
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
    

    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    if (s.h_avatar.isTouchingBody(c) || s.h_avatar.isTouchingBody(c2)){
      s.h_avatar.setDamping(800);
      //file.play();
        
    }else{
      s.h_avatar.setDamping(0);
    }
    
    
    renderingForce = false;
    world.step(1.0f/1000.0f);
  }
}
/* end simulation section **********************************************************************************************/

void Save(){
  //blobs.clear();
  save("./saved/test.png");//
  
}

void Return(){
   printPath("launch_test.pde");
//      launch(sketchPath("")+"myfile.bat");
//      delay(500);
//      exit();
  
}
/* helper functions section, place helper functions here ***************************************************************/
//public void Save(int theValue) {
//  s.h_avatar.setFill(255,0,0);
//}

//public void Return(int theValue) {
//  printPath("launch_test.pde");
//      launch(sketchPath("")+"myfile.bat");
//      delay(500);
//      exit();
//}

/* end helper functions section ****************************************************************************************/


void createMenu(){
  
  menu              = new FBox(4, 20);
  menu.setFill(100,100,100);
  menu.setPosition(28,10);
  menu.setStatic(true);
  world.add(menu);
  
}

void printPath(String app) {
  PrintWriter output=null;
  output = createWriter("myfile.bat");
  output.print("cd ");
  // output.println(myPath);
  String myPath = sketchPath("");
  String newPath = myPath.substring(0, myPath.lastIndexOf('\\'));
  newPath = newPath.substring(0, newPath.lastIndexOf('\\'));
  //newPath = newPath.substring(0, newPath.lastIndexOf('\\')); // uncomment when exporting!!
  output.print(newPath);
  output.println("\\launch_test\\application.windows64\\");
  output.println("launch_test.exe");
  //output.println(app);
  output.flush();
  output.close();
  output=null;
}

//void controlEvent(CallbackEvent event) {
//  if (event.getAction() == ControlP5.ACTION_CLICK) {
//    switch(event.getController().getAddress()) {
//    case "/Save":
//      save("./saved/test.png");//
//      break;
//    case "/Return":
//      printPath("launch_test.pde");
//      launch(sketchPath("")+"myfile.bat");
//      delay(500);
//      exit();
//      break;
//    }
//  }
//}

void createObjects(){
  
  
}
