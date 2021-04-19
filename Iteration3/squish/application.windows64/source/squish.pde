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
public final int NUM_PALETTES = 10;
public final float PALETTE_SPACER = 1.5-1; //space between palette elements
public final float BUTTON_SPACER = 1.25*1.5; //space between GUI elements


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
float             worldWidth                          = 30.0;   //35
float             worldHeight                         = 20.0;  //25

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;


/* Initialization of interative shapes */
FCircle           c, c2;
FPoly             t;
FCircle           g;
FCircle           e;
FBlob             f, f2;
FBox              menu;
public int actNum = 0;
int colR, colG, colB;

/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar;
PGraphics    blobs;

ControlP5 cp5, cp6, cp7, cp8;


FBox[] colorSwatch = new FBox[6];
ArrayList<ColorPalette> palettes;
ColorPalette selected=null;
int shade=0;
int paletteIndex;

/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */
  //file = new SoundFile(this, "squish.mp3");
  /* screen size definition */
  size(1200, 800);

  smooth();
  cp5 = new ControlP5(this);
  cp6 = new ControlP5(this);
  cp7 = new ControlP5(this);
  cp8 = new ControlP5(this);
  //cp7 = new ControlP5(this);

  PFont p = createFont("Verdana", 17); 
  ControlFont font = new ControlFont(p);

  // change the original colors
  cp5.setColorForeground(0xffaa0000);
  cp5.setColorBackground(color(255, 0, 0));
  cp5.setFont(font);
  cp5.setColorActive(0xffff0000);

  // change the original colors
  cp6.setColorForeground(0xffaa0000);
  cp6.setColorBackground(color(0, 0, 255));
  cp6.setFont(font);
  cp6.setColorActive(0xffff0000);

  // change the original colors
  cp7.setColorForeground(0xffaa0000);
  cp7.setColorBackground(color(0, 255, 0));
  cp7.setFont(font);
  cp7.setColorActive(0xffff0000);

  cp8.setColorForeground(0xffaa0000);
  cp8.setColorBackground(color(0, 255, 0));
  cp8.setFont(font);
  cp8.setColorActive(0xffff0000);


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
  cp5.addButton("Return")
    .setValue(0)
    .setPosition(1075, 300)
    .setSize(90, 30)
    ;
  cp6.addButton("Save")
    .setValue(0)
    .setPosition(1075, 340)
    .setSize(90, 30)
    ;
  cp7.addButton("Next")
    .setValue(0)
    .setPosition(1075, 100)
    .setSize(90, 30)
    ;

  cp7.addButton("Prev")
    .setValue(0)
    .setPosition(1075, 60)
    .setSize(90, 30)
    ;

  //blobs = createGraphics(1200, 1200, JAVA2D);

  /* creation of blob shape, warning may slow down simulation */
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


  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setNoStroke();
  s.h_avatar.setFill(0, 0, 0);
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 


  ///* If you are developing on a Mac users must update the path below 
  // * from "../img/Haply_avatar.png" to "./img/Haply_avatar.png" 
  // */
  //haplyAvatar = loadImage("../img/Haply_avatar.png"); 
  //haplyAvatar.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1)));
  //s.h_avatar.attachImage(haplyAvatar); 


  /* world conditions setup */
  // world.setGravity((0.0), (1000.0)); //1000 cm/(s^2)
  //world.setGravity((0.0), (0.0));
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  background(255);

  createMenu();
  createPalettes();
  createMenu();
  paletteIndex = 0;
  float x = createColorPicker(palettes.get(paletteIndex)) - BUTTON_SPACER;
  float y = edgeBottomRightY - 1.5;
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
    //background(255);
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
      //posEE.set(device_to_graphics(posEE));
    }

    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 


    s.updateCouplingForce();
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons



    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    if (s.h_avatar.isTouchingBody(c) || s.h_avatar.isTouchingBody(c2)) {
      s.h_avatar.setDamping(800);
      //file.play();
    } else {
      s.h_avatar.setDamping(0);
    }
    selected=palettes.get(paletteIndex);
    shade=int(random(6));
    setDrawingColor(selected.getSwatch(shade).getColor());

    renderingForce = false;
    world.step(1.0f/1000.0f);
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/
public void Return(int theValue) {
  if (actNum>0) {
    printPath("launch_test.pde");
    launch(sketchPath("")+"myfile.bat");
    delay(500);
    exit();
  }
  actNum++;
  println(actNum);
}

public void Save(int theValue) {
  if (actNum >0) {
    save("./saved/test.png");
  }
  actNum++;
  println(actNum);
  //s.h_avatar.setFill(0,0,255);
}

public void Next(int theValue) {
  if (actNum >0) {
    paletteIndex = (paletteIndex + 1) % (NUM_PALETTES);
    updateColorPicker(palettes.get(paletteIndex));
  }
  actNum++;
  println(actNum);
  //s.h_avatar.setFill(0,0,255);
}

public void Prev(int theValue) {
  if (actNum >0) {
    paletteIndex = (paletteIndex - 1 ) % (NUM_PALETTES);
    if (paletteIndex < 0) {
      paletteIndex = NUM_PALETTES - 1;
    }
    updateColorPicker(palettes.get(paletteIndex));
  }
  actNum++;
  println(actNum);
  //s.h_avatar.setFill(0,0,255);
}

//public void Green(int theValue) {
//  s.h_avatar.setFill(0, 255, 0);
//}

void printPath(String app) {
  PrintWriter output=null;
  output = createWriter("myfile.bat");
  output.print("cd ");
  // output.println(myPath);
  String myPath = sketchPath("");
  String newPath = myPath.substring(0, myPath.lastIndexOf('\\'));
  newPath = newPath.substring(0, newPath.lastIndexOf('\\'));
  newPath = newPath.substring(0, newPath.lastIndexOf('\\')); // uncomment when exporting!!
  output.print(newPath);
  output.println("\\launch_test\\application.windows64\\");
  output.println("launch_test.exe");
  //output.println(app);
  output.flush();
  output.close();
  output=null;
}


void createMenu() {

  menu              = new FBox(4, 20);
  menu.setFill(100, 100, 100);
  menu.setPosition(28, 10);
  menu.setStatic(true);
  world.add(menu);
}


//palettes
void createPalettes() {
  palettes = new ArrayList<ColorPalette>();
  for (int i=0; i< NUM_PALETTES; i++) {
    palettes.add(createPalette(i)); //add all defined palettes
  }
}

ColorPalette createPalette(int index) {
  ColorSwatch[] palette = new ColorSwatch[6];
  switch(index) {
    case(9): //pastel og
    palette[5] = new ColorSwatch(255, 166, 158, 5); //pink
    palette[4] = new ColorSwatch(250, 243, 221, 4); //yellow
    palette[3] = new ColorSwatch(184, 242, 230, 3); //green
    palette[2] = new ColorSwatch(205, 168, 230, 2); //purple
    palette[1] = new ColorSwatch(153, 196, 224, 1); //blue
    palette[0] = new ColorSwatch(94, 100, 114, 0); //grey
    break;
    case(8): //pastel but make it fun
    palette[5] = new ColorSwatch(155, 140, 237, 5); //purple
    palette[4] = new ColorSwatch(235, 226, 134, 4); //yellow
    palette[3] = new ColorSwatch(232, 104, 147, 3); //pink
    palette[2] = new ColorSwatch(255, 149, 138, 2); //orange
    palette[1] = new ColorSwatch(126, 222, 204, 1); //green
    palette[0] = new ColorSwatch(103, 182, 219, 0); //blue
    break;
    case(7): //depressed cherry blossoms
    palette[5] = new ColorSwatch(234, 191, 203, 5); //pink
    palette[4] = new ColorSwatch(193, 145, 161, 4); //brown pink
    palette[3] = new ColorSwatch(95, 010, 135, 3); //purple
    palette[2] = new ColorSwatch(47, 0, 79, 2); //angry purple
    palette[1] = new ColorSwatch(164, 80, 139, 1); //deep pink
    palette[0] = new ColorSwatch(26, 020, 035, 0); //dead purple
    break;
    case(6): //a field of tulips that you'll never see because you never get out of your house
    palette[5] = new ColorSwatch(211, 063, 073, 5); //red
    palette[4] = new ColorSwatch(221, 255, 247, 4); //blue
    palette[3] = new ColorSwatch(147, 184, 073, 3); //green
    palette[2] = new ColorSwatch(234, 214, 055, 2); //yellow
    palette[1] = new ColorSwatch(65, 60, 80, 1); //blue
    palette[0] = new ColorSwatch(38, 39, 48, 0); //black
    break;
    case(5): //party balloons at the party you never go to
    palette[5] = new ColorSwatch(0, 071, 119, 5); //blue
    palette[4] = new ColorSwatch(163, 000, 000, 4); //red
    palette[3] = new ColorSwatch(255, 119, 000, 3); //orange
    palette[2] = new ColorSwatch(239, 210, 141, 2); //beige
    palette[1] = new ColorSwatch(0, 175, 181, 1); //blue
    palette[0] = new ColorSwatch(255, 249, 79, 0); //yellow
    break;
    case(4): //daisies but in high saturation
    palette[5] = new ColorSwatch(251, 97, 7, 5); //orange
    palette[4] = new ColorSwatch(243, 222, 44, 4); //yellow
    palette[3] = new ColorSwatch(124, 181, 24, 3); //green slime
    palette[2] = new ColorSwatch(31, 39, 07, 2); //black like my heart
    palette[1] = new ColorSwatch(92, 128, 1, 1); //moss green
    palette[0] = new ColorSwatch(251, 176, 45, 0); //if orange and yellow had a kid
    break;
    case(3): //tie dye shirt gone wrong
    palette[5] = new ColorSwatch(255, 102, 102, 5); //pink
    palette[4] = new ColorSwatch(204, 255, 102, 4); //green or yellow
    palette[3] = new ColorSwatch(93, 046, 140, 3); //purple
    palette[2] = new ColorSwatch(46, 196, 182, 2); //teal not cyan
    palette[1] = new ColorSwatch(241, 232, 184, 1); //beige again
    palette[0] = new ColorSwatch(004, 004, 003, 0); //the void
    break;
    case(2): //daisies but they're all dead
    palette[5] = new ColorSwatch(241, 247, 237, 5); //eggshell
    palette[4] = new ColorSwatch(36, 062, 054, 4); //moss
    palette[3] = new ColorSwatch(124, 169, 130, 3); //dead green
    palette[2] = new ColorSwatch(224, 238, 198, 2); //light green
    palette[1] = new ColorSwatch(194, 168, 062, 1); //yellow
    palette[0] = new ColorSwatch(56, 29, 42, 0); //brown
    break;
    case(1): //that one namib desert photo
    palette[5] = new ColorSwatch(255, 210, 117, 5); //sand
    palette[4] = new ColorSwatch(232, 174, 104, 4); //dark sand
    palette[3] = new ColorSwatch(165, 127, 96, 3); //even darker sand
    palette[2] = new ColorSwatch(227, 165, 135, 2); //flesh
    palette[1] = new ColorSwatch(219, 90, 60, 1); //sand but tanned
    palette[0] = new ColorSwatch(66, 066, 066, 0); //demonic black
    break;
    case(0): //every monday blues in 2020
    palette[5] = new ColorSwatch(3, 026, 107, 5); //blue
    palette[4] = new ColorSwatch(2, 19, 79, 4); //also blue
    palette[3] = new ColorSwatch(105, 108, 194, 3); //still blue
    palette[2] = new ColorSwatch(182, 235, 252, 2); //more blue
    palette[1] = new ColorSwatch(5, 178, 220, 1); //it's all blue
    palette[0] = new ColorSwatch(82, 126, 183, 0); //red. lol nope
    break;
  default:  //pastel rainbow
    palette[5] = new ColorSwatch(155, 140, 237, 5); //purple
    palette[4] = new ColorSwatch(235, 226, 134, 4); //yellow
    palette[3] = new ColorSwatch(232, 104, 147, 3); //pink
    palette[2] = new ColorSwatch(255, 149, 138, 2); //orange
    palette[1] = new ColorSwatch(126, 222, 204, 1); //green
    palette[0] = new ColorSwatch(103, 182, 219, 0); //blue
    break;
  }

  return new ColorPalette(palette);
}


//check color
void checkChangeColor() {
  ColorPalette palette = palettes.get(paletteIndex);
  for (int i=0; i<palette.getLength(); i++) {
    //if (colorSwatch[i].isTouchingBody(s.h_avatar)) {
    //    setDrawingColor(palette.getSwatch(i).getColor());
    //}
  }
}


void setDrawingColor(int r, int g, int b) {
  colR = r;
  colG = g;
  colB = b;
  s.h_avatar.setFill(colR, colG, colB);
}

void setDrawingColor(int[] rgb) {
  setDrawingColor(rgb[0], rgb[1], rgb[2]);
}

void updateColorPicker(ColorPalette palette) {
  ColorSwatch swatch;
  for (int i=0; i<palette.getLength(); i++) {
    swatch = palette.getSwatch(i);
    colorSwatch[i].setFillColor(color(swatch.getRed(), swatch.getGreen(), swatch.getBlue()));
    world.draw();
  }
}

float createColorPicker(ColorPalette palette) {
  float x = 25.2+2.8;
  float y = 3.2;
  ColorSwatch swatch;
  for (Integer i=0; i< 6; i++) {
    y = y + PALETTE_SPACER;
    colorSwatch[i] = new FBox(2, 0.4);
    colorSwatch[i].setPosition(x, y);
    colorSwatch[i].setStatic(true);
    colorSwatch[i].setSensor(true);
    colorSwatch[i].setName(i.toString());

    swatch = palette.getSwatch(i);
    //print(swatch);
    colorSwatch[i].setFillColor(color(swatch.getRed(), swatch.getGreen(), swatch.getBlue()));
    //print("here "+swatch.getRed());
    world.add(colorSwatch[i]);

    //world.draw();
  }

  return x;
}  

/* end helper functions section ****************************************************************************************/
