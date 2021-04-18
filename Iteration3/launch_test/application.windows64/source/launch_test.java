import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import controlP5.*; 
import java.awt.datatransfer.Clipboard; 
import java.awt.datatransfer.Transferable; 
import java.awt.datatransfer.DataFlavor; 
import java.awt.datatransfer.UnsupportedFlavorException; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class launch_test extends PApplet {








boolean DEBUG = true;


ControlP5 cp4, cp5, cp6, cp7, cp8;
Button bA, bB, bC;
Button sub;
Textfield sysPath; 
Textlabel title;
String myPath;
String a1=""; 
Boolean hideButton = false;

public void setup() {
  
  
  cp4 = new ControlP5(this);
  cp5 = new ControlP5(this);
  cp6 = new ControlP5(this);
  cp7 = new ControlP5(this);
  cp8 = new ControlP5(this);
  PFont p = createFont("Verdana", 17); 
  ControlFont font = new ControlFont(p);

  PFont p3 = createFont("Verdana", 12); 


  //cp4.addTextlabel("Title", "Anxious", 10, 10)
  //  .setColor(color(0, 0, 0))
  //  .setFont(p);
  //cp4.addTextlabel("Instructions", "Please copy and paste your path in the textbox", 10, 40)
  //  .setColor(color(0, 0, 0))
  //  .setFont(p);
  //cp4.addTextlabel("Example", "example: C:\\Users\\rbree\\OneDrive\\Documents\\GitHub\\teamAnxious\\Iteration3\\launch_test\\application.windows64\\", 10, 75 )
  //  .setColor(color(0, 0, 0))
  //  .setFont(p3);
  // change the original colors
  cp5.setColorForeground(0xffaa0000);
  cp5.setColorBackground(color(22, 100, 0));
  cp5.setFont(font);
  cp5.setColorActive(0xffff0000);

  // change the original colors
  cp6.setColorForeground(0xffaa0000);
  cp6.setColorBackground(color(65, 0, 255));
  cp6.setFont(font);
  cp6.setColorActive(0xffff0000);

  // change the original colors
  cp7.setColorForeground(0xffaa0000);
  cp7.setColorBackground(color(255, 22, 22));
  cp7.setFont(font);
  cp7.setColorActive(0xffff0000);

  // change the original colors
  cp8.setColorForeground(0xffaa0000);
  cp8.setColorBackground(color(255, 200, 0));
  cp8.setFont(font);
  cp8.setColorActive(0xffff0000);

  //buttons

  //sysPath = cp4.addTextfield("input")
  //  .setPosition(50, 130)
  //  .setSize(450, 50)
  //  .setFont(font)
  //  .setFocus(true)
  //  .setColor(color(255, 0, 0))
  //  ;

  //sub = cp4.addButton("Submit")
  //  .setValue(0)
  //  .setPosition(225, 200)
  //  .setSize(100, 30)
  //  ;
  //sub = cp4.addButton("Skip")
  //  .setValue(0)
  //  .setPosition(400, 200)
  //  .setSize(100, 30)
  //  ;

  cp5.addTextlabel("Description", "Please ensure your Haply is in the Home Position", 50, 200)
    .setColor(color(0, 0, 0))
    .setFont(p);

  cp5.addButton("Squish")
    .setValue(0)
    .setPosition(0, 0)
    .setSize(100, 30)
    ;

  cp6.addButton("Slingshot")
    .setValue(0)
    .setPosition(150, 0)
    .setSize(110, 30)
    ;
  cp7.addButton("Popped")
    .setValue(0)
    .setPosition(300, 0)
    .setSize(100, 30)
    ;
  cp8.addButton("Guided")
    .setValue(0)
    .setPosition(450, 0)
    .setSize(100, 30)
    ;
  a1 = GetTextFromClipboard();
}

public void draw() { 
  // draw() must be present for mousePressed() to work
  background(255);
}

public void controlEvent(CallbackEvent event) {
  if (event.getAction() == ControlP5.ACTION_CLICK) {
    switch(event.getController().getAddress()) {
    case "/Squish":

      printPath("\\squish\\application.windows64\\","squish,exe");
      println(sketchPath(""));

      launch(sketchPath("")+"myfile.bat"); //
      delay(500);
      exit();
      break;
    case "/Slingshot":

      printPath("\\slingshot_Unma\\application.windows64\\","slingshot_Unma.exe");
      launch(sketchPath("")+"myfile.bat");
      delay(500);
      exit();
      break;
    case "/Popped":
      //exit();
      printPath("\\paintball_explode_multi\\application.windows64\\","paintball_explode_multi.exe");
      launch(sketchPath("")+"myfile.bat");
      delay(500);
      exit();
      break;
    case "/Guided":
      //exit();
      printPath("\\cont_gradation_Again.exe\\application.windows64\\","cont_gradation_Again.exe");
      launch(sketchPath("")+"myfile.bat");
      delay(500);
      exit();
      break;
    }
  }
}

public void keyPressed() {
  if (key==' ') {
    a1 = GetTextFromClipboard ();
    sysPath.setText(a1);
  }
}

public void printPath(String path, String app) {
  PrintWriter output=null;
  output = createWriter("myfile.bat");
  output.print("cd ");
  // output.println(myPath);
  //output.print(sketchPath(""));
  String myPath = sketchPath("");
  String newPath = myPath.substring(0, myPath.lastIndexOf('\\'));
  newPath = newPath.substring(0, newPath.lastIndexOf('\\'));
  newPath = newPath.substring(0, newPath.lastIndexOf('\\'));
  //output.println("application.windows64\\");
  output.print(newPath);
  output.println(path); //
  output.println(app);
  output.flush();
  output.close();
  output=null;
}

public String GetTextFromClipboard () {
  String text = (String) GetFromClipboard(DataFlavor.stringFlavor);

  if (text==null) 
    return "";

  return text;
}

public Object GetFromClipboard (DataFlavor flavor) {

  Clipboard clipboard = getJFrame(getSurface()).getToolkit().getSystemClipboard();

  Transferable contents = clipboard.getContents(null);
  Object object = null; // the potential result 

  if (contents != null && contents.isDataFlavorSupported(flavor)) {
    try
    {
      object = contents.getTransferData(flavor);
      println ("Clipboard.GetFromClipboard() >> Object transferred from clipboard.");
    }

    catch (UnsupportedFlavorException e1) // Unlikely but we must catch it
    {
      println("Clipboard.GetFromClipboard() >> Unsupported flavor: " + e1);
      e1.printStackTrace();
    }

    catch (java.io.IOException e2)
    {
      println("Clipboard.GetFromClipboard() >> Unavailable data: " + e2);
      e2.printStackTrace() ;
    }
  }

  return object;
}
public static final javax.swing.JFrame getJFrame(final PSurface surf) {
  return
    (javax.swing.JFrame)
    ((processing.awt.PSurfaceAWT.SmoothCanvas)
    surf.getNative()).getFrame();
}

public void clearPathScreen() {
  cp5.show();
  cp6.show();
  cp7.show();
  cp8.show();
  //cp4.remove("input"); 
  //sub.hide();
  cp4.hide();
}

public void saveMyPath() {
  myPath = sysPath.getText();
  //myPath = myPath.replace("\", "\\\\");
  myPath = myPath.trim();
  myPath = myPath+("\\application.windows64\\");
  if (DEBUG) {
    println(myPath);
    println("C:\\Users\\lakshmi\\Documents\\GitHub\\teamAnxious\\Iteration3\\launch_test\\application.windows64\\");
  }
}
  public void settings() {  size(550, 250);  smooth(); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "launch_test" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
