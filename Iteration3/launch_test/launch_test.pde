import controlP5.*;
ControlP5 cp5, cp6, cp7, cp8;

void setup() {
  size(550, 75);
  smooth();
  cp5 = new ControlP5(this);
  cp6 = new ControlP5(this);
  cp7 = new ControlP5(this);
  cp8 = new ControlP5(this);
  PFont p = createFont("Verdana", 17); 
  ControlFont font = new ControlFont(p);

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
}

void draw() { 
  // draw() must be present for mousePressed() to work
}

//void mousePressed() {
//  //  PrintWriter output=null;
//  //  output = createWriter("myfile.bat");
//  //  output.println("cd C:\\Users\\rbree\\OneDrive\\Documents\\GitHub\\teamAnxious\\Iteration3\\launch_test\\application.windows64\\");
//  //  output.println("squish.exe");
//  //  output.flush();
//  //  output.close();
//  //  output=null;
//  //  //launch
//  //  //println("Opening Process_4");
//  //  //launch("bubble.png");
//  //  //launch(".\application.windows64\slingshot_Unma.exe");
//  //  //String path = dataPath("C:/Users/rbree/OneDrive/Documents/GitHub/teamAnxious/Iteration3/launch_test");
//  launch(sketchPath("")+"myfile.bat");
//  //  //launch(path+"/application.windows64/slingshot_Unma.exe");
//  //  //launch("C:\Users\Lakshmi\Documents\GitHub\teamAnxious\Iteration3\launch_test\application.windows64\slingshot_Unma.exe");
//  //  //println("launched");
//}

public void Squish() {
  PrintWriter output=null;
  output = createWriter("myfile.bat");
  output.println("cd C:\\Users\\Lakshmi\\Documents\\GitHub\\teamAnxious\\Iteration3\\launch_test\\application.windows64\\");
  output.println("squish.exe");
  output.flush();
  output.close();
  output=null;
  //launch(sketchPath("")+"myfile.bat");
}

public void Slingshot() {
  PrintWriter output=null;
  output = createWriter("myfile.bat");
  output.println("cd C:\\Users\\Lakshmi\\Documents\\GitHub\\teamAnxious\\Iteration3\\launch_test\\application.windows64\\");
  output.println("slingshot_Unma.exe");
  output.flush();
  output.close();
  output=null;
  //launch(sketchPath("")+"myfile.bat");
}

public void Popped() {
  PrintWriter output=null;
  output = createWriter("myfile.bat");
  output.println("cd C:\\Users\\Lakshmi\\Documents\\GitHub\\teamAnxious\\Iteration3\\launch_test\\application.windows64\\");
  output.println("paintball_explode_multi.exe");
  output.flush();
  output.close();
  output=null;
  //launch(sketchPath("")+"myfile.bat");
}

public void Guided() {
  PrintWriter output=null;
  output = createWriter("myfile.bat");
  output.println("cd C:\\Users\\Lakshmi\\Documents\\GitHub\\teamAnxious\\Iteration3\\launch_test\\application.windows64\\");
  output.println("cont_gradation_Again.exe");
  output.flush();
  output.close();
  output=null;
  //launch(sketchPath("")+"myfile.bat");
}

void controlEvent(CallbackEvent event) {
  if (event.getAction() == ControlP5.ACTION_CLICK) {
    switch(event.getController().getAddress()) {
    case "/Squish":
      Squish();
      launch(sketchPath("")+"myfile.bat");
      break;
    case "/Slingshot":
      Slingshot();
      launch(sketchPath("")+"myfile.bat");
      break;
    case "/Popped":
      Popped();
      launch(sketchPath("")+"myfile.bat");
      break;
    }
  }
}
