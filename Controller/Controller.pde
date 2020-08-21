//import class to set up serial connection with wiring board
import processing.serial.*;
Serial port;
//button setup
color bg, buttoncolor, highlight, green, greenH, yellow, yellowH, red, redH, grey, greyH, blue, blueH, navy, navyH;
SqButton up, down, left, right, forward, backward, center, stop;
RectButton r1u, r1d, r2u, r2d, r3u, r3d, r4u, r4d, pgu, igu, dgu, pgd, igd, dgd;
boolean locked = false;
String string = "";
String stringUpdate = "";
String[] stringList;
String[] lineList;
boolean datalog = false;
PFont font;
int size = 120, xsize = size, ysize = xsize/2;
int space = size/10;
Table table;
TableRow newRow;

void setup() {
   //set up window
   size(1000, 800);
   font = createFont("FRADM.TTF", size*0.3);
   textFont(font);
   bg = color(224, 224, 224);
   // List all the available serial ports in the output pane.
   // You will need to choose the port that the Wiring board is
   // connected to from this list. 
   println(Serial.list());
   // Open the port that the Wiring board is connected to (in this case 1
   // which is the second open port in the array)
   // Make sure to open the port at the same speed Wiring is using (115200bps)
   if(Serial.list()[0].equals("COM1")){
     port = new Serial(this, Serial.list()[3], 115200);
   }else{
     port = new Serial(this, Serial.list()[2], 115200);
   }
   
   port.bufferUntil('~');
   
   // Color Def
   green = color(50, 215, 61);
   greenH = color(38, 172, 47);
   yellow = color(255, 153, 51);
   yellowH = color(255, 128, 0);
   red = color(223, 43, 43);
   redH = color(174, 43, 43);
   grey = color(192, 192, 192);
   greyH = color(160, 160, 160);
   blue = color(153, 204, 255);
   blueH = color(102, 178, 255);
   navy = color(153, 153, 255);
   navyH = color(102, 102, 255);
   
   // Control Panel
   up = new SqButton(space, space*2 + size, size, green, greenH);
   down = new SqButton(space, space*3 + size*2, size, yellow, yellowH);
   stop = new SqButton(space, space*4 + size*3, size, red, redH);
   left = new SqButton(space*2 + size, space*2 + size, size, grey, greyH);
   right = new SqButton(space*4 + size*3, space*2 + size, size, grey, greyH);
   forward = new SqButton(space*3 + size*2, space, size, grey, greyH);
   backward = new SqButton(space*3 + size*2, space*3 + size*2, size, grey, greyH);
   center = new SqButton(space*3 + size*2, space*2 + size, size, grey, greyH);
   
   // Rotor 1
   r1u = new RectButton(space*2 + size, space, xsize, ysize, green, greenH);
   r1d = new RectButton(space*2 + size, space + ysize, xsize, ysize, yellow, yellowH);
   
   // Rotor 2
   r2u = new RectButton(space*4 + size*3, space, xsize, ysize, green, greenH);
   r2d = new RectButton(space*4 + size*3, space + ysize, xsize, ysize, yellow, yellowH);
   
   // Rotor 4
   r4u = new RectButton(space*2 + size, space*3 + size*2, xsize, ysize, green, greenH);
   r4d = new RectButton(space*2 + size, space*3 + size*2 + ysize, xsize, ysize, yellow, yellowH);
   
   // Rotor 3
   r3u = new RectButton(space*4 + size*3, space*3 + size*2, xsize, ysize, green, greenH);
   r3d = new RectButton(space*4 + size*3, space*3 + size*2 + ysize, xsize, ysize, yellow, yellowH);
   
   // PID Gain
   pgu = new RectButton(space*2 + size, space*4 + size*3, xsize, ysize, navy, navyH);
   igu = new RectButton(space*3 + size*2, space*4 + size*3, xsize, ysize, navy, navyH);
   dgu = new RectButton(space*4 + size*3, space*4 + size*3, xsize, ysize, navy, navyH);
   pgd = new RectButton(space*2 + size, space*4 + size*3 + ysize, xsize, ysize, blue, blueH);
   igd = new RectButton(space*3 + size*2, space*4 + size*3 + ysize, xsize, ysize, blue, blueH);
   dgd = new RectButton(space*4 + size*3, space*4 + size*3 + ysize, xsize, ysize, blue, blueH);
   
   table = new Table();
   table.addColumn("Time");
   table.addColumn("Roll");
   table.addColumn("Pitch");
   table.addColumn("Yaw");
}
 
void draw() {
   background(bg);

   update(mouseX, mouseY);
   
   up.display();
   down.display();
   left.display();
   right.display();
   forward.display();
   backward.display();
   center.display();
   stop.display();
   r1u.display();
   r1d.display();
   r2u.display();
   r2d.display();
   r3u.display();
   r3d.display();
   r4u.display();
   r4d.display();
   pgu.display();
   igu.display();
   dgu.display();
   pgd.display();
   igd.display();
   dgd.display();
   
   drawArrow(space*3 + size*2 + size/2, space + size/5*4, size/5*3, -90);
   drawArrow(space*4 + size*3 + size/5, space*2 + size + size/2, size/5*3, 0);
   drawArrow(space*3 + size*2 + size/2, space*3 + size*2 + size/5, size/5*3, 90);
   drawArrow(space*2 + size*2 - size/5, space*2 + size + size/2, size/5*3, 180);
   
   fill(47, 47, 47);
   textAlign(LEFT, TOP);
   text(stringUpdate, space*5 + size*4, space, size*4, space*6 + size*5);
   textAlign(CENTER, CENTER);
   text("UP", space, space*2 + size, size, size);
   text("DOWN", space, space*3 + size*2, size, size);
   text("STOP", space, space*4 + size*3, size, size);
   text("R1", space*2 + size, space, size, size);
   text("R2", space*4 + size*3, space, size, size);
   text("R4", space*2 + size, space*3 + size*2, size, size);
   text("R3", space*4 + size*3, space*3 + size*2, size, size);
   text("P", space*2 + size, space*4 + size*3, size, size);
   text("I", space*3 + size*2, space*4 + size*3, size, size);
   text("D", space*4 + size*3, space*4 + size*3, size, size);
   stroke(255);
   strokeWeight(size/50);
}
 
void update(int x, int y) {
   if(locked == false) {
      up.update();
      down.update();
      left.update();
      right.update();
      forward.update();
      backward.update();
      center.update();
      stop.update();
      r1u.update();
      r1d.update();
      r2u.update();
      r2d.update();
      r3u.update();
      r3d.update();
      r4u.update();
      r4d.update();
      pgu.update();
      igu.update();
      dgu.update();
      pgd.update();
      igd.update();
      dgd.update();
   } else {
      locked = false;
   }
   //Turn LED on and off if buttons pressed where
   if(mousePressed) {
      if(up.pressed()) {
         port.write('A');
      } else if(down.pressed()) {
         port.write('V');
      } else if(left.pressed()) {
         port.write('L');
      } else if(right.pressed()) {
         port.write('R');
      } else if(forward.pressed()) {
         port.write('F');
      } else if(backward.pressed()) {
         port.write('B');
      } else if(center.pressed()) {
         port.write('C');
      } else if(stop.pressed()) {
         port.write('S');
      }else if(r1u.pressed()) {
         port.write(11);
      }else if(r1d.pressed()) {
         port.write(10);
      }else if(r2u.pressed()) {
         port.write(21);
      }else if(r2d.pressed()) {
         port.write(20);
      }else if(r3u.pressed()) {
         port.write(31);
      }else if(r3d.pressed()) {
         port.write(30);
      }else if(r4u.pressed()) {
         port.write(41);
      }else if(r4d.pressed()) {
         port.write(40);
      }else if(pgu.pressed()) {
         port.write('P');
      }else if(igu.pressed()) {
         port.write('I');
      }else if(dgu.pressed()) {
         port.write('D');
      }else if(pgd.pressed()) {
         port.write('p');
      }else if(igd.pressed()) {
         port.write('i');
      }else if(dgd.pressed()) {
         port.write('d');
      }
      delay(200);
   }
}

void serialEvent(Serial port) {
  string = port.readString();
  if(!datalog){
    datalog = true;
  }else{
    stringUpdate = "";
    stringList = split(string, " ");
    stringUpdate += "Time: ";
    stringUpdate += stringList[0];
    stringUpdate += " ms\n";
    stringUpdate += "RPY: ";
    stringUpdate += stringList[1];
    stringUpdate += " deg\n";
    stringUpdate += "R1234: ";
    stringUpdate += stringList[2];
    stringUpdate += "\n";
    stringUpdate += "PID: ";
    stringUpdate += stringList[3];
    stringUpdate += "\n";
    stringUpdate += "PWM: ";
    stringUpdate += stringList[4];
    stringUpdate += "\n";
    stringUpdate += "Voltage: ";
    stringUpdate += stringList[5];
    stringUpdate += " V\n";
    stringUpdate += "Loop Time: ";
    stringUpdate += stringList[6];
    stringUpdate += " us\n";
    
    newRow = table.addRow();
    newRow.setString("Time", stringList[0]);
    lineList = split(stringList[1], ',');
    newRow.setString("Roll", lineList[0]);
    newRow.setString("Pitch", lineList[1]);
    newRow.setString("Yaw", lineList[2]);
    newRow.setString("Voltage", stringList[5]);
  }
  
  if(string == null){
    println("null serial string");
    string = "";
  }
}

void drawArrow(int cx, int cy, int len, float angle){
   pushMatrix();
   translate(cx, cy);
   rotate(radians(angle));
   stroke(80);
   strokeWeight(size/20);
   line(0,0,len, 0);
   line(len, 0, len - size/10, -size/10);
   line(len, 0, len - size/10, size/10);
   popMatrix();
}
 
class Button {
   int x, y;
   int size;
   color basecolor, highlightcolor;
   color currentcolor;
   boolean over = false;
   boolean pressed = false;
   void update() {
      if(over()) {
         currentcolor = highlightcolor;
      } else {
         currentcolor = basecolor;
      }
   }
   boolean pressed() {
      if(over) {
          bg = basecolor;
          locked = true;
          return true;
      } else {
          locked = false;
          return false;
      }
   }
   boolean over() {
      return true;
   }
   void display() {
   }
}
 
class SqButton extends Button {
   SqButton(int ix, int iy, int isize, color icolor, color ihighlight) {
      x = ix;
      y = iy;
      size = isize;
      basecolor = icolor;
      highlightcolor = ihighlight;
      currentcolor = basecolor;
   }
   boolean over() {
      if( overRect(x, y, size, size) ) {
         over = true;
         return true;
      } else {
         over = false;
         return false;
      }
    }
   void display() {
      stroke(255);
      fill(currentcolor);
      rect(x, y, size, size);
   }
}

class RectButton extends Button {
   int xsize, ysize;
   RectButton(int ix, int iy, int ixsize, int iysize, color icolor, color ihighlight) {
      x = ix;
      y = iy;
      xsize = ixsize;
      ysize = iysize;
      basecolor = icolor;
      highlightcolor = ihighlight;
      currentcolor = basecolor;
   }
   boolean over() {
      if( overRect(x, y, xsize, ysize) ) {
         over = true;
         return true;
       } else {
         over = false;
         return false;
       }
   }
   void display() {
      stroke(255);
      fill(currentcolor);
      rect(x, y, xsize, ysize);
   }
}

boolean overRect(int x, int y, int width, int heiguht) {
   if (mouseX >= x && mouseX <= x+width && mouseY >= y && mouseY <= y+heiguht) {
      return true;
   } else {
      return false;
   }
}

void keyPressed()
{
  //save as a table in csv format(data/table - data folder name table)
  saveTable(table, "data/TestResult.csv");
  port.stop();
  exit();
}
