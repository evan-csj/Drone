//import class to set up serial connection with wiring board
import processing.serial.*;
Serial port;
//button setup
color currentcolor, buttoncolor, highlight;
SqButton up, down, left, right, forward, backward, center, stop;
RectButton r1u, r1d, r2u, r2d, r3u, r3d, r4u, r4d, pgu, igu, dgu, pgd, igd, dgd;
boolean locked = false;
String string = "";
PFont font;

void setup() {
   //set up window
   size(800, 560);
   font = createFont("FRADM.TTF", 24);
   textFont(font);
   color baseColor = color(200, 200, 200);
   currentcolor = baseColor;
   // List all the available serial ports in the output pane.
   // You will need to choose the port that the Wiring board is
   // connected to from this list. 
   println(Serial.list());
   // Open the port that the Wiring board is connected to (in this case 1
   // which is the second open port in the array)
   // Make sure to open the port at the same speed Wiring is using (9600bps)
   if(Serial.list()[0].equals("COM1")){
     port = new Serial(this, Serial.list()[4], 115200);
   }else{
     port = new Serial(this, Serial.list()[3], 115200);
   }
   
   port.bufferUntil('~');
   
   // Define and create rectangle button UP
   int x = 10;
   int y = 10;
   int size = 100;
   int xsize, ysize;
   buttoncolor = color(50, 215, 61);
   highlight = color(38, 172, 47);
   up = new SqButton(x, y, size, buttoncolor, highlight);
   
   // Define and create rectangle button DOWN
   x = 10;
   y = 120;
   size = 100;
   buttoncolor = color(255, 153, 51);
   highlight = color(255, 128, 0);
   down = new SqButton(x, y, size, buttoncolor, highlight);
   
   // Define and create rectangle button STOP
   x = 10;
   y = 230;
   size = 100;
   buttoncolor = color(223, 43, 43);
   highlight = color(174, 43, 43);
   stop = new SqButton(x, y, size, buttoncolor, highlight);
   
   // Define and create rectangle button LEFT
   x = 120;
   y = 120;
   size = 100;
   buttoncolor = color(150, 150, 150);
   highlight = color(100, 100, 100);
   left = new SqButton(x, y, size, buttoncolor, highlight);
   
   // Define and create rectangle button right
   x = 340;
   y = 120;
   size = 100;
   buttoncolor = color(150, 150, 150);
   highlight = color(100, 100, 100);
   right = new SqButton(x, y, size, buttoncolor, highlight);
   
   // Define and create rectangle button FORWARD
   x = 230;
   y = 10;
   size = 100;
   buttoncolor = color(150, 150, 150);
   highlight = color(100, 100, 100);
   forward = new SqButton(x, y, size, buttoncolor, highlight);
   
   // Define and create rectangle button BACKWARD
   x = 230;
   y = 230;
   size = 100;
   buttoncolor = color(150, 150, 150);
   highlight = color(100, 100, 100);
   backward = new SqButton(x, y, size, buttoncolor, highlight);
   
   // Define and create rectangle button CENTER
   x = 230;
   y = 120;
   size = 100;
   buttoncolor = color(150, 150, 150);
   highlight = color(100, 100, 100);
   center = new SqButton(x, y, size, buttoncolor, highlight);
   
   // Rotor 1
   x = 120;
   y = 10;
   xsize = 100;
   ysize = 50;
   buttoncolor = color(50, 215, 61);
   highlight = color(38, 172, 47);
   r1u = new RectButton(x, y, xsize, ysize, buttoncolor, highlight);
   
   y = 60;
   buttoncolor = color(255, 153, 51);
   highlight = color(255, 128, 0);
   r1d = new RectButton(x, y, xsize, ysize, buttoncolor, highlight);
   
   // Rotor 2
   x = 340;
   y = 10;
   xsize = 100;
   ysize = 50;
   buttoncolor = color(50, 215, 61);
   highlight = color(38, 172, 47);
   r2u = new RectButton(x, y, xsize, ysize, buttoncolor, highlight);
   
   y = 60;
   buttoncolor = color(255, 153, 51);
   highlight = color(255, 128, 0);
   r2d = new RectButton(x, y, xsize, ysize, buttoncolor, highlight);
   
   // Rotor 4
   x = 120;
   y = 230;
   xsize = 100;
   ysize = 50;
   buttoncolor = color(50, 215, 61);
   highlight = color(38, 172, 47);
   r4u = new RectButton(x, y, xsize, ysize, buttoncolor, highlight);
   
   y = 280;
   buttoncolor = color(255, 153, 51);
   highlight = color(255, 128, 0);
   r4d = new RectButton(x, y, xsize, ysize, buttoncolor, highlight);
   
   // Rotor 3
   x = 340;
   y = 230;
   xsize = 100;
   ysize = 50;
   buttoncolor = color(50, 215, 61);
   highlight = color(38, 172, 47);
   r3u = new RectButton(x, y, xsize, ysize, buttoncolor, highlight);
   
   y = 280;
   buttoncolor = color(255, 153, 51);
   highlight = color(255, 128, 0);
   r3d = new RectButton(x, y, xsize, ysize, buttoncolor, highlight);
   
   // P Gain
   x = 120;
   y = 340;
   xsize = 100;
   ysize = 50;
   buttoncolor = color(153, 153, 255);
   highlight = color(102, 102, 255);
   pgu = new RectButton(x, y, xsize, ysize, buttoncolor, highlight);
   
   // I Gain
   x = 230;
   y = 340;
   xsize = 100;
   ysize = 50;
   buttoncolor = color(153, 204, 255);
   highlight = color(102, 178, 255);
   igu = new RectButton(x, y, xsize, ysize, buttoncolor, highlight);
   
   // D Gain
   x = 340;
   y = 340;
   xsize = 100;
   ysize = 50;
   buttoncolor = color(153, 255, 255);
   highlight = color(102, 255, 255);
   dgu = new RectButton(x, y, xsize, ysize, buttoncolor, highlight);
   
   // P Gain
   x = 120;
   y = 390;
   xsize = 100;
   ysize = 50;
   buttoncolor = color(255, 153, 204);
   highlight = color(255, 102, 178);
   pgd = new RectButton(x, y, xsize, ysize, buttoncolor, highlight);
   
   // I Gain
   x = 230;
   y = 390;
   xsize = 100;
   ysize = 50;
   buttoncolor = color(255, 153, 255);
   highlight = color(255, 102, 255);
   igd = new RectButton(x, y, xsize, ysize, buttoncolor, highlight);
   
   // D Gain
   x = 340;
   y = 390;
   xsize = 100;
   ysize = 50;
   buttoncolor = color(204, 153, 255);
   highlight = color(178, 102, 255);
   dgd = new RectButton(x, y, xsize, ysize, buttoncolor, highlight);
}
 
void draw() {
   background(currentcolor);
   fill(0, 0, 0);
   text(string,450,0,350,500);
   stroke(255);
   strokeWeight(2);
   
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
   
   drawArrow(280,90,60,-90);
   drawArrow(360,170,60,0);
   drawArrow(280,250,60,90);
   drawArrow(200,170,60,180);
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
      if(up.pressed()) { //ON button
         currentcolor = up.basecolor;
         port.write('A');
      } else if(down.pressed()) { //OFF button
         currentcolor = down.basecolor;
         port.write('V');
      } else if(left.pressed()) {
         currentcolor = left.basecolor;
         port.write('L');
      } else if(right.pressed()) {
         currentcolor = right.basecolor;
         port.write('R');
      } else if(forward.pressed()) {
         currentcolor = forward.basecolor;
         port.write('F');
      } else if(backward.pressed()) {
         currentcolor = backward.basecolor;
         port.write('B');
      } else if(center.pressed()) {
         currentcolor = center.basecolor;
         port.write('C');
      } else if(stop.pressed()) {
         currentcolor = stop.basecolor;
         port.write('S');
      }else if(r1u.pressed()) {
         currentcolor = r1u.basecolor;
         port.write(11);
      }else if(r1d.pressed()) {
         currentcolor = r1d.basecolor;
         port.write(10);
      }else if(r2u.pressed()) {
         currentcolor = r2u.basecolor;
         port.write(21);
      }else if(r2d.pressed()) {
         currentcolor = r2d.basecolor;
         port.write(20);
      }else if(r3u.pressed()) {
         currentcolor = r3u.basecolor;
         port.write(31);
      }else if(r3d.pressed()) {
         currentcolor = r3d.basecolor;
         port.write(30);
      }else if(r4u.pressed()) {
         currentcolor = r4u.basecolor;
         port.write(41);
      }else if(r4d.pressed()) {
         currentcolor = r4d.basecolor;
         port.write(40);
      }else if(pgu.pressed()) {
         currentcolor = pgu.basecolor;
         port.write('P');
      }else if(igu.pressed()) {
         currentcolor = igu.basecolor;
         port.write('I');
      }else if(dgu.pressed()) {
         currentcolor = dgu.basecolor;
         port.write('D');
      }else if(pgd.pressed()) {
         currentcolor = pgd.basecolor;
         port.write('p');
      }else if(igd.pressed()) {
         currentcolor = igd.basecolor;
         port.write('i');
      }else if(dgd.pressed()) {
         currentcolor = dgd.basecolor;
         port.write('d');
      }
      delay(200);
   }
}

void serialEvent(Serial port) {
  string = port.readString();
  if(string == null){
    println("null serial string");
    string = "";
  }
}

void drawArrow(int cx, int cy, int len, float angle){
  pushMatrix();
  translate(cx, cy);
  rotate(radians(angle));
  stroke(255);
  strokeWeight(6);
  line(0,0,len, 0);
  line(len, 0, len - 8, -8);
  line(len, 0, len - 8, 8);
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
