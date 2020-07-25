//import class to set up serial connection with wiring board
import processing.serial.*;
Serial port;
//button setup
color currentcolor, buttoncolor, highlight;
RectButton up, down, left, right, forward, backward, center;
boolean locked = false;
String string = "";
PFont font;

void setup() {
   //set up window
   size(450, 400);
   font = createFont("FRADM.TTF", 36);
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
   port = new Serial(this, Serial.list()[0], 9600);
   port.bufferUntil('\n');
   // Define and create rectangle button UP
   int x = 10;
   int y = 10;
   int size = 100;
   buttoncolor = color(50, 215, 61);
   highlight = color(38, 172, 47);
   up = new RectButton(x, y, size, buttoncolor, highlight);
   
   // Define and create rectangle button DOWN
   x = 10;
   y = 230;
   size = 100;
   buttoncolor = color(223, 43, 43);
   highlight = color(174, 43, 43);
   down = new RectButton(x, y, size, buttoncolor, highlight);
   
   // Define and create rectangle button LEFT
   x = 120;
   y = 120;
   size = 100;
   buttoncolor = color(150, 150, 150);
   highlight = color(100, 100, 100);
   left = new RectButton(x, y, size, buttoncolor, highlight);
   
   // Define and create rectangle button RIGHT
   x = 340;
   y = 120;
   size = 100;
   buttoncolor = color(150, 150, 150);
   highlight = color(100, 100, 100);
   right = new RectButton(x, y, size, buttoncolor, highlight);
   
   // Define and create rectangle button FORWARD
   x = 230;
   y = 10;
   size = 100;
   buttoncolor = color(150, 150, 150);
   highlight = color(100, 100, 100);
   forward = new RectButton(x, y, size, buttoncolor, highlight);
   
   // Define and create rectangle button BACKWARD
   x = 230;
   y = 230;
   size = 100;
   buttoncolor = color(150, 150, 150);
   highlight = color(100, 100, 100);
   backward = new RectButton(x, y, size, buttoncolor, highlight);
   
   // Define and create rectangle button CENTER
   x = 230;
   y = 120;
   size = 100;
   buttoncolor = color(150, 150, 150);
   highlight = color(100, 100, 100);
   center = new RectButton(x, y, size, buttoncolor, highlight);
}
 
void draw() {
   background(currentcolor);
   fill(0, 0, 0);
   text(string,10,375);
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
   } else {
      locked = false;
   }
   //Turn LED on and off if buttons pressed where
   //H = on (high) and L = off (low)
   if(mousePressed) {
      if(up.pressed()) { //ON button
         currentcolor = up.basecolor;
         port.write('U');
      } else if(down.pressed()) { //OFF button
         currentcolor = down.basecolor;
         port.write('D');
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
      }
      delay(100);
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
 
class RectButton extends Button {
   RectButton(int ix, int iy, int isize, color icolor, color ihighlight) {
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
 
boolean overRect(int x, int y, int width, int height) {
   if (mouseX >= x && mouseX <= x+width && mouseY >= y && mouseY <= y+height) {
      return true;
   } else {
      return false;
   }
}
