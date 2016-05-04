/*
 * This script controls the movement of a three-wheeled omniwheel robot powered by an ESP8266 board from http://www.wemos.cc/
 * 
 *            wheel 1   
 *              __   
 *             /  \        
 *            /    \    
 *  wheel 3  \------/ wheel 2
 * 
 * Change the wheel control pins using the motorXForwardPin constants
 * 
 * The ESP8266 is configured as a wireless access point with the address "ESP8266 Thing xxxx" where xxxx is the last 4 digits of the MAC address.
 * Connect to the robot by going to http://192.168.4.1/x/y where x and y are numbers from -255 to 255.
 * x and y are the end points of a directional vector starting at 0,0.
 * The robot moves in the direction of the vector at a speed determined by the vector magnitude.
 * The robot will continue to move until another command is sent.
 * Send 0,0 to stop.
 * Send 0,255 to move full speed in the direction of wheel 1
 * Send 0,-255 to move backwards full speed
 * 
 */
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library
#include <math.h>

WiFiServer server(80);
const char WiFiAPPSK[] = "foo";

// Motor control pins
// Motor 1
const int motor1ForwardPin = D3; 
const int motor1ReversePin = D4;
// Motor 2
const int motor2ForwardPin = D5;
const int motor2ReversePin = D6;
// Motor 3
const int motor3ForwardPin = D7;
const int motor3ReversePin = D8;

// forwardPin, reversePin: motor pins
// motorDirection: true = forward, false = reverse
// motorSpeed: motor speed from 0-255
void moveMotor(int forwardPin, int reversePin, boolean motorDirection, byte motorSpeed) 
{
  if (motorDirection) {
    Serial.println("Moving setting pin " + String(reversePin) + " LOW");
    analogWrite(reversePin, LOW);
    Serial.println("Moving setting pin " + String(forwardPin) + " to " + String(motorSpeed));
    analogWrite(forwardPin, motorSpeed);
  } else {
    Serial.println("Moving setting pin " + String(forwardPin) + " LOW");
    analogWrite(forwardPin, LOW);
    Serial.println("Moving setting pin " + String(reversePin) + " to " + String(motorSpeed));
    analogWrite(reversePin, motorSpeed);
  }
}

// Move in the direction of x, y for x, y [-255 to 255]
// x, y represents the endpoint of a directional vector starting at 0,0. 
// The magnitude of the vector determines the speed.
// The robot will continue in the given direction and speed until given further instruction.
// Send 0,0 to stop the wheels.
// This function is modified from http://makezine.com/projects/make-40/kiwi/
void moveToDirection(int x, int y)
{
  // Convert x,y to a direction vector
  float theta = atan2(y, x);
  float magnitude = sqrt((x*x)+(y*y));
  
  if(magnitude > 0.0f){  
    
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(" y: ");
    Serial.print(y);
  
    Serial.print(" Angle: ");
    Serial.print(degrees(theta));
  
    Serial.print(" Magnitude: ");
    Serial.print(magnitude);  
    
    float vx = magnitude * cos(theta);
    float vy = magnitude * sin(theta);
    const float sqrt3o2 = 1.0*sqrt(3)/2;
    //STEP 3. Find wheel vecotrs
    float w0 = -vx;                   // v dot [-1, 0] / 25mm
    float w1 = 0.5*vx - sqrt3o2 * vy; // v dot [1/2, -sqrt(3)/2] / 25mm
    float w2 = 0.5*vx + sqrt3o2 * vy; // v dot [1/2, +sqrt(3)/2] / 25mm
   
    boolean w0_ccw = w0 < 0 ? true : false;
    boolean w1_ccw = w1 < 0 ? true : false;
    boolean w2_ccw = w2 < 0 ? true : false;
    byte w0_speed = (byte) abs(w0);
    byte w1_speed = (byte) abs(w1);
    byte w2_speed = (byte) abs(w2);
   
    Serial.print(" vx: ");
    Serial.print(vx);
    Serial.print(" vy: ");
    Serial.print(vy);
    
    Serial.print(" w0: ");
    Serial.print(w0_speed);
    if(w0_ccw) Serial.print(" CCW"); else Serial.print(" CW");
    Serial.print(" w1: ");
    Serial.print(w1_speed);
    if(w1_ccw) Serial.print(" CCW"); else Serial.print(" CW");
    Serial.print(" w2: ");
    Serial.print(w2_speed);
    if(w2_ccw) Serial.print(" CCW"); else Serial.print(" CW");  
    Serial.println();
    
    moveMotor(motor1ForwardPin, motor1ReversePin, w0_ccw, w0_speed);
    moveMotor(motor2ForwardPin, motor2ReversePin, w1_ccw, w1_speed);
    moveMotor(motor3ForwardPin, motor3ReversePin, w2_ccw, w2_speed);
  }
  else{
    moveMotor(motor1ForwardPin, motor1ReversePin, true, 0);
    moveMotor(motor2ForwardPin, motor2ReversePin, true, 0);
    moveMotor(motor3ForwardPin, motor3ReversePin, true, 0);
  }
}

// This function stolen from https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/example-sketch-ap-web-server
void setupWiFi()
{
  WiFi.mode(WIFI_AP);

  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "Thing-":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "ESP8266 Thing " + macID;

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i=0; i<AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  WiFi.softAP(AP_NameChar, WiFiAPPSK);
}

void setup() {
  Serial.begin(9600);
  Serial.print("Hello World!");

  setupWiFi();
  server.begin();
  
  pinMode(motor1ForwardPin, OUTPUT);
  pinMode(motor1ReversePin, OUTPUT);
  pinMode(motor2ForwardPin, OUTPUT);
  pinMode(motor2ReversePin, OUTPUT);
  pinMode(motor3ForwardPin, OUTPUT);
  pinMode(motor3ReversePin, OUTPUT);
  
  analogWrite(motor1ForwardPin, LOW);
  analogWrite(motor1ReversePin, LOW);
  analogWrite(motor2ForwardPin, LOW);
  analogWrite(motor2ReversePin, LOW);
  analogWrite(motor3ForwardPin, LOW);
  analogWrite(motor3ReversePin, LOW);
}

void loop()
{
  /* 
   *  Server code stolen from https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/example-sketch-ap-web-server
   */
  // TODO: Convert this to accept JSON with lists of commands
  // TODO: Add commands to tell the robot to rotate
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  
  // Read the first line of the request
  String req = client.readStringUntil('\r');
  Serial.println(req);
  client.flush();

  if (req.indexOf("favicon.ico") > 0)
    return;

  // TODO: return JSON
  // Match the request
  // Request format: GET /x/y HTTP/1.1 where x and y are integer values from 1000 to 2000
  int val = -1; // We'll use 'val' to keep track of both the
                // request type (read/set) and value if set.
  int xIndex, yIndex;
  String xIn, yIn;
  xIndex = req.indexOf("/");
  req = req.substring(xIndex + 1);
  yIndex = req.indexOf("/");
  xIn = req.substring(0, yIndex);
  xIndex = req.indexOf(" ");
  yIn = req.substring(yIndex + 1, xIndex);
  
  int x = xIn.toInt();
  int y = yIn.toInt();
 
  client.flush();

  // Prepare the response. Start with the common header:
  String s = "HTTP/1.1 200 OK\r\n";
  s += "Content-Type: text/html\r\n\r\n";
  s += "<!DOCTYPE HTML>\r\n<html>\r\n";
  s += "X=[" + xIn + "], Y=[" + yIn + "]";
  s += "REQ=[" + req + "]";
  s += "</html>\n";

  // Send the response to the client
  client.print(s);
  delay(1);
  Serial.println("Client disonnected");

  // The client will actually be disconnected 
  // when the function returns and 'client' object is detroyed

  /* 
   *  END stolen Server code
   */

  // Move in the direction and speed (magnitude) of vector from 0,0 to x,y
  moveToDirection(x, y);
}

