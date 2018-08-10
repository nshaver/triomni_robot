////////////////////////////////////////////////////////////////
// wifi
////////////////////////////////////////////////////////////////
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
const char* host= "triomni";
const char* ssid = "yourssid";
const char* password = "yourwifipassword";
const char* apssid = "triomni";
const char* appassword = "a47a47a47a";
const int port = 9876; // and this port
MDNSResponder mdns;
ESP8266WebServer webserver(80);
ESP8266HTTPUpdateServer httpUpdater;
WiFiServer server(port);
WiFiClient client;

////////////////////////////////////////////////////////////////
// adafuit motor shield v2.3 - i2c
////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// drive motors
Adafruit_DCMotor *motor1=AFMS.getMotor(1);
Adafruit_DCMotor *motor2=AFMS.getMotor(2);
Adafruit_DCMotor *motor3=AFMS.getMotor(3);
int motor1_speed_offset=100;
int motor2_speed_offset=100;
int motor3_speed_offset=100;

unsigned int drivespeed=255;
int thisspeed=0;
int thisdir=0;
boolean connExists=false;
unsigned long reqid=0;
unsigned int ms_fr, ms_fl, ms_rl, ms_rr;
char drive[3];
int speed=0;
int xysep=0;
char tpxchar[10];	
char tpychar[10];	
int tpx=0;
int tpy=0;
int tpxbyte=0;
int tpybyte=0;
double tpangle;
double tpradius;
double fm1, fm2, fm3;
int fm1i, fm2i, fm3i;

//////////////////////////
// PIEZO output pin
//////////////////////////
#define PIEZO D3

//////////////////////////
// pixy i2c
//
// pinout of pixy io port (* = used for i2c)
// 1 2   MISO/RX   5V*
// 3 4   SCK       MOSI/TX
// 5 6   SCL*      GND
// 7 8   SS        GND
// 9 10  SDA*      GND*
//
//////////////////////////
#include <PixyI2C.h>  
PixyI2C pixy;

unsigned long nextPrint;
unsigned long thisMillis;
#define X_CENTER    160L
#define Y_CENTER    100L
#define RCS_MIN_POS     0L
#define RCS_MAX_POS     1000L
#define RCS_CENTER_POS	((RCS_MAX_POS-RCS_MIN_POS)/2)
char buf[50];
uint32_t lastBlockTime = 0;
int oldX, oldY, oldSignature;
int32_t size = 400;
int scanIncrement = (RCS_MAX_POS - RCS_MIN_POS) / 150;
uint32_t lastMove = 0;
uint32_t lastBlockCount=0;

boolean autonomous=false;
 
//---------------------------------------
// Servo Loop Class
// A Proportional/Derivative feedback
// loop for pan/tilt servo tracking of
// blocks.
// (Based on Pixy CMUcam5 example code)
//---------------------------------------
class ServoLoop {
	public:
		ServoLoop(int32_t proportionalGain, int32_t derivativeGain);
	 
		void update(int32_t error);
	 
		int32_t m_pos;
		int32_t m_prevError;
		int32_t m_proportionalGain;
		int32_t m_derivativeGain;
};

// ServoLoop Constructor
ServoLoop::ServoLoop(int32_t proportionalGain, int32_t derivativeGain) {
	m_pos = RCS_CENTER_POS;
	m_proportionalGain = proportionalGain;
	m_derivativeGain = derivativeGain;
	m_prevError = 0x80000000L;
}
 
// ServoLoop Update 
// Calculates new output based on the measured
// error and the current state.
void ServoLoop::update(int32_t error) {
	long int velocity;
	if (m_prevError!=0x80000000) {	
		velocity = (error*m_proportionalGain + (error - m_prevError)*m_derivativeGain)>>10;
 
		m_pos += velocity;
		if (m_pos>RCS_MAX_POS) {
			m_pos = RCS_MAX_POS; 
		} else if (m_pos<RCS_MIN_POS) {
			m_pos = RCS_MIN_POS;
		}
	}
	m_prevError = error;
}
 
ServoLoop panLoop(200, 200);  // Servo loop for pan
ServoLoop tiltLoop(150, 200); // Servo loop for tilt
 
//---------------------------------------
// Track blocks via the Pixy pan/tilt mech
// (based in part on Pixy CMUcam5 pantilt example)
//---------------------------------------
int TrackBlock(int blockCount) {
	int trackedBlock = 0;
	long maxSize = 0;
 
	boolean timeToPrint=false;
	boolean sufficientSizeBlockFound=false;

	for (int i = 0; i < blockCount; i++) {
		if (pixy.blocks[i].width>=50){
			sufficientSizeBlockFound=true;
			if ((oldSignature == 0) || (pixy.blocks[i].signature == oldSignature)) {
				long newSize = pixy.blocks[i].height * pixy.blocks[i].width;
				if (newSize > maxSize) {
					trackedBlock = i;
					maxSize = newSize;
				}
			}
		}
	}

	if (sufficientSizeBlockFound==false){
		trackedBlock=-1;
		return trackedBlock;
	}

	int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
	int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;
 
	panLoop.update(panError);
	tiltLoop.update(tiltError);
 
	pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
 
	oldX = pixy.blocks[trackedBlock].x;
	oldY = pixy.blocks[trackedBlock].y;
	oldSignature = pixy.blocks[trackedBlock].signature;
	return trackedBlock;
}
 
void FollowBlock(int trackedBlock) {
	int32_t followError = RCS_CENTER_POS - panLoop.m_pos;  // How far off-center are we looking now?
 
	// Size is the area of the object.
	// We keep a running average of the last 8.
	size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height; 
	size -= size >> 3;

	// send the x:y to the client
	sprintf(buf, "x %d\n", pixy.blocks[trackedBlock].x);
	client.write((const uint8_t*)buf, sizeof(buf));
	sprintf(buf, "y %d\n", pixy.blocks[trackedBlock].y);
	client.write((const uint8_t*)buf, sizeof(buf));
	sprintf(buf, "w %d\n", pixy.blocks[trackedBlock].width);
	client.write((const uint8_t*)buf, sizeof(buf));
 
	// Forward speed decreases as we approach the object (size is larger)
	int forwardSpeed = constrain(400 - (size/256), -100, 400);  
 
	// Steering differential is proportional to the error times the forward speed
	int32_t differential = (followError + (followError * forwardSpeed))>>8;
 
	// Adjust the left and right speeds by the steering differential.
	int leftSpeed = constrain(forwardSpeed + differential, -400, 400);
	int rightSpeed = constrain(forwardSpeed - differential, -400, 400);
 
	// And set the motor speeds
	//motors.setLeftSpeed(leftSpeed);
	//motors.setRightSpeed(rightSpeed);
	if (autonomous){
		// super basic steering for now
		if (forwardSpeed > 0){
			if (leftSpeed > rightSpeed*1.5){
				// spin right
				fSpin(125);
			} else if (rightSpeed > leftSpeed*1.5){
				// spin left
				fSpin(-125);
			} else {
				// go north
				fDrivePolar(0, map(forwardSpeed, 0, 400, 0, 255));
			}
		} else if (forwardSpeed<0) {
			// go south
			fDrivePolar(180, map(forwardSpeed, 0, -100, 0, 175));
		} else {
			// stop
			fDrivePolar(0, 0);
		}
	}
}
 
/* 
void ScanForBlocks()
{
	if (millis() - lastMove > 20)
	{
		lastMove = millis();
		panLoop.m_pos += scanIncrement;
		if ((panLoop.m_pos >= RCS_MAX_POS)||(panLoop.m_pos <= RCS_MIN_POS))
		{
			tiltLoop.m_pos = random(RCS_MAX_POS * 0.6, RCS_MAX_POS);
			scanIncrement = -scanIncrement;
			if (scanIncrement < 0)
			{
				//motors.setLeftSpeed(-250);
				//motors.setRightSpeed(250);
			}
			else
			{
				//motors.setLeftSpeed(+180);
				//motors.setRightSpeed(-180);
			}
			delay(random(250, 500));
		}
 
		pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
	}
}
*/

void ScanForBlocks() {
	// don't do this until you make sure it's nonblocking
}
 
////////////////////////////////////////////////////////////////
// roboremo
////////////////////////////////////////////////////////////////
const int chCount = 4; // 4 channels, you can add more if you have GPIOs :)
int chVal[] = {1500, 1500, 1500, 1500}; // default value (middle)

int usMin = 0; // min pulse width
int usMax = 255; // max pulse width

char cmd[100]; // stores the command chars received from RoboRemo
int cmdIndex;
unsigned long lastHeartbeatTime=0;
unsigned long aliveSentTime=0;

unsigned int loopcnt=0;

boolean cmdStartsWith(const char *st) { // checks if cmd starts with st
  for(int i=0; ; i++) {
    if(st[i]==0) return true;
    if(cmd[i]==0) return false;
    if(cmd[i]!=st[i]) return false;;
  }
  return false;
}

void beep(int pitch, int duration, int times) {
	for (int t=0; t<times; t++){
		for (int i = 0; i < duration; i++) {
			digitalWrite(PIEZO, HIGH);
			delayMicroseconds(pitch);
			digitalWrite(PIEZO, LOW);
			delayMicroseconds(pitch);
		}
		delay(100);
	}
}

void fDrivePolar(double thisangle, double thisradius){
	if (thisradius==0){
		// stop
		motor1->run(RELEASE);
		motor2->run(RELEASE);
		motor3->run(RELEASE);

		// done
		return;
	}

	// get each motor value using cosine of motor offset - angle in radians
	fm1=thisradius * cos((150.0 - thisangle) * 71 / 4068);
	fm2=thisradius * cos((30.0 - thisangle ) * 71 / 4068);
	fm3=thisradius * cos((270.0 - thisangle) * 71 / 4068);

	// convert output values to integers
	fm1i=(int) fm1;
	fm2i=(int) fm2;
	fm3i=(int) fm3;

	if (fm1i>0){
		motor1->run(FORWARD);
	} else if (fm1i<0){
		motor1->run(BACKWARD);
	} else {
		motor1->run(RELEASE);
	}
	motor1->setSpeed(abs(fm1i)*motor1_speed_offset/100);

	if (fm2i>0){
		motor2->run(FORWARD);
	} else if (fm2i<0){
		motor2->run(BACKWARD);
	} else {
		motor2->run(RELEASE);
	}
	motor2->setSpeed(abs(fm2i)*motor2_speed_offset/100);

	if (fm3i>0){
		motor3->run(FORWARD);
	} else if (fm3i<0){
		motor3->run(BACKWARD);
	} else {
		motor3->run(RELEASE);
	}
	motor3->setSpeed(abs(fm3i)*motor3_speed_offset/100);
}

void fDrive(int angle, int thisspeed){
	// angle 0=north, 180=south, 90=east, 270=west
	if (thisspeed==0){
		// stop
		motor1->run(RELEASE);
		motor2->run(RELEASE);
		motor3->run(RELEASE);
	} else if (angle==0){
		// north
		motor1->run(BACKWARD);
		motor1->setSpeed((thisspeed*motor1_speed_offset)/100);
		motor2->run(FORWARD);
		motor2->setSpeed((thisspeed*motor2_speed_offset)/100);
		motor3->run(RELEASE);
	} else if (angle==180) {
		// south
		motor1->run(FORWARD);
		motor1->setSpeed((thisspeed*motor1_speed_offset)/100);
		motor2->run(BACKWARD);
		motor2->setSpeed((thisspeed*motor2_speed_offset)/100);
		motor3->run(RELEASE);
	} else if (angle==90) {
		// east
		motor1->run(FORWARD);
		motor1->setSpeed((thisspeed*motor1_speed_offset*.67)/100);
		motor2->run(FORWARD);
		motor2->setSpeed((thisspeed*motor2_speed_offset*.33)/100);
		motor3->run(BACKWARD);
		motor3->setSpeed((thisspeed*motor3_speed_offset)/100);
	} else if (angle==270) {
		// west
		motor1->run(BACKWARD);
		motor1->setSpeed((thisspeed*motor1_speed_offset*.67)/100);
		motor2->run(BACKWARD);
		motor2->setSpeed((thisspeed*motor2_speed_offset*.33)/100);
		motor3->run(FORWARD);
		motor3->setSpeed((thisspeed*motor3_speed_offset)/100);
	}
}

void fSpin(int thisspeed){
	// thisspeed full right = 255, full left = -255
	if (thisspeed==0){
		// stop
		motor1->run(RELEASE);
		motor2->run(RELEASE);
		motor3->run(RELEASE);
	} else if (thisspeed > 0) {
		// spin right
		motor1->run(FORWARD);
		motor1->setSpeed((thisspeed*motor1_speed_offset)/100);
		motor2->run(FORWARD);
		motor2->setSpeed((thisspeed*motor2_speed_offset)/100);
		motor3->run(FORWARD);
		motor3->setSpeed((thisspeed*motor3_speed_offset)/100);
	} else if (thisspeed < 0) {
		// spin left
		thisspeed=thisspeed*-1;
		motor1->run(BACKWARD);
		motor1->setSpeed((thisspeed*motor1_speed_offset)/100);
		motor2->run(BACKWARD);
		motor2->setSpeed((thisspeed*motor2_speed_offset)/100);
		motor3->run(BACKWARD);
		motor3->setSpeed((thisspeed*motor3_speed_offset)/100);
	}
}

// process commands received from roboremo app
void exeCmd() {
  if(cmdStartsWith("tpr")){
		// trackpad released, stop motors
		fDrivePolar(0, 0);
	} else if(cmdStartsWith("tpp") || cmdStartsWith("tpd")){
		// received something like:
		// tpp 15 230  // pressed
		// tdp 1 1     // dragged
		// find the space that separates the x and the y
		// extract the x:y into integer variables tpx and tpy
		xysep=0;
		tpx=0;
		tpy=0;
		tpxbyte=0;
		tpybyte=0;

		memset(tpxchar, 0, sizeof(tpxchar));
		memset(tpychar, 0, sizeof(tpychar));

		for (int x=4; x<=cmdIndex; x++){
			if (cmd[x]==' '){
				// this is that space that separates x and y
				xysep=x;
				tpx=(int)atof(tpxchar);	
				continue;
			} else if (xysep>0){
				// in the y
				tpychar[tpybyte]=cmd[x];
				tpybyte++;
			} else if (xysep==0){
				// in the x
				tpxchar[tpxbyte]=cmd[x];
				tpxbyte++;
			}
		}

		// finished looping through cmd, can now set tpy
		tpy=(int)atof(tpychar);	

		// x:y assumes 0:0 is top left, bottom-right max x:y is 255:255
		// move center of x:y axes to 127:127
		tpx=map(tpx, 0, 255, -128, 127);
		tpy=map(tpy, 0, 255, 127, -128);

		// turn cartesian x:y into polar, angle and radius

		// get radius using pythagorean theorum
		tpradius=sqrt((tpx*tpx) + (tpy*tpy));
		// lop off corners, just limit to a circle
		if (tpradius > 127.0) tpradius=127.0;
		// adjust to max at motor max, 255
		tpradius=tpradius * 2.0;

		// get angle using arctangent
		// convert from radians to degrees ( *180/PI )
		// convert from CCW to CW (180 - )
		// convert to azimuth (north = 0 degrees)
		tpangle=180 - atan2((double)tpy, (double)tpx) * 180 / PI;
		if (tpangle >= 90){
			tpangle=tpangle-90;
		} else {
			tpangle=tpangle+270;
		}

		// finally, drive the vehicle using angle for direction and radius for speed
		fDrivePolar(tpangle, tpradius);
	} else if(cmdStartsWith("ch") && autonomous==false) {
    int ch = cmd[2] - '0';
    if(ch>=0 && ch<=9 && cmd[3]==' ') {
      chVal[ch] = (int)atof(cmd+4);
			if (ch==0){
				thisspeed=chVal[ch];

				if (thisspeed>10){
					// north
					fDrivePolar(0, thisspeed);
				} else if (thisspeed<-10){
					// south
					thisspeed=thisspeed*-1;
					fDrivePolar(180, thisspeed);
				} else {
					// stop
					fDrivePolar(0, 0);
				}
			} else if (ch==1){
				thisspeed=chVal[ch];

				if (thisspeed>10){
					// spin right
					fSpin(thisspeed);
				} else if (thisspeed<-10){
					// spin left
					fSpin(thisspeed);
				} else {
					// stop
					fSpin(0);
				}
			} else if (ch==2){
				thisspeed=chVal[ch];

				if (thisspeed>10){
					// east
					fDrivePolar(90, thisspeed);
				} else if (thisspeed<-10){
					// west
					thisspeed=thisspeed*-1;
					fDrivePolar(270, thisspeed);
				} else {
					// stop
					fDrivePolar(0, 0);
				}
			}
    }
  } else if (cmdStartsWith("auto")) {
		// always stop when switching to/from auto
		fDrivePolar(0, 0);

		// auto/teleop
		if (cmd[5]=='1'){
			autonomous=true;
			client.write("auto mode on");
		} else {
			autonomous=false;
			client.write("auto mode off");
		}
  } else if (cmdStartsWith("hb")) {
		// heartbeat received
  	lastHeartbeatTime=millis();
	} else if( cmdStartsWith("ca") ) {
		// use accelerometer:
		// example: set RoboRemo acc y id to "ca1"
  
    int ch = cmd[2] - '0';
    if(ch>=0 && ch<=9 && cmd[3]==' ') {
      chVal[ch] = (usMax+usMin)/2 + (int)( atof(cmd+4)*51 ); // 9.8*51 = 500 => 1000 .. 2000
    }
  }
}

/*
 * handleRoot
 *
 * create and transmit the control GUI
 */
void handleRoot() {
	webserver.send(200, "text/html", "<head></head><body style=\"font-size:24px;\"><a href=\"/v1\">Video on IP 1</a><br><head></head><body><a href=\"/v2\">Video on IP 2</a></body></html>");
}

/*
 * handleNotFound
 *
 * tell the browser that the page is not found
 */
void handleNotFound(){
	webserver.send(404, "text/plain", "File not found");
}

void setup() {
  Serial.begin(115200);

	//////////////////////////////////////////////
	// drive motors
	//////////////////////////////////////////////
	AFMS.begin();
	fDrivePolar(0, 0);

	// setup wifi as both a client and as an AP
	WiFi.mode(WIFI_AP_STA);

	// start the access point
	WiFi.softAP(apssid, appassword);

	// start the client
	WiFi.begin(ssid, password);

	Serial.println("");

	unsigned int conn_tries=0;
	boolean wifi_client_gaveup=false;
	while (WiFi.status() != WL_CONNECTED) {
		conn_tries++;
		delay(500);
		Serial.print(".");
		if (conn_tries>20) {
			// give up, just be an AP via softAP
			wifi_client_gaveup=true;
			beep(200, 750, 1);
			break;
		}
	}

	if (wifi_client_gaveup==false){
		Serial.print("Connected to ssid: ");
		Serial.println(ssid);
		Serial.print("IP address: ");
		Serial.println(WiFi.localIP());

		// register hostname on network
		if (mdns.begin(host, WiFi.localIP())) {
			Serial.print("Hostname: ");
			Serial.println(host);
		}

		beep(250, 300, 1);
	}

	Serial.print("AP ssid: ");
	Serial.println(apssid);
	Serial.print("AP IP address: ");
	Serial.println(WiFi.softAPIP());

	httpUpdater.setup(&webserver);

	// establish http bootloader updater
	webserver.on("/", handleRoot);
	webserver.onNotFound(handleNotFound);
	
	webserver.begin();
	server.begin();
	Serial.println("HTTP server started");

	//////////////////////////////////////////////
	// setup piezo speaker
	//////////////////////////////////////////////
	pinMode(PIEZO, OUTPUT);

	//////////////////////////////////////////////
	// setup pixy
	//////////////////////////////////////////////
	pixy.init();

	// move servos around once
	pixy.setServos(100, 100);
	delay(1000);
	pixy.setServos(500, 500);


  cmdIndex = 0;
	beep(150, 500, 3);
}

void loop() {
	webserver.handleClient();
	loopcnt++;

  // if contact lost for more than half second
  if(millis()-lastHeartbeatTime>2000) {  
		// contact lost, stop vehicle
		connExists=false;
		fDrivePolar(0, 0);
		if (connExists==true){
			beep(300, 500, 1);
		}
  }

	// If we have blocks in sight, track and follow them
	uint16_t blocks;
	blocks = pixy.getBlocks();
	if (blocks) {
		lastBlockCount=blocks;
		int trackedBlock = TrackBlock(blocks);
		if (trackedBlock>=0){
			FollowBlock(trackedBlock);
			lastBlockTime = millis();
		} else {
			lastBlockCount=0;
			if (autonomous){
				fDrivePolar(0, 0);
			}
			//ScanForBlocks();
		}
	}  else if (thisMillis - lastBlockTime > 100) {
		//motors.setLeftSpeed(0);
		//motors.setRightSpeed(0);
		//ScanForBlocks();
		lastBlockCount=0;
		if (autonomous){
			fDrivePolar(0, 0);
		}
	}

  if(!client.connected()) {
    client = server.available();
		//Serial.println("no client connected: "+(String)loopcnt);
    return;
  }

  // here we have a connected client
  if(client.available()) {
		if (connExists==false){
			connExists=true;
			beep(150, 150, 1);
		}

    char c = (char)client.read(); // read char from client (RoboRemo app)

    if(c=='\n') { // if it is command ending
			Serial.print("received:");
			Serial.println(cmd);
      cmd[cmdIndex] = 0;
      exeCmd();  // execute the command
      cmdIndex = 0; // reset the cmdIndex
    } else {      
      cmd[cmdIndex] = c; // add to the cmd buffer
      if(cmdIndex<99) cmdIndex++;
    }
	}

  if(millis() - aliveSentTime > 500) {
		sprintf(buf, "b %d\n", lastBlockCount);
		client.write((const uint8_t*)buf, sizeof(buf));

    client.write("alive 1\n");
    // send the alive signal, so the "connected" LED in RoboRemo will stay ON
    // (the LED must have the id set to "alive")
    
    aliveSentTime = millis();
    // if the connection is lost, the RoboRemo will not receive the alive signal anymore,
    // and the LED will turn off (because it has the "on timeout" set to 700 (ms) )
  }
}
