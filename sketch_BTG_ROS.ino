
/* This Code is part of the bachelor thesis of Caspar Briner 
 It is coded for an Arduino NANO. 

 Things it does:
 
– drives an magnet with PWM
– reads in signals from an endswitch and an IR distance sensor
– lights an led for every sensor if contact is sensed 
– outputs sensor values ('0' = no contact, '1' = switch contact, '2' = IR sensor contact, '3' = both sensor sense contact)
– Magnetic force is adjustable input 
– connects to Voliro (ROS)

*/

#include <ros.h>              // include the ROS library !Important: KINETIC Realease of ROS has to be used
#include <std_msgs/Int16.h>   // use 16bit integers for communication



// define pins and variables


int contactPinIN = A0;  // set analog input pin to detect contact from endswitch
int distancePinIN = A1; // set analog input pin to get measurement input from IR distance sensor
int magnetPin = 6;      // PWM Pin for magnet connected to MOSFET

float distanceVal = 0;  // Reads in Voltage of distance sensor
int contactState =0;

float magSet = 0;       // variable where magnetic force is set



int ledContactRed = 4;    // Set led Pin to 4, red light indicates contact 
int ledDistanceBlue = 5;  // set led Pin to 5, blue light indicates distance sensor value is withing contact range
int ledMagnet = 3;        // set led Pin to 3, green light indicates magnet is on (given that power is supplied)
int ledAux = 2;           // set led pin to 2, yellow light indicates magnet is driven over 3W (100%)

float limit = 340;        // analog pin value (which goes from 0 to 1023), at which magnet has contact
                          // can be tuned, range from 330 to 360 showed to be useful. 


int output = 0;           // '0' = no contact, '1' = switch senses contact, '2' = IR sensor senses contact, '3' = both sensors sense contact
int val = 0;



// ROS

ros::NodeHandle nh;              // instantiate node handle

std_msgs::Int16 contact_msg;     // create publisher
ros::Publisher contactStateGripper("contactStateGripper", &contact_msg);  // Publisher topic: "contactStateGripper"


void messageCb (const std_msgs::Int16 magnetValueSub) {   // Callback function for subscriber. 
  val = magnetValueSub.data;                              // handle input. must be string containing a number from 0 to 100)
  if (val > 100 || 0 > val) {
    output = -1;
  }
  else {
    magSet = val;                // set magSet to value from 0 to 100
  }
}

ros::Subscriber<std_msgs::Int16> sub("magnetSetGripper", &messageCb);  // instantiate subscriber, topic: "magnetSetGripper"


void setup() {   // executed at start up



pinMode(contactPinIN, INPUT);  // contactPinIN Pin is set to input
pinMode(distancePinIN, INPUT); // distancePinIN is set to input
  

pinMode(ledContactRed, OUTPUT);   // LedContactRed pin is set for Led output
pinMode(ledDistanceBlue, OUTPUT); // LedDistanceBlue pin is set for Led output
pinMode(ledMagnet, OUTPUT); // LedDistanceBlue pin is set for Led output
pinMode(ledAux, OUTPUT); // LedDistanceBlue pin is set for Led output

pinMode(magnetPin, OUTPUT);       // PWM pin for magnet is set

analogWrite(magnetPin, 0);    // PWM pin for magnet is set to 0


// play LED light starting sequence  
digitalWrite(ledDistanceBlue , HIGH);
delay(100);
digitalWrite(ledContactRed , HIGH);
delay(100);
digitalWrite(ledMagnet , HIGH);
delay(100);
digitalWrite(ledAux , HIGH);
delay(100);

digitalWrite(ledDistanceBlue , LOW);
delay(100);
digitalWrite(ledContactRed , LOW);
delay(100);
digitalWrite(ledMagnet , LOW);
delay(100);
digitalWrite(ledAux, LOW);

// end LED light starting sequence

// initialize ROS node handle

nh.initNode();
nh.advertise(contactStateGripper);
nh.subscribe(sub);

}

void loop() {

// read in two sensor values 

distanceVal = analogRead(distancePinIN); // read in voltage of IR distance sensor (see data sheet)

contactState = digitalRead(contactPinIN);

output = 0; //Resets the output variable to 0 in every loop

// toggle LEDs


if (contactState == HIGH){            //check if endswitch senses contact
  digitalWrite(ledContactRed, LOW);  // set led pin to HIGH if yes
  
}
else {
  digitalWrite(ledContactRed, HIGH);   // else set led pin to LOW
  output += 1;
}

if(distanceVal > limit){           // check if IR sensor voltage is higher than treshold
  digitalWrite(ledDistanceBlue, HIGH);    // set led pin to HIGH if yes
  output += 2;
}
else {
 digitalWrite(ledDistanceBlue, LOW);     // else set led pin to LOW
}

if (magSet > 0) {
  digitalWrite(ledMagnet, HIGH);
}
else {
  digitalWrite(ledMagnet, LOW);
}

if (magSet > 50) {
  digitalWrite(ledAux, HIGH); // yellow LED lights up if magnet is driven at 50%
}
else {
  digitalWrite(ledAux, LOW); 
}


// set magnet force via PWM to MOSFET 

analogWrite(magnetPin, magSet/100*255); // PWM output to Mosfet


// ROS communication

contact_msg.data = output;
contactStateGripper.publish( &contact_msg);

nh.spinOnce();
delay(1);        // delay can be adjustet 


}


