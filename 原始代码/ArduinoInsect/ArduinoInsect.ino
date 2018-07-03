/*
*Copyright(C)   Zhejiang University
*FileName:			Arduino insect
*Author:			  kk
*Version:			  1st version
*Date:				  26th August,2017
*Description:		This arudino code is my summer holiday homework.
*Others:			  My attempt is to complete a mechanical insect, which can automatic keep away from barrier.
                In addtion, I also want to accomplish photoaxis, but the material object has little space
                for me to fix in the insect body, so is very pity to be failed to realize the photoaxis.
*Function List:  */
void REST();			            //this function is used to let the arduino insect have a rest, just like a real insect.
void RUN();				            //this function is used to let the arduino insect run, just like a real insect.
void SHAFT();			            //this function is used to calibrate the shaft's angle.
void SHAFT_REVERSE();	        //this function is used to reverse the shaft
void START();			            //this function is used to start
void TRANSITION_GAIT();	      //this function is used to transition the arduino insect's gait
void TRANSITION_START();      //this function is used to record the state of the transition when starting
int ROTATE_RANDOM();	        //this function is used to rotate randomly,maybe right or maybe left
void FORWARD();			          //this funciton is used to let the arduino insect go forward.
void RETREAT();			          //this function is used to let the arduino insect retreat
void WALK();			            //this function is used to let the arduino insect walk
void ROTATE();			          //this function is used to let the arduino insect rotate
void ROTATE_LEFT();		        //this function is used to let the arduino insect rotate left
void ROTATE_RIGHT();	        //this function is used to let the arduino insect rotate right
void SONAR_READ_ALL();        //this function is used to let the arduino insect konw the distance of front, left and right.
float SONAR_READ(int index);  //this function is used to let the arduino insect konw the distance of front, left or right. 
void RETREAT_AVOID();	        //this funciton is used to avoid the arduino insect retreating.
void ROTATE_LEFT_AVOID();     //this function is used to avoid the arudino insect rotating left.
void ROTATE_RIGHT_AVOID();    //this funciton is used to avoid the arduino insect rotating right.
void ROTATE_RANDOM_AVOID();   //this function is used to avoid the arduino insect rotating randomly.
void SIDE_AVOID();		        //this function is used to let the arduino insect pay attention to the side.



#include <Servo.h>
/*this arduino insect's behavior is determined by 5 servos.
2 servos are used for controling the front feet,
2 servos are used for controling the behind feet,
1 servo is used for calibrate the angle of the behind feet.
*/
//Initalize:	Calibrate servos' center angle. Start with 1500.
const int midAngleShaft = 1520;
const int midAngleFrontLeft = 1550;
const int midAngleFrontRight = 1570;
const int midAngleBackLeft = 1450;
const int midAngleBackRight = 1450;

const int angleSweep = 250;			      // Set this value (in ms) to determine how wide the servos will sweep (legs' step width). Bigger value means wider sweep angle.
const int angleRes = 10;				      // Set the servo movement resolution (in ms).
int sweepSPEED;							          // variable to determine how fast the servos will sweep.
int sweepSPEED_Rand[3] = { 4, 6, 8 };	// Servo speed (gait speed) will change randomly in 3 modes. Set the speed (in ms) for each mode. Smaller value means faster.

const int angle_turnMax = angleSweep * 1.5;     // Set this value to determine how much maximum the bot will turn toward. Bigger value means bigger turn.
const int angle_turnNarrow = angleSweep * 0.25; // Set this value to determine how much maximum the bot will turn avoiding objects at its sides in narrow space. Bigger value means bigger turn.

// Ddeclaration of servos:
Servo servoShaft;
Servo servoFrontLeft;
Servo servoFrontRight;
Servo servoBackLeft;
Servo servoBackRight;

// Variables for each servos' angles:
int angleShaft = midAngleShaft;
int angleFrontLeft = midAngleFrontLeft;
int angleFrontRight = midAngleFrontRight;
int angleBackLeft = midAngleBackLeft;
int angleBackRight = midAngleBackRight;

// Angle manipulation for middle servo (shaft).
const int maxAngleShaft = midAngleShaft + angleSweep;
const int minAngleShaft = midAngleShaft - angleSweep;
int angleSweep_val;

// Variables for recording current angles of each servos:
int angleShaftRecord;
int angleFrontLeftRecord;
int angleFrontRightRecord;
int angleBackLeftRecord;
int angleBackRightRecord;

// ID's for sonars:
int sonarID;
const int FRONT = 0;
const int LEFT = 1;
const int RIGHT = 2;

const int sonarSum = 3;						        // Amount of sonars used.
int PIN_trig[sonarSum] = { 13, 11, 8 };		// Set arduino pins connected to ultrasonic sensors' trigger pins; {front, left, right}.
int PIN_echo[sonarSum] = { 12, 10, 7 };		// Set arduino pins connected to ultrasonic sensors' echo pins; {front, left, right}.

const int distRotate = 25;			          // Configure minimum distance (in cm) between the robot and obstacle before the robot avoid it by rotating.
const int distRetreat = 10;			          // Configure minimum distance (in cm) between the robot and obstacle before the robot avoid it by retreating.
const int distTurn = 20;			            // Configure minimum distance (in cm) between the robot and obstacle before the robot avoid it by turning.
const int maxCounterGait = 8;		          // Configure how many steps the robot will take to avoid obstacle (when rotating or retreating).

const int RUN_time = 25000;			          // Configure how long the bot rest & run (in ms).
const int REST_time = 3000;

const int sonarTrigSignal = 10;          // Duration (in uS) of trigger signal the sensors needed to produce ultrasonic sound.
const unsigned long maxSonarEchoSignal = 50000;  // Maximum duration (in uS) of the echo signal given by the sensors.
const float soundSpeed = 340.0 * 100 / 1000000 ; // The speed of sound on air in cm/µS.
int distance[sonarSum];                  // Results of the calculation of distance.

// Variables for servos angles correction according to sonar detection:
int leftSonar;
int rightSonar;

// That things such as flags, counters and records.
int previousAngle;
int shaftReverseFlag;
int transitionRotateFlag;
int transitionStartFlag = 1;
int restFlag = 0;
int runTimeFlag = 0;
int randomRotate;
int counterGait;


void setup()
{

	/* add setup code here */
	// Serial.begin(9600);			// this is used for debugging and testing

	/* Set up the module pin on arduino. */
	// Setting pins for servos.
	servoShaft.attach(2);			    //Set up horizontal (shaft) servo's signal pin on arduino.
	servoFrontRight.attach(3);		//Set up front-right servo's signal pin on arduino.
	servoFrontLeft.attach(4);		  //Set up front-left servo's signal pin on arduino.
	servoBackRight.attach(5);		  //Set up back-right servo's signal pin on arduino.
	servoBackLeft.attach(6);		  //Set up back-left servo's signal pin on arduino.

	// Setting pins for sonars, both pinMode and value.
	for (sonarID = 0; sonarID < sonarSum; sonarID++) {
		pinMode(PIN_trig[sonarID], OUTPUT);
		pinMode(PIN_echo[sonarID], INPUT);
		digitalWrite(PIN_trig[sonarID], LOW);
	}

	// Get the servos ready at their middle angles.
	servoShaft.writeMicroseconds(midAngleShaft);
	servoFrontLeft.writeMicroseconds(midAngleFrontLeft);
	servoFrontRight.writeMicroseconds(midAngleFrontRight);
	servoBackLeft.writeMicroseconds(midAngleBackLeft);
	servoBackRight.writeMicroseconds(midAngleBackRight);



	randomSeed(analogRead(5));  // Using analog pin 5 to generate random values to be used later. 
	SONAR_READ_ALL();           // Initiate first sonar reading before doing anything.
	START();                    // This function make sure which legs are lifted randomly at the first step.

	delay(3000);				        //delay some time to wait to adjust the Arduino insect's position.
}

void loop()
{

	/* add main program code here */
	//let the arduino insect rest or run randomly.
	int state = random(0, 2);

	if (state == 0) {
		REST();
	}
	else {
		int randomSPEED = random(0, 3);
		sweepSPEED = sweepSPEED_Rand[randomSPEED];
		RUN();
		restFlag = 0;
	}
}

void REST() {
	if (restFlag == 0) {
		TRANSITION_GAIT();
		restFlag = 1;
	}
	delay(REST_time);
}

void RUN() {
	unsigned long TIMER_state = millis();
	while ((millis() - TIMER_state) <= RUN_time) {
		//detect the front distance 
		if (distance[FRONT] > distRotate) {
			runTimeFlag = 0;
			while (runTimeFlag == 0) {
				FORWARD();
			}
		}

		while (distance[FRONT] > distRetreat && distance[FRONT] <= distRotate) {
			while (distance[LEFT] > distance[RIGHT]) {
				ROTATE_LEFT_AVOID();
				break;
			}
			while (distance[LEFT] < distance[RIGHT]) {
				ROTATE_RIGHT_AVOID();
				break;
			}
			while (distance[LEFT] == distance[RIGHT]) {
				ROTATE_RANDOM_AVOID();
				break;
			}
		}

		while (distance[FRONT] <= distRetreat) {
			RETREAT_AVOID();
		}
	}
}

/*=================================== SHAFT MOVEMENT ===================================*/

void SHAFT() {
	unsigned long TIMER_servo = millis();
	while ((millis() - TIMER_servo) <= sweepSPEED) {
		while (angleShaft == midAngleShaft) {
			counterGait++;
			SONAR_READ_ALL();
			SIDE_AVOID();
			runTimeFlag = 1;
			break;
		}
	}

	if (previousAngle < angleShaft && angleShaft < maxAngleShaft) {
		previousAngle = angleShaft;
		angleShaft += angleRes;
	}
	else if (angleShaft >= maxAngleShaft) {
		previousAngle = angleShaft;
		angleShaft -= angleRes;
	}
	else if (previousAngle > angleShaft && angleShaft > minAngleShaft) {
		previousAngle = angleShaft;
		angleShaft -= angleRes;
	}
	else if (angleShaft <= minAngleShaft) {
		previousAngle = angleShaft;
		angleShaft += angleRes;
	}
	servoShaft.writeMicroseconds(angleShaft);	
}

void SHAFT_REVERSE() {
	if (previousAngle < angleShaft) {
		previousAngle = angleShaft + 1;
	}
	else if (previousAngle > angleShaft) {
		previousAngle = angleShaft - 1;
	}
}

/*================================ END OF SHAFT MOVEMENT ================================*/


/*===================================== TRANSITION =====================================*/

void TRANSITION_GAIT() {
	angleFrontLeftRecord = angleFrontLeft;
	angleFrontRightRecord = angleFrontRight;
	angleBackLeftRecord = angleBackLeft;
	angleBackRightRecord = angleBackRight;
	angleShaftRecord = angleShaft;

	int flag = HIGH;
	int counter = 0;
	while (flag == HIGH) {
		SHAFT();
		counter++;

		angleFrontLeft = map(counter, 1, ((angleSweep * 2) / angleRes), angleFrontLeftRecord, midAngleFrontLeft);
		angleFrontRight = map(counter, 1, ((angleSweep * 2) / angleRes), angleFrontRightRecord, midAngleFrontRight);
		angleBackLeft = map(counter, 1, ((angleSweep * 2) / angleRes), angleBackLeftRecord, midAngleBackLeft);
		angleBackRight = map(counter, 1, ((angleSweep * 2) / angleRes), angleBackRightRecord, midAngleBackRight);

		servoShaft.writeMicroseconds(angleShaft);
		servoFrontLeft.writeMicroseconds(angleFrontLeft);
		servoFrontRight.writeMicroseconds(angleFrontRight);
		servoBackLeft.writeMicroseconds(angleBackLeft);
		servoBackRight.writeMicroseconds(angleBackRight);

		if (counter == ((angleSweep * 2) / angleRes)) {
			flag = LOW;
			START();
			transitionRotateFlag = 0;
		}
	}
}

void TRANSITION_START() {
	if (angleShaft == midAngleShaft || (angleShaft > midAngleShaft && angleShaft > previousAngle) || (angleShaft < midAngleShaft && angleShaft < previousAngle)) {
		angleSweep_val = 0;
	}
	else {
		transitionStartFlag = 0;
	}
}

void START() {
	previousAngle = random((angleShaft), (angleShaft + 2));
	if (previousAngle == angleShaft) {
		previousAngle -= 1;
	}
	transitionStartFlag = 1;
}

int ROTATE_RANDOM() {
	return random(0, 2);
}

/*================================== END OF TRANSITION ==================================*/


/*======================================== WALK ========================================*/

void FORWARD() {
	while (transitionRotateFlag == 2) {
		TRANSITION_GAIT();
	}
	transitionRotateFlag = 1;

	while (shaftReverseFlag == 0) {
		SHAFT_REVERSE();
		break;
	}
	shaftReverseFlag = 1;

	while (transitionStartFlag == 1) {
		SHAFT();
		TRANSITION_START();
		WALK();
	}

	SHAFT();
	angleSweep_val = angleSweep;
	WALK();
}

void RETREAT() {
	while (transitionRotateFlag == 2) {
		TRANSITION_GAIT();
	}
	transitionRotateFlag = 1;

	while (shaftReverseFlag == 1) {
		SHAFT_REVERSE();
		break;
	}
	shaftReverseFlag = 0;

	while (transitionStartFlag == 1) {
		SHAFT();
		TRANSITION_START();
		WALK();
	}

	SHAFT();
	angleSweep_val = (angleSweep * -1);
	WALK();
}

void WALK() {
	if (angleShaft >= midAngleShaft && previousAngle < angleShaft) {
		angleFrontLeft = map(angleShaft, midAngleShaft, maxAngleShaft, ((midAngleFrontLeft - angleSweep_val) + leftSonar), midAngleFrontLeft);
		angleFrontRight = map(angleShaft, midAngleShaft, maxAngleShaft, ((midAngleFrontRight - angleSweep_val) - rightSonar), midAngleFrontRight);
		angleBackLeft = map(angleShaft, midAngleShaft, maxAngleShaft, ((midAngleBackLeft + angleSweep_val) - leftSonar), midAngleBackLeft);
		angleBackRight = map(angleShaft, midAngleShaft, maxAngleShaft, ((midAngleBackRight + angleSweep_val) + rightSonar), midAngleBackRight);
	}
	else if (angleShaft >= midAngleShaft && previousAngle > angleShaft) {
		angleFrontLeft = map(angleShaft, maxAngleShaft, midAngleShaft, midAngleFrontLeft, ((midAngleFrontLeft + angleSweep_val) - leftSonar));
		angleFrontRight = map(angleShaft, maxAngleShaft, midAngleShaft, midAngleFrontRight, ((midAngleFrontRight + angleSweep_val) + rightSonar));
		angleBackLeft = map(angleShaft, maxAngleShaft, midAngleShaft, midAngleBackLeft, ((midAngleBackLeft - angleSweep_val) + leftSonar));
		angleBackRight = map(angleShaft, maxAngleShaft, midAngleShaft, midAngleBackRight, ((midAngleBackRight - angleSweep_val) - rightSonar));
	}
	else if (angleShaft < midAngleShaft && previousAngle > angleShaft) {
		angleFrontLeft = map(angleShaft, midAngleShaft, minAngleShaft, ((midAngleFrontLeft + angleSweep_val) - leftSonar), midAngleFrontLeft);
		angleFrontRight = map(angleShaft, midAngleShaft, minAngleShaft, ((midAngleFrontRight + angleSweep_val) + rightSonar), midAngleFrontRight);
		angleBackLeft = map(angleShaft, midAngleShaft, minAngleShaft, ((midAngleBackLeft - angleSweep_val) + leftSonar), midAngleBackLeft);
		angleBackRight = map(angleShaft, midAngleShaft, minAngleShaft, ((midAngleBackRight - angleSweep_val) - rightSonar), midAngleBackRight);
	}
	else if (angleShaft < midAngleShaft && previousAngle < angleShaft) {
		angleFrontLeft = map(angleShaft, minAngleShaft, midAngleShaft, midAngleFrontLeft, ((midAngleFrontLeft - angleSweep_val) + leftSonar));
		angleFrontRight = map(angleShaft, minAngleShaft, midAngleShaft, midAngleFrontRight, ((midAngleFrontRight - angleSweep_val) - rightSonar));
		angleBackLeft = map(angleShaft, minAngleShaft, midAngleShaft, midAngleBackLeft, ((midAngleBackLeft + angleSweep_val) - leftSonar));
		angleBackRight = map(angleShaft, minAngleShaft, midAngleShaft, midAngleBackRight, ((midAngleBackRight + angleSweep_val) + rightSonar));
	}

	servoFrontLeft.writeMicroseconds(angleFrontLeft);
	servoFrontRight.writeMicroseconds(angleFrontRight);
	servoBackLeft.writeMicroseconds(angleBackLeft);
	servoBackRight.writeMicroseconds(angleBackRight);
}

/*===================================== END OF WALK =====================================*/


/*======================================= ROTATE =======================================*/

void ROTATE_LEFT() {
	while (transitionRotateFlag == 1) {
		TRANSITION_GAIT();
	}
	transitionRotateFlag = 2;

	while (shaftReverseFlag == 2) {
		SHAFT_REVERSE();
		break;
	}
	shaftReverseFlag = 3;

	while (transitionStartFlag == 1) {
		SHAFT();
		TRANSITION_START();
		ROTATE();
	}

	SHAFT();
	angleSweep_val = angleSweep;
	ROTATE();
}

void ROTATE_RIGHT() {
	while (transitionRotateFlag == 1) {
		TRANSITION_GAIT();
	}
	transitionRotateFlag = 2;

	while (shaftReverseFlag == 3) {
		SHAFT_REVERSE();
		break;
	}
	shaftReverseFlag = 2;

	while (transitionStartFlag == 1) {
		SHAFT();
		TRANSITION_START();
		ROTATE();
	}

	SHAFT();
	angleSweep_val = (angleSweep * -1);
	ROTATE();
}

void ROTATE() {
	if (angleShaft >= midAngleShaft && previousAngle < angleShaft) {
		angleFrontLeft = map(angleShaft, midAngleShaft, maxAngleShaft, (midAngleFrontLeft + angleSweep_val), midAngleFrontLeft);
		angleFrontRight = map(angleShaft, midAngleShaft, maxAngleShaft, (midAngleFrontRight - angleSweep_val), midAngleFrontRight);
		angleBackLeft = map(angleShaft, midAngleShaft, maxAngleShaft, (midAngleBackLeft - angleSweep_val), midAngleBackLeft);
		angleBackRight = map(angleShaft, midAngleShaft, maxAngleShaft, (midAngleBackRight + angleSweep_val), midAngleBackRight);
	}
	else if (angleShaft >= midAngleShaft && previousAngle > angleShaft) {
		angleFrontLeft = map(angleShaft, maxAngleShaft, midAngleShaft, midAngleFrontLeft, (midAngleFrontLeft - angleSweep_val));
		angleFrontRight = map(angleShaft, maxAngleShaft, midAngleShaft, midAngleFrontRight, (midAngleFrontRight + angleSweep_val));
		angleBackLeft = map(angleShaft, maxAngleShaft, midAngleShaft, midAngleBackLeft, (midAngleBackLeft + angleSweep_val));
		angleBackRight = map(angleShaft, maxAngleShaft, midAngleShaft, midAngleBackRight, (midAngleBackRight - angleSweep_val));
	}
	else if (angleShaft < midAngleShaft && previousAngle > angleShaft) {
		angleFrontLeft = map(angleShaft, midAngleShaft, minAngleShaft, (midAngleFrontLeft - angleSweep_val), midAngleFrontLeft);
		angleFrontRight = map(angleShaft, midAngleShaft, minAngleShaft, (midAngleFrontRight + angleSweep_val), midAngleFrontRight);
		angleBackLeft = map(angleShaft, midAngleShaft, minAngleShaft, (midAngleBackLeft + angleSweep_val), midAngleBackLeft);
		angleBackRight = map(angleShaft, midAngleShaft, minAngleShaft, (midAngleBackRight - angleSweep_val), midAngleBackRight);
	}
	else if (angleShaft < midAngleShaft && previousAngle < angleShaft) {
		angleFrontLeft = map(angleShaft, minAngleShaft, midAngleShaft, midAngleFrontLeft, (midAngleFrontLeft + angleSweep_val));
		angleFrontRight = map(angleShaft, minAngleShaft, midAngleShaft, midAngleFrontRight, (midAngleFrontRight - angleSweep_val));
		angleBackLeft = map(angleShaft, minAngleShaft, midAngleShaft, midAngleBackLeft, (midAngleBackLeft - angleSweep_val));
		angleBackRight = map(angleShaft, minAngleShaft, midAngleShaft, midAngleBackRight, (midAngleBackRight + angleSweep_val));
	}

	servoFrontLeft.writeMicroseconds(angleFrontLeft);
	servoFrontRight.writeMicroseconds(angleFrontRight);
	servoBackLeft.writeMicroseconds(angleBackLeft);
	servoBackRight.writeMicroseconds(angleBackRight);
}

/*==================================== END OF ROTATE ====================================*/


/*================================= ULTRASONIC READING =================================*/

void SONAR_READ_ALL() {
	for (sonarID = 0; sonarID < sonarSum; sonarID++) {
		distance[sonarID] = int(SONAR_READ(sonarID));
	}
}

float SONAR_READ(int index) {
	float SONAR_distance;

	digitalWrite(PIN_trig[index], HIGH);
	delayMicroseconds(sonarTrigSignal);
	digitalWrite(PIN_trig[index], LOW);

	float SONAR_EcInterval = float(pulseIn(PIN_echo[index], HIGH, maxSonarEchoSignal));

	while (SONAR_EcInterval > 0.0) {
		SONAR_distance = SONAR_EcInterval * (soundSpeed / 2.0);
		break;
	}
	while (SONAR_EcInterval == 0.0) {
		SONAR_distance = 501.0;
		break;
	}
	return SONAR_distance;
}

/*============================= END OF ULTRASONIC READING  =============================*/


/*====================================== BEHAVIOUR ======================================*/

void RETREAT_AVOID() {
	counterGait = 0;
	while (counterGait <= maxCounterGait) {
		RETREAT();
	}
}

void ROTATE_LEFT_AVOID() {
	counterGait = 0;
	randomRotate = 2;
	while (counterGait <= maxCounterGait) {
		ROTATE_LEFT();
	}
}

void ROTATE_RIGHT_AVOID() {
	counterGait = 0;
	randomRotate = 2;
	while (counterGait <= maxCounterGait) {
		ROTATE_RIGHT();
	}
}

void ROTATE_RANDOM_AVOID() {
	randomRotate = ROTATE_RANDOM();
	while (randomRotate == 0) {
		ROTATE_LEFT_AVOID();
	}
	while (randomRotate == 1) {
		ROTATE_RIGHT_AVOID();
	}
}

void SIDE_AVOID() {
	if (distance[LEFT] <= distTurn && distance[RIGHT] > distTurn) {

		leftSonar = 0;
		rightSonar = -(map(distance[LEFT], 0, distTurn, angle_turnMax, 0));
	}
	else if (distance[RIGHT] <= distTurn && distance[LEFT] > distTurn) {

		rightSonar = 0;
		leftSonar = map(distance[RIGHT], 0, distTurn, angle_turnMax, 0);
	}
	else if (distance[LEFT] <= distTurn && distance[RIGHT] <= distTurn) {

		if (distance[LEFT] < distance[RIGHT]) {
			leftSonar = 0;
			rightSonar = -(map(distance[LEFT], 0, distTurn, angle_turnNarrow, 0));
		}
		else if (distance[RIGHT] < distance[LEFT]) {
			rightSonar = 0;
			leftSonar = map(distance[RIGHT], 0, distTurn, angle_turnNarrow, 0);
		}
	}
	else {
		rightSonar = 0;
		leftSonar = 0;
	}
}

/*================================== END OF BEHAVIOUR ==================================*/
