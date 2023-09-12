// Include Libraries
#include <Servo.h>

// Define pins for RC inputs - all analog pins
#define RIGHT_IN A1   // Channel 3 - Get input for right thruster
#define LEFT_IN A4    // Channel 2 - Get input for left thruster
#define LEFT_KNOB A2  // Channel 6 - Activate or deactivate kill switch
#define RIGHT_KNOB A3 // Channel 5 - switch between manual and autonomous

// Define pins for thrusters - all digital pwm pins
#define R_THRUSTER 13
#define L_THRUSTER 12

// Define pins to read cell voltage from battery
#define BAT_CELL1 A15
#define BAT_CELL2 A14
#define BAT_CELL3 A13
#define BAT_CELL4 A12

// Define pin to read onbaord killswitch
#define ONBOARD_KS 22

// Define ranges of RC inputs
#define RIGHT_IN_MAX 1950
#define RIGHT_IN_MIN 1200
#define LEFT_IN_MAX 1950
#define LEFT_IN_MIN 1200
#define LEFT_KNOB_MAX 2030  // Down position
#define LEFT_KNOB_MIN 990   // Up position
#define RIGHT_KNOB_MAX 2030 // Down position
#define RIGHT_KNOB_MIN 980  // Up position

// Define buffer ranges for knob control
#define VALUE_BUFFER 400

// Amount of iterations between battery reads
#define BATTERY_ITER 1000

// Define ranges of thruster PWM output - standard to T200
#define THRUSTER_MAX 1900
#define THRUSTER_MIN 1100
#define THRUSTER_STOP 1500

// Define Thruster as Servos
Servo right_thruster;
Servo left_thruster;

// Global Variables
int right_in_val;
int left_in_val;
int l_knob_val;
int r_knob_val;
int thruster_r_out;
int thruster_l_out;
int onboard_ks_val = false;
bool sys_kill = false; // hold state of kill switch, true if killed
bool is_manual = true;
int battery_counter = 0;

// debouncing vars
int lg_right_knob;
int lg_left_knob;
int lg_right_in;
int lg_left_in;
int db_left_knob = 0;
int db_right_knob = 0;
int db_left_input = 0;
int db_right_input = 0;

#define DEBOUNCING_THRESHOLD 3

// Battery global variables
float sensorValue0;
float sensorValue1;
float sensorValue2;
float sensorValue3;

// ==================================================================================
// Stop thrusters and activate kill switch
// Sits and waits until system is unkilled
void killAll()
{
  sys_kill = true;
  right_thruster.writeMicroseconds(1500);
  left_thruster.writeMicroseconds(1500);
  Serial.println("KILL");
  l_knob_val = pulseIn(LEFT_KNOB, HIGH);
  right_in_val = pulseIn(RIGHT_IN, HIGH);
  left_in_val = pulseIn(LEFT_IN, HIGH);
  r_knob_val = pulseIn(RIGHT_KNOB, HIGH);
  onboard_ks_val = digitalRead(ONBOARD_KS);
  while (l_knob_val < LEFT_KNOB_MIN + VALUE_BUFFER || onboard_ks_val == 0 || right_in_val < RIGHT_IN_MIN || left_in_val < LEFT_IN_MIN || r_knob_val < RIGHT_KNOB_MIN) // exit while loop if both kill switches are not active
  {
    onboard_ks_val = digitalRead(ONBOARD_KS);
    l_knob_val = pulseIn(LEFT_KNOB, HIGH);
    right_in_val = pulseIn(RIGHT_IN, HIGH);
    left_in_val = pulseIn(LEFT_IN, HIGH);
    r_knob_val = pulseIn(RIGHT_KNOB, HIGH);
    // Serial.println("L_knob: " + String(l_knob_val) + ", KS: " + String(onboard_ks_val));
    Serial.println("KILL");
  }
  unKillAll();
}
// ==================================================================================
// ==================================================================================

// Deactivate kill switch
void unKillAll()
{
  sys_kill = false;
  right_thruster.writeMicroseconds(1500);
  left_thruster.writeMicroseconds(1500);
  Serial.println("UNKILL");
  delay(3000);
}
// ==================================================================================

// ==================================================================================
// Runs once upn start-up
void setup()
{
  // Set up serial communication
  Serial.begin(115200);
  // Serial.println("Started");

  // Set up I/O pins
  pinMode(RIGHT_IN, INPUT);
  pinMode(LEFT_IN, INPUT);
  pinMode(LEFT_KNOB, INPUT);
  pinMode(RIGHT_KNOB, INPUT);
  pinMode(ONBOARD_KS, INPUT);
  pinMode(BAT_CELL1, INPUT);
  pinMode(BAT_CELL2, INPUT);
  pinMode(BAT_CELL3, INPUT);
  pinMode(BAT_CELL4, INPUT);

  // Attach thruster pin to servo object
  right_thruster.attach(R_THRUSTER);
  left_thruster.attach(L_THRUSTER);
  right_thruster.writeMicroseconds(1500);
  left_thruster.writeMicroseconds(1500);

  // Only go when user is ready
  bool notReady = true;
  while (notReady)
  {
    onboard_ks_val = digitalRead(ONBOARD_KS);
    l_knob_val = pulseIn(LEFT_KNOB, HIGH);
    // Serial.println("Waiting to Start. Left Knob Value: " + String(l_knob_val) + ", KS: " + String(onboard_ks_val));
    if (l_knob_val > (LEFT_KNOB_MIN + VALUE_BUFFER) && onboard_ks_val == 1)
    {
      notReady = false;
    }
  }

  // Stop thrusters and delay for 3 seconds to initialize
  // You should hear a series of beeps
  right_thruster.writeMicroseconds(1500);
  left_thruster.writeMicroseconds(1500);
  delay(3000);

  // Serial.println("Started Moving");
}
// ==================================================================================

// ==================================================================================

void debouncing(int &input_val, int input_min, int &db_input, int &lg_input)
{
  if (input_val < input_min)
  {
    db_input++;

    if (db_input > DEBOUNCING_THRESHOLD)
    {
      killAll();
    }
    else
    {
      input_val = lg_input;
    }
  }
  else
  {
    db_input = 0;
    lg_input = input_val;
  }
}

// ==================================================================================

// ==================================================================================
// Continuously runs
void loop()
{
  onboard_ks_val = digitalRead(ONBOARD_KS);
  if (onboard_ks_val == 0) // Onboard kill switch pressed
  {
    killAll(); // Stop motors
  }

  // Poll the RC inputs for knobs, update the values
  l_knob_val = pulseIn(LEFT_KNOB, HIGH);
  r_knob_val = pulseIn(RIGHT_KNOB, HIGH);

  debouncing(r_knob_val, RIGHT_KNOB_MIN, db_right_knob, lg_right_knob);

  // Left knob (remote kill switch) logic, 500 allows for knob to act as switch
  if (l_knob_val < LEFT_KNOB_MIN + 500 && l_knob_val > LEFT_KNOB_MIN)
  {
    killAll();
  }
  debouncing(l_knob_val, LEFT_KNOB_MIN, db_left_knob, lg_left_knob);

  // Right knob (manual/autonomous switch) logic
  if (r_knob_val <= (RIGHT_KNOB_MIN + VALUE_BUFFER) && !is_manual) // Manual signal received from user
  {
    Serial.println("MAN");
    is_manual = true;
  }
  else if (r_knob_val > (RIGHT_KNOB_MIN + VALUE_BUFFER) && is_manual)
  {
    Serial.println("AUTO");
    thruster_r_out = 1500;
    thruster_l_out = 1500;
    is_manual = false;
  }

  // Only do if manual mode
  if (is_manual)
  {
    left_in_val = pulseIn(LEFT_IN, HIGH);
    right_in_val = pulseIn(RIGHT_IN, HIGH);

    // left in deboucning logic
    debouncing(left_in_val, LEFT_IN_MIN, db_left_input, lg_left_in);

    // Right in debouncing logic
    debouncing(right_in_val, RIGHT_IN_MIN, db_right_input, lg_right_in);

    // flip THRUSTER_MIN and THRUSTER_MAX to flip direction of thrusters
    thruster_r_out = (int)map(right_in_val, RIGHT_IN_MAX, RIGHT_IN_MIN, THRUSTER_MIN, THRUSTER_MAX);
    if (thruster_r_out < 1600 && thruster_r_out > 1400)
    {
      thruster_r_out = 1500;
    }

    // flip THRUSTER_MIN and THRUSTER_MAX to flip direction of thrusters
    thruster_l_out = (int)map(left_in_val, LEFT_IN_MAX, LEFT_IN_MIN, THRUSTER_MIN, THRUSTER_MAX);
    if (thruster_l_out < 1600 && thruster_l_out > 1400)
    {
      thruster_l_out = 1500;
    }
  }
  else // Only do if autonomous
  {
    // Serial handling from Jetson
    if (Serial.available() > 0)
    {
      String data = Serial.readStringUntil('\n');
      if (data.startsWith("MC "))
      {
        // extract the right and left motor values from the string
        thruster_r_out = data.substring(4, 8).toInt();
        thruster_l_out = data.substring(10, 14).toInt();

        // Handle cases where incorrect values are sent
        thruster_r_out = constrain(thruster_r_out, 1100, 1900);
        thruster_l_out = constrain(thruster_l_out, 1100, 1900);

        // flip THRUSTER_MIN and THRUSTER_MAX to flip direction of thrusters
        thruster_r_out = (int)map(thruster_r_out, THRUSTER_MAX, THRUSTER_MIN, THRUSTER_MIN, THRUSTER_MAX);
        thruster_l_out = (int)map(thruster_l_out, THRUSTER_MAX, THRUSTER_MIN, THRUSTER_MIN, THRUSTER_MAX);
      }

      if (data == "KILL")
      {
        right_thruster.writeMicroseconds(1500);
        left_thruster.writeMicroseconds(1500);
        Serial.println("KILL");
        while (1)
        {
          // Serial.println("Autonomous kill");
        } // stop running all code
      }
    }
  }

  // Monitor battery logic
  //   battery_counter++;
  //   if (battery_counter > BATTERY_ITER) // no need to run this every time... check every x amount of iterations
  //   {
  //     sensorValue0 = analogRead(A0);
  //     sensorValue1 = analogRead(A1);
  //     sensorValue2 = analogRead(A2);
  //     sensorValue3 = analogRead(A3);
  //     sensorValue0 = (2*(sensorValue0*5)/1023);
  //     sensorValue1 = (2*(sensorValue1*5)/1023) - sensorValue0;
  //     sensorValue2 = (6*(sensorValue2*5)/1023) - sensorValue0 - sensorValue1;
  //     sensorValue3 = (11*(sensorValue3*5)/1023) - sensorValue0 - sensorValue1 - sensorValue2;
  //     battery_counter = 0;

  //     // Serial.print("A0: " );
  //     // Serial.println(sensorValue0);
  //     // Serial.print("; A1: " );
  //     // Serial.println(sensorValue1);
  //     // Serial.print("; A2: " );
  //     // Serial.println(sensorValue2);
  //     // Serial.print("; A3: " );
  //     // Serial.println(sensorValue3);

  //     bool battBool = sensorValue0 < 3.2 || sensorValue1 < 3.2 ||sensorValue2 < 3.2 || sensorValue3 < 3.2;
  //     if (battBool)
  //     {
  //       right_thruster.writeMicroseconds(1500);
  //       left_thruster.writeMicroseconds(1500);
  //       Serial.println("KILL");
  //       while (1) {} // stop running all code, battery is dead
  //     }
  //   }

  // Write to thrusters
  right_thruster.writeMicroseconds(thruster_r_out);
  left_thruster.writeMicroseconds(thruster_l_out);

  // Print values.. for testing only
  // Serial.print("; onboard: " + String(onboard_ks_val));
  // Serial.print("; is_manual: " + String(is_manual));
  // Serial.print("; L knob: " + String(l_knob_val));
  // Serial.print("; R knob: " + String(r_knob_val));
  Serial.print("; L thr: " + String(thruster_l_out) + " || " + String(left_in_val));
  Serial.print("; R thr: " + String(thruster_r_out) + " || " + String(right_in_val));
  Serial.print("; db Rknob: " + String(db_right_knob));
  Serial.print("; db Lknob: " + String(db_left_knob));
  // Serial.print("; db Lin: " + String(db_left_input));
  // Serial.print("; db Rin: " + String(db_right_input));
  Serial.println();
}
// ==================================================================================