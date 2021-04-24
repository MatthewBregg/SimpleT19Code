
////S-Core Blaster Controller Firmware
////Version 0.98
////by torukmakto4
////DZ Industries
////This is part of Project T19
////Forever Open - Forever Independent
////Project T19 and this Software (DZ Core) are released under the Creative Commons BY-SA 4.0 license.
////https://creativecommons.org/licenses/by-sa/4.0/
////This is free and open source software.
////You may use or modify it for any purpose. No warranty or guarantee of fitness for any purpose is provided.
////It may be included in a product for sale, given that attribution is provided,
////and that if altered, the modified source is made publically available in a very clear manner.
////ANY DERIVATIVE OF THIS PRODUCT MUST BE PUBLISHED UNDER THE ORIGINAL LICENSE.
////No attempt to circumvent/thwart access or obfuscate its release by any technical measure may be made.
////In the end:
////Please arms race responsibly,
////don't be greedy,
////and respect the sporting spirit of our great hobby,
////above all else.
////DBAD.


// PC1 - A1 - VOLTIMETER
// - A4 - DRV8825 Step signal. Output. Rising edge triggered.
const int step_pin = A4; // Good, no change here from stock firmware!
// - A5 - DRV8825 Direction signal. Output.
const int dir_pin = A5;
// - D6 - Trigger input 1. Pullup.
const int trig_a_pin = 6;
// - D7 - Trigger input 2. Pullup.
const int trig_b_pin = 7;
// - D2 - INT0 - Tach input channel 0 (M1F). (Warning: These should NOT be allowed to float with the interrupt turned on if unused!!!!!!)
// - D3 - INT1 - Tach input channel 1 (M2F).
// - D12 - Bolt limit switch input. Pullup.
const int limit_pin = 12;
// - A0 - DRV8825 M2. Output
const int m0_pin = A5;
// - D11 - DRV8825 M1. Output
const int m1_pin = 11;
// - D5 - DRV8825 M0. Output
const int m2_pin = 5;
// - D10 - OCR1B - Throttle channel 0 (M1, M1F) associated with INT0 (PD2)
// - D9 - OCR1A - Throttle channel 1 (M2, M2F) associated with INT1 (PD3)
// - D8- DRV8825 Enable. Output. Active low.
const int enable_pin = 8;

#define READTRIGGER() ((PIND & /* trig_a_pin */ 0b01000000) && !(PIND & /* trig_b_pin */ 0b10000000))
#define READLIMIT() ((PINB & /* limit pin */ 0b00010000))


int           alarm_want                  = 0;     //Load nonzero value on STC abort to get alarm before driver shutdown. 1 for _beeep_ and anything else for ^beep^-_beep_.
// Stepper drive variables:
//// This uses the per-cycle lightweight calculation of stepper commutation period described in
//// http://hwml.com/LeibRamp.pdf
//BOLT DRIVE PARAMETERS - don't mess with these unless you know what you are doing
const unsigned long startSpeed = 400; //us (default 400 - leave alone)
const unsigned long shiftSpeed = 150; //us (default 150)
const unsigned long runSpeedMax = 75000/(720 /* Rounds Per minute */); //Bolt speed absolute maximum - limit precisely
const double accelPhaseOne = 0.000000253494; //m value for ramp in low speed region
const double accelPhaseTwo = 0.000000180748; //m value for ramp in high speed region
const double decel =         0.000000439063; //m value for hard decel
const int boltShutdownTime = 1000; //ms - post-shot delay after which motor current is turned off

int stepsToGo;
double currSpeed = startSpeed; //to be used by fire() to be aware of motor speed from last run and mate speed ramp to that
double stepdelay; //us
boolean boltHomed = 0;
unsigned long runSpeed;         //us
 
////PWM Timer1 interrupt governor control variables
volatile int gov_currentBit;               //Loop counter/index (shall not be unsigned)
volatile unsigned long save_ocr1a;         //Save throttle settings to resume emitting them after all digital speed bits are shifted out
volatile unsigned long save_ocr1b;
volatile unsigned long governor;           //this has to be a volatile even though the ISR does not modify it
volatile boolean gov_enable_strobe = false; //Set to true to fire off update
volatile boolean gov_packet_active = false; //True while shifting bits out in the gov ISR, resets at the end of the cleanup cycle

int gov_update_repeats = 10;               //Number of min. times to repeat (Like Dshot commands, a one-time setting should be sent about 10 times to ensure noise robustness, etc.)

////FlyShot constant/setup section
volatile unsigned long gov_ocr_t0h = 26;   //T0H = 100us (Less than 250us)
volatile unsigned long gov_ocr_t1h = 90;   //T1H = 400us (More than 250 and less than MIN_RC_PULS)
//TL is by the PWM period. We will just keep running at 400Hz.
  
////tach variables - modified in tach ISRs
////tach pulse is counted on rising edge, once per AC cycle: 6*TIMING_MAX/4
////speed range should be something like 1200us - 330us for 7100 - 25900 motor rpm for reference
volatile unsigned long thisPulseTime0;
volatile unsigned long lastPulseTime0;
volatile unsigned long pulseLength0;
volatile unsigned long thisPulseTime1;
volatile unsigned long lastPulseTime1;
volatile unsigned long pulseLength1;

////tach watchdog variables
volatile boolean drive0TachValid = false;  //Make sure we received fresh data before acting on it
volatile boolean drive1TachValid = false;

////Orthomatic - Tach based STC variables.
volatile unsigned long speedSetpoint;     //us (6 * TIMING_MAX / 4)
int goodTachsToFire = 5;                  //how many consecutive good readings to consider motors to be stable and start bolt motion?
unsigned long speedOffsetMargin;          //Consider tach in range if speedOffsetMargin us greater than (lower speed) speedSetpoint
unsigned long speedOffsetMarginMin = 45;  //At low speed   (This compensates competing effects from period being 1/f, control loop performance, and tach resolution so isn't much more
unsigned long speedOffsetMarginMax = 22;  //At max speed    than a fudge factor)
int goodTachCount = 0;                    //counter for loops with good speed reading
unsigned long startBlankTime  = 1;        //actually delay step for tach checks
unsigned long failTime = 1000;            //Abort if drives don't either send tach pulses or reach set speed in a reasonable time.
unsigned long lastTriggerDown;            //(ms) time trigger pulled
unsigned long minRPM = 8000;              //Fly speed command min (Set this appropriately for your system to ensure passing darts at minimum speed)
unsigned long maxRPM = 30000;             //Fly speed command max (Set this appropriately. If the drive can't actually reach this speed you won't be able to fire!)


////Speed/Analog control variables.
unsigned long motorPolepairs = 7;         //Pole order of your flywheel motors

//fire control state variables section
boolean prevTrigState = 0;
boolean currTrigState = 0;
unsigned long lastTriggerUp = 0;
boolean firstRun = 1;

//Selftest counters
int selftestStepCounter = 0;
int selftestCycleCounter = 0;
int selftestGrowlCounter = 0;  //For internal use stepper_code_blipper
int selftestAlarmCounter = 0;  //For internal use stepper_alarm

//Motor drive selftest stuff
unsigned long selftestTachIntegPollTime   = 300;  //us - delay step for monitoring state of tach lines at idle to hopefully catch swings
int           selftestTachIntegCheckCount = 10;   //Number of matching logic states above time apart to determine stable
int           selftestTachIntegChecks;            //Counter
boolean       selftestTachState;                  //Save logic state of first read

unsigned long selftestTachSpinFloor       = 4800; //us - tach period to consider motor spinning if consistently shorter than (must be subst. longer than a flyshot default minimum speed if so config'd)
int           selftestGoodTachs           = 20;   //Number of consecutive above-floor tach reads with less than selftestTachMulligans failures to pass selftest
//int         goodTachCount (shared with other tach utilities)
int           selftestMaxTachMulligans    = 7;    //zero goodTachCount after this many accumulated failures per count sequence during selftest
int           selftestTachMulligans;              //Number of failures during this count seqquence (Consecutive/nonconsecutive etc. doesn't matter)
unsigned long selftestTachTimeout         = 1000; //ms - die after attempting to drive motor for this time without reaching selftestGoodTachs
unsigned long selftestTimeStartedTaching;         //=millis() when tach counting started

unsigned long speedtrap_buf0;                     //Add every successive read to this and /2 (LSR) it. Rolling-average low pass filter thingy. May be later implemented for fire control (not floor test)
unsigned long speedtrap_buf1;                     //Ditto for drive 1 (Done simultaneously)
unsigned long speedtrap_offsetMargin      = 20;   //Increment goodTachCount in speedtrap when both buffers within this distance of the setpoint
unsigned long speedtrap_overspeedTripMargin = 65; //Unfiltered "critical overspeed" detector uses this. Fails on any single pulse shorter than this after a delay to account for any initial desyncs



void commutate(double commDelay) {
  //Advance 1 microstep.
  //Send rising edge
  PORTC |= 0b00010000;
  //delayMicroseconds() is actually very accurate. N.b.: delayMicroseconds() Does NOT use interrupts.
  delayMicroseconds((unsigned long)(commDelay / 2));
  //Send falling edge
  // Register digital write
  PORTC = PORTC & 0b11101111;
  //Delay again. No particular reason for the symmetrical square wave.
  delayMicroseconds((unsigned long)(commDelay / 2));
}


bool decelerateBoltToSwitch() {
  //try to gracefully end drivetrain rotation
  //called after the last fire() has returned
  //return true for home and false for not home
  //fire() runs the bolt forward 720 clicks (on 4:1 mode) leaving us 80 to bring it down to ~3rps where we know it can stop instantly and keep position.
  //abort instantly on limit switch low
  stepsToGo = 150;
  stepdelay = currSpeed;
  while ((stepsToGo > 0) && READLIMIT()) {
    commutate(stepdelay);
    if (stepdelay < startSpeed) {
      stepdelay = stepdelay * (1 + decel * stepdelay * stepdelay);
    }
    stepsToGo--;
  }
  currSpeed = startSpeed;
  boltHomed = 1;
  return !READLIMIT();
}

bool reverseBoltToSwitch() {
  //this is called if decelerateBoltToSwitch() returns false, and to realign on startup.
  stepsToGo = 800; //up to a full rev permitted
  //set bolt direction reverse
  digitalWrite(dir_pin, HIGH);
  //run bolt back at idle speed
  while READLIMIT() {
    commutate(startSpeed);
    stepsToGo--;
    if (stepsToGo == 0) {
      //ran out of angle, die and reset direction
      digitalWrite(dir_pin, LOW);
      return false;
    }
  }
  digitalWrite(dir_pin, LOW);
  return !READLIMIT();
}

void stepper_growl(void) {
  //Stepper user feedback utility: low frequency growl. Makes a nice clattery noise from the linkage on older looser guns.
  //Assumptions: Bolt current is ON.
  //Limitations: Blocks. Don't use while waiting for trigger events. Make another trigger-aware noise generating function for that.

  selftestCycleCounter = 0;
  while (selftestCycleCounter < 3) {   // ~0.13 s
    //do 4 microsteps one direction
    selftestStepCounter = 0;
    digitalWrite(dir_pin, LOW);
    while (selftestStepCounter < 4) {
      commutate(5000);
      selftestStepCounter++;
    }
    //and do 4 microsteps back the other way
    selftestStepCounter = 0;
    digitalWrite(dir_pin, HIGH);
    while (selftestStepCounter < 4) {
      commutate(5000);
      selftestStepCounter++;
    }
    selftestCycleCounter++;
  }
  digitalWrite(dir_pin, LOW);
  return;
}

void stepper_beep_low(void) {
  //Stepper user feedback utility: _beep_ tone.
  //Assumptions: Bolt current is ON.
  //Limitations: Blocks. Don't use while waiting for trigger events. Make another trigger-aware noise generating function for that.

  selftestCycleCounter = 0;
  while (selftestCycleCounter < 70) {   // ~0.25 s
    //do 4 microsteps one direction
    selftestStepCounter = 0;
    digitalWrite(dir_pin, LOW);
    while (selftestStepCounter < 4) {
      commutate(210);
      selftestStepCounter++;
    }
    //and do 4 microsteps back the other way
    selftestStepCounter = 0;
    digitalWrite(dir_pin, HIGH);
    while (selftestStepCounter < 4) {
      commutate(210);
      selftestStepCounter++;
    }
    selftestCycleCounter++;
  }
  digitalWrite(dir_pin, LOW);
  return;
}

void stepper_beep_high(void) {
  //Stepper user feedback utility: ^beep^ tone.
  //Assumptions: Bolt current is ON.
  //Limitations: Blocks. Don't use while waiting for trigger events. Make another trigger-aware noise generating function for that.

  selftestCycleCounter = 0;
  while (selftestCycleCounter < 140) {   // ~0.25 s
    //do 4 microsteps one direction
    selftestStepCounter = 0;
    digitalWrite(dir_pin, LOW);
    while (selftestStepCounter < 4) {
      commutate(98);
      selftestStepCounter++;
    }
    //and do 4 microsteps back the other way
    selftestStepCounter = 0;
    digitalWrite(dir_pin, HIGH);
    while (selftestStepCounter < 4) {
      commutate(98);
      selftestStepCounter++;
    }
    selftestCycleCounter++;
  }
  digitalWrite(dir_pin, LOW);
  return;
}

void stepper_beep_low_fast(void) {
  //Stepper user feedback utility: short _beep_ tone.
  //Assumptions: Bolt current is ON.
  //Limitations: Blocks, but briefly, and when pulling the trigger is likely bad anyway.

  selftestCycleCounter = 0;
  while (selftestCycleCounter < 60) {   // Brief
    //do 4 microsteps one direction
    selftestStepCounter = 0;
    digitalWrite(dir_pin, LOW);
    while (selftestStepCounter < 4) {
      commutate(210);
      selftestStepCounter++;
    }
    //and do 4 microsteps back the other way
    selftestStepCounter = 0;
    digitalWrite(dir_pin, HIGH);
    while (selftestStepCounter < 4) {
      commutate(210);
      selftestStepCounter++;
    }
    selftestCycleCounter++;
  }
  digitalWrite(dir_pin, LOW);
  return;
}

void stepper_beep_high_fast(void) {
  //Stepper user feedback utility: ^beep^ tone.
  //Assumptions: Bolt current is ON.
  //Limitations: Blocks, but briefly, and when pulling the trigger is likely bad anyway.

  selftestCycleCounter = 0;
  while (selftestCycleCounter < 90) {   // Brief
    //do 4 microsteps one direction
    selftestStepCounter = 0;
    digitalWrite(dir_pin, LOW);
    while (selftestStepCounter < 4) {
      commutate(98);
      selftestStepCounter++;
    }
    //and do 4 microsteps back the other way
    selftestStepCounter = 0;
    digitalWrite(dir_pin, HIGH);
    while (selftestStepCounter < 4) {
      commutate(98);
      selftestStepCounter++;
    }
    selftestCycleCounter++;
  }
  digitalWrite(dir_pin, LOW);
  return;
}

void stepper_alarm(void) {
  //Stepper user feedback utility: Alarm melody.
  //Assumptions: Bolt current is ON.
  //Limitations: Blocks. Subordinate calls in this block. Don't use while waiting for trigger events. Make another trigger-aware noise generating function for that.

  selftestAlarmCounter = 0;
  while (selftestAlarmCounter < 3) {     //repeat alarm 3 times
    stepper_beep_high();
    stepper_beep_low();
    stepper_beep_low();
    stepper_beep_low();                  // "Sad Raymond"
    selftestAlarmCounter++;
  }
  return;
}

void stepper_fast_alarm(void) {
  //Stepper user feedback utility: "^beep^-_beep_" alarm.
  //Assumptions: Bolt current is ON.
  //Limitations: Blocks. Subordinate calls in this block. Don't use while waiting for trigger events. Make another trigger-aware noise generating function for that.
  stepper_beep_high_fast();
  stepper_beep_low_fast();            // "Dead cellphone"
  return;
}


void stepper_code_blipper(int code_major, int code_minor) {
  //Stepper user feedback utility: Blip out error code (or other value) with growls.
  //Assumptions: Bolt current is ON.
  //Limitations: Blocks. Subordinate calls in this block. Don't use while waiting for trigger events. Make another trigger-aware noise generating function for that.
  while (code_major > 0) {     //Growl code_major times
    stepper_growl();
    delay(200);                //With 0.2s between growls
    code_major--;
  }
  delay(700);                  //0.7s silence between major and minor
  while (code_minor > 0) {     //Growl code_minor times
    stepper_growl();
    delay(200);                //With 0.2s between growls
    code_minor--;
  }
  return;
}


void die(int major, int minor) {
  //Terminal error handler: Invoked when further operation is unsafe or impossible. Play alarm, then blip out error code. Loop forever, blocking everything else.
  //Zero flywheel drive throttle to ensure anything that happened during the fault gets shut off
  OCR1A = 230;
  OCR1B = 230;
  //Mute certain ISRs that might have been left on by code where fault occurred
  disableGovernorInterrupt();
  disableTachInterrupts();
  //Turn bolt motor current on, if it wasn't already
  digitalWrite(enable_pin, LOW);
  //Wait for 8825 to stabilize, if necessary
  delay(20);
  while (1) {
    stepper_alarm();
    delay(700);
    stepper_code_blipper(major, minor);
    delay(700);
  } //Loop for eternity, there's nothing more to be done
}

void error(int major, int minor) {
  //Non-terminal error handler: Invoked when further operation is possible. Play alarm, then blip out error code, once.
  //Turn bolt motor current on, if it wasn't already
  digitalWrite(enable_pin, LOW);
  //Wait for 8825 to stabilize, if necessary
  delay(20);
  stepper_alarm();
  delay(700);
  stepper_code_blipper(major, minor);
  delay(700);
  digitalWrite(enable_pin, HIGH);
  return;
}

void selftest() {
  //Power-on selftest routine (new 14-Feb-20).
  delay(500); //allow some time for drives to boot up and all voltages to stabilize and so forth (depending on when this is called)

  //Is the bolt drive alive? Turn motor current on and growl
  digitalWrite(enable_pin, LOW);
  //Wait for 8825 to stabilize, likely not required.
  delay(20);
  //Growl once (Like old world t19s do) - It is hard to self-detect if the pusher inverter is working aside from moving it later on, and if it is not, we have no voice to warn anyone anyway!!
  //So the single growl is the telltale.
  stepper_growl();
  delay(20);                        //Wait any vibration to subside (so motor doesn't bounce if unlocked immediately after growling)
  //Turn current off.
  digitalWrite(enable_pin, HIGH);
  //Rest of bolt checks after flywheel drive rotation check so we can do reverseBoltToSwitch with flywheels spinning at low speed to eject any darts or debris,
  //if the act of resetting the bolt dislodges or feeds anything accidentally. Sometimes it does.

  //Trigger invalid state detection:
  //The complementary trigger input must have one line HIGH and one LOW at any given time unless the switch is moving.
  //Two highs is a broken or fouled switch, a bad connection or most likely, an unplugged cable.
  //Two lows is probably a short or the wrong device plugged into the trigger connector.

  if (digitalRead(trig_a_pin) && digitalRead(trig_b_pin)) {
    die(2, 1); //Major 2 minor 1: Trigger fault both inputs high
  }
  if (!digitalRead(trig_a_pin) && !digitalRead(trig_b_pin)) {
    die(2, 2); //Major 2 minor 2: Trigger fault both inputs low
  }

  //End trigger checks.

  //Flywheel drive checks.
  //BEFORE enabling edge-triggered tach input, attempt to verify line integrity:
  //Drive channel 0 (M1F)
  selftestTachIntegChecks = 0;                                     //Zero counter first
  selftestTachState = digitalRead(2);                              //Capture initial logic state
  while (selftestTachIntegChecks < selftestTachIntegCheckCount) {  //Loop until enough checks done
    if (selftestTachState == digitalRead(2)) {                     //Same logic state?
      selftestTachIntegChecks++;                                   //Yes - Increment check counter
      delayMicroseconds(selftestTachIntegPollTime);                //Wait 1 polling period before next read
    } else {                                                       //Landed here: Pin changed state while we were looking at it. FAILED
      die(3, 1);                                                   //Major3 minor1 = Tach integrity fault drive 0 (M1F)
    }
  }
  //Drive channel 1 (M2F)
  selftestTachIntegChecks = 0;                                     //Zero counter first
  selftestTachState = digitalRead(3);                              //Capture initial logic state
  while (selftestTachIntegChecks < selftestTachIntegCheckCount) {  //Loop until enough checks done
    if (selftestTachState == digitalRead(3)) {                     //Same logic state?
      selftestTachIntegChecks++;                                   //Yes - Increment check counter
      delayMicroseconds(selftestTachIntegPollTime);                //Wait 1 polling period before next read
    } else {                                                       //Landed here: Pin changed state while we were looking at it. FAILED
      die(3, 2);                                                   //Major3 minor2 = Tach integrity fault drive 1 (M2F)
    }
  }
  //End tach integrity checks.
  //Cleared to enable interrupts.
  enableTachInterrupts();
  //Now try to catch seized motors/wheels, singlephasing (causes a no-start from 0 rpm on simonk), and failed inverters. Also catches if it BORFs when trying to start (maybe dead battery).
  //Also obviously fails if the tach channels were crossed with the throttle channels they expect to control them, on someone's hand wired board.
  //Spin up each drive one at a time until satisfied that it is in fact turning (above the floor speed).
  //There is no set minimum time to be commanding torque for, we just have to make it spin and stay spinning enough to build up a clean history of tach readings within the fail time.
  drive0TachValid = false;                                         //Zero tach validity flags to be sure
  drive1TachValid = false;
  //Drive 0 - Motor start NOW
  OCR1A = 500;                                                     //Floored throttle
  selftestTimeStartedTaching = millis();                           //Record start timestamp
  while (!drive0TachValid) {                                       //Loop while still invalid
    if ((millis() - selftestTimeStartedTaching) > selftestTachTimeout) {
      die(3, 3);                                                   //Hit timeout waiting for signal = Code 33, drive 0 no speed feedback
    }
  }                                                                //Fall through to here iff the tach is valid and there is time left
  goodTachCount = 0;                                               //Clear
  selftestTachMulligans = 0;                                       //Clear
  while (goodTachCount < selftestGoodTachs) {
    delayMicroseconds(selftestTachSpinFloor);                      //Abuse pulse period we're looking for as appropriate polling period to check for it
    if (pulseLength0 < selftestTachSpinFloor) {
      goodTachCount++;                                             //Above the floor? another tally mark on the wall
    } else {
      selftestTachMulligans++;                                     //But also track how many bad pulses we have got in this cycle
    }
    if (selftestTachMulligans > selftestMaxTachMulligans) {        //Too many bad tachs in this cycle, start over
      goodTachCount = 0;
      selftestTachMulligans = 0;
    }
    if ((millis() - selftestTimeStartedTaching) > selftestTachTimeout) {
      die(3, 4);                                                   //Timed out = code 34: drive 0 no rotation
    }
  }                                                                //Reached this point = drive 0 PASSES rotation check.
  OCR1A = 230;                                                     //Drive 0 shutdown.
  delay(150);                                                      //Wait before blipping next drive to reduce the chances of fluke pass if tach crosstalk, etc. and motor still turning fast
  drive1TachValid = false;                                         //Be sure
  //Drive 1 - Motor start NOW
  OCR1B = 500;                                                     //Floored throttle
  selftestTimeStartedTaching = millis();                           //Record start timestamp
  while (!drive1TachValid) {                                       //Loop while still invalid
    if ((millis() - selftestTimeStartedTaching) > selftestTachTimeout) {
      die(3, 5);                                                   //Hit timeout waiting for signal = Code 35, drive 1 no speed feedback
    }
  }                                                                //Fall through to here iff the tach is valid and there is time left
  goodTachCount = 0;                                               //Clear
  selftestTachMulligans = 0;                                       //Clear
  while (goodTachCount < selftestGoodTachs) {
    delayMicroseconds(selftestTachSpinFloor);                      //Abuse pulse period we're looking for as appropriate polling period to check for it
    if (pulseLength1 < selftestTachSpinFloor) {
      goodTachCount++;                                             //Above the floor? another tally mark on the wall
    } else {
      selftestTachMulligans++;                                     //But also track how many bad pulses we have got in this cycle
    }
    if (selftestTachMulligans > selftestMaxTachMulligans) {        //Too many bad tachs in this cycle, start over
      goodTachCount = 0;
      selftestTachMulligans = 0;
    }
    if ((millis() - selftestTimeStartedTaching) > selftestTachTimeout) {
      die(3, 6);                                                   //Timed out = code 36: drive 1 no rotation
    }
  }                                                                //Reached this point = drive 1 PASSES rotation check.
  OCR1B = 230;                                                     //Drive 1 shutdown.
  disableTachInterrupts();                                         //Done, take ISRs out of gear
  drive0TachValid = false;                                         //Clear for later
  drive1TachValid = false;
  //End flywheel drive checks.

  //Attempt to establish bolt home position. We want to do this with flywheel drives enabled at a low speed so as to spit out any accidentally fed dart or trash that dislodges from cage
  //when running the bolt home. Since the drives haven't had a speed set, they are at default ~5000rpm (for 14 pole) and anything mistakenly launched will go about 36fps.
  //Turn bolt motor current on
  digitalWrite(enable_pin, LOW);
  //Wait for 8825 to stabilize, likely not required.
  delay(20);
  //Enable flywheel drives (Avoid latency between this and reverseBoltToSwitch return if already home to avoid the drives having time to turn current on for a moment if unnecessary)
  OCR1A = 500;                      //Max torque
  OCR1B = 500;                      //Nb: 5000rpm governor by default
  if (!reverseBoltToSwitch()) {     //Attempt to find home with up to 1 full revolution (Returns true immediately without motion if already home)
    die(1, 1);                      //Major 1 minor 1: Bolt drive fault, homing failed
  }
  OCR1A = 230;                      //Disable
  OCR1B = 230;
  //Turn bolt motor current off.
  digitalWrite(enable_pin, HIGH);
  //End bolt checks.
  //End POST.
}

void speedtrap(void) {
  //Drive critical overspeed trip, and final setpoint checker.
  //This is a ballistic safety measure as well as a safeguard against FlyShot signalling mishaps. Blocks anything that would cause considerably unexpected velocity.
  //Call this AFTER locking down the speed setpoint, pushing the last FlyShot updates to drives, and calculating control parameters.
  //Spins both drives while sniffing both tachs (through a noise filter) and waiting for the speed to come in range.
  //The speed should not grossly overshoot i.e. buffer < (setpoint - offsetmargin) at any point instantaneously (this is immediate failure).
  //Allow up to selftestTachTimeout milliseconds for the speed to be in range.
  goodTachCount = 0;                             //Clear
  drive0TachValid = 0;
  drive1TachValid = 0;
  speedtrap_buf0 = 0;
  speedtrap_buf1 = 0;
  enableTachInterrupts();                        //Established to be safe after POST
  OCR1A = 500;                                   //Start motors
  OCR1B = 500;
  delay(30);                                     //Tachs should be valid after this delay.
  if (!drive0TachValid) {                        //No signal?
    die(4, 1);                                   //Code 41, overspeed detector tach signal loss drive 0
  }
  if (!drive1TachValid) {                        //No signal?
    die(4, 2);                                   //Code 42, overspeed detector tach signal loss drive 1
  }
  //Reached this point: Drives are still present and alive, emitting tach pulses and they are being received, start measuring them
  selftestTimeStartedTaching = millis();         //Record time of measurement start
  while (goodTachCount < selftestGoodTachs) {    //Main loop (any exit from this is a success - all failures call die() and hang forever)
    delayMicroseconds(speedSetpoint);            //Abuse target period as polling period
    if (pulseLength0 < (speedSetpoint - speedtrap_overspeedTripMargin)) {
      die(4, 3);                                 //Instantaneous per-cycle overspeed trip. Code 43, drive 0 critical overspeed
    }
    if (pulseLength1 < (speedSetpoint - speedtrap_overspeedTripMargin)) {
      die(4, 4);                                 //Instantaneous per-cycle overspeed trip. Code 44, drive 1 critical overspeed
    }
    //ONLY get to here if we didn't see a huge overshoot and abort already.
    //Filter speeds into averaged buffers (Dividing is bad juju but /2 should compile as a LSR/ROR and be fast, I believe)
    speedtrap_buf0 = (speedtrap_buf0 + pulseLength0) / 2;
    speedtrap_buf1 = (speedtrap_buf1 + pulseLength1) / 2;
    if ((speedtrap_buf0 < (speedSetpoint + speedtrap_offsetMargin)) && (speedtrap_buf0 > (speedSetpoint - speedtrap_offsetMargin))) {
      //Less (faster than) a margin above (slower than) the setpoint. Greater (slower than) a margin below (faster than) the setpoint: drive 0 - speed OK this cycle.
      //So check drive 1:
      if ((speedtrap_buf1 < (speedSetpoint + speedtrap_offsetMargin)) && (speedtrap_buf1 > (speedSetpoint - speedtrap_offsetMargin))) {
        //Less (faster than) a margin above (slower than) the setpoint. Greater (slower than) a margin below (faster than) the setpoint: drive 1 - speed OK this cycle.
        //Both OK, increment.
        goodTachCount++;
      } else {
        //Drive 1 not in range, reset
        goodTachCount = 0;
      }
    } else {
      //Drive 0 not in range, reset
      goodTachCount = 0;
    }
    //Bug/Misfeature: Error reporting as to which drive is the problem and which direction the error was is uniquely lacking for this among all selftest/error reporting functions here.
    //The problem is all the valid combinations of failing states. Needs a bit of addressing or else move to sampling each speed serially, like the POST rotation checker.
    //That cuts the mess and variables by a lot
    //I just wanted to have one quick, clean sounding rev after speed updating, versus delay the user from shooting intially after boot while we screw around revving up motors and measuring speeds.
    if ((millis() - selftestTimeStartedTaching) > selftestTachTimeout) {
      //Still looping unsuccessfully and hit timeout
      die(4, 5);                                     //Code 45, flywheel drive speed verification failed (Why do I have the suspicion this will be a dreaded, baffling, common one)
    }
  }
  OCR1A = 230;                                       //Shut down motors
  OCR1B = 230;
  disableTachInterrupts();                           //Mute tach ISRs before returning
}

void fire() {
  //loop called to fire a shot
  //set distance to run bolt forward
  stepsToGo = 720;
  //if continuing a previous instance add 80 steps
  if (!boltHomed) {
    stepsToGo += 80;
  }
  //get start point for first ramp
  if (currSpeed < startSpeed) {
    //bolt already running
    stepdelay = currSpeed;
  } else {
    //bolt not running
    stepdelay = startSpeed;
  }
  // do first ramp if speed below shiftpoint
  while (stepdelay > shiftSpeed) {
    commutate(stepdelay);
    stepdelay = stepdelay * (1 - accelPhaseOne * stepdelay * stepdelay);
    stepsToGo--;
  }
  //do second speed ramp if speed above shift but below running speed
  while (stepdelay > runSpeed) {
    commutate(stepdelay);
    stepdelay = stepdelay * (1 - accelPhaseTwo * stepdelay * stepdelay);
    stepsToGo--;
  }
  //do constant speed run until out of steps
  while (stepsToGo > 0) {
    commutate(stepdelay);
    stepsToGo--;
  }
  currSpeed = stepdelay;
  boltHomed = 0;
}
//Interrupt mute/unmute functions for convenience.

void enableTachInterrupts() {
  cli();
  EIMSK = 0b00000011; //INT0 and INT1 on
  sei();
}

void disableTachInterrupts() {
  cli();
  EIMSK = 0b00000000; //INT0 and INT1 off
  sei();
}

void enableGovernorInterrupt() {
  cli();
  TIMSK1 = 0b00000010; //set OCIE1A
  sei();
}

void disableGovernorInterrupt() {
  cli();
  TIMSK1 = 0b00000000; //clear OCIE1A to disable
  sei();
}

boolean setGovernorBoth(void) {
  if(governor > 0x7fff) {return false;}           //Die: Cannot set governor to more than 0x7fff
  disableTachInterrupts();                        //Ensure tach capture is off to reduce overhead
  governor |= 0x8000;                             //Set msb (flag bit: this is a governor update frame) to 1 (n.b.: 7fff becomes ffff)
  gov_currentBit = 15;                            //Prepare for enabling interrupt routine
  save_ocr1a = OCR1A;                             //Save throttle command (restored by ISR during the cleanup cycle after packet is over)
  save_ocr1b = OCR1B; 
  enableGovernorInterrupt();                      //Enable ISR
  delay(10);                                      //A few idle cycles (timer is still creating throttle pulses)
  gov_enable_strobe = true;                       //Push the start button
  while((gov_enable_strobe || gov_packet_active)){
     delayMicroseconds(1000);                     //Either the strobe state hasn't been checked yet or the packet is still transmitting: block until done
  }                                               
 delay(10);                                       //Ensure some clean throttle pulses get fired between governor packets so drive doesn't disarm/balk
 disableGovernorInterrupt();                      //Mute ISR
 return true;
}
void updateSpeedFixed(unsigned long setpointRPM) {
  //Set setpointRPM from limitRPM only, update governor, push governor update, and update tach control parameters.
  unsigned long setpointGovernor = (320000000/(setpointRPM * motorPolepairs));   //Convert to governor, which is 8 * TIMING_MAX (i.e. TIMING_MAX * CPU_MHZ (=16) / 2) at full resolution
  speedSetpoint = ((3 * setpointGovernor)/16);                     //Convert to tach: 6 * TIMING_MAX / 4
  //Update margin for Orthomatic control. For now trying linear with RPM request like this
  speedOffsetMargin = map(setpointRPM, minRPM, maxRPM, speedOffsetMarginMin, speedOffsetMarginMax);
  //Push governor update
  governor = setpointGovernor;
  setGovernorBoth();                                               //Eventually this ought to check the return value
}

void setup(){
  //pin 2 and 3 tach external interrupt - Set AVR internal pullups.
  //This is measure/attempt to prevent noise on a floated pin from causing a race condition and LOCKING UP THE BOARD as soon as tachs are enabled if the CABLE GETS UNPLUGGED.
  //Nevertheless - DO NOT LEAVE TACH LINES ON ACTIVE ISRS OPEN!!!! If using a non-tach  or 1 motor (Nitron?) application, use firmware without unused tach ISRs enabled.
  //Nb: Tach line is facing an OUTPUT DRIVER on another AVR. DO NOT ATTEMPT TO DRIVE IT. That might get kind of smokey.
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  //pin 0 and 1 trigger switch
  //(If the trigger doesn't work, flip the connector)
  pinMode(trig_a_pin, INPUT);
  pinMode(trig_b_pin, INPUT);
 
  //pin 9,10 flywheel motor controller PWM throttle signal
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  //set up Timer1 (2 channel throttle generation, 400Hz, 1-2ms PWM protocol)
  //fast PWM prescaler 64 (250kHz)
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  ICR1 = 624; //400Hz
  OCR1A = 230;
  OCR1B = 230; //write 920us low throttle
  //set up INT0 and INT1 interrupts - tach
  cli();
  EICRA = _BV(ISC11) | _BV(ISC10) | _BV(ISC01) | _BV(ISC00); //INT0 and INT1 triggered on rising edge
  EIMSK = 0b00000011; //INT0 and INT1 on
  sei();

  //bolt drive hardware setup
  //DRV8825 stepper driver
  //pin 19: direction
  pinMode(dir_pin, OUTPUT);
  //drive low
  digitalWrite(dir_pin, LOW);
  //pin 18: step
  pinMode(step_pin, OUTPUT);
  //pin 4: bolt limit switch (has 1k external pullup)
  pinMode(limit_pin, INPUT);
  //pin 5,6,7: microstep mode select M2,M1,M0
  pinMode(m2_pin, OUTPUT);
  pinMode(m1_pin, OUTPUT);
  pinMode(m0_pin, OUTPUT);
  //pin 8: enable
  pinMode(enable_pin, OUTPUT);
  //turn motor current off
  digitalWrite(enable_pin, HIGH);

  //configure stepper driver parameters
  //4:1 microstep
  digitalWrite(m2_pin, LOW);
  digitalWrite(m1_pin, HIGH);
  digitalWrite(m0_pin, LOW);
}

void loop(){
  if (firstRun) {
    //Wait for SimonK boot and arm (TBD value, Selftest already waits 500ms)
    delay(500);
    //Do POST
    selftest();
    //Wait a moment and...
    delay(500);
    //default speed is tournament lock setting
    delay(100); //Some anti-noise buffer
    while (gov_update_repeats) {
      // Set speed to 20K RPM.
      updateSpeedFixed(20000); //Nb: updateGovernorBoth blocks while a packet is being transmitted, thus so does this call.
      delay(20); //Some anti-noise buffer
      gov_update_repeats--;
    }
    delay(100); //Same here
    ////Speed setpoint and associated parameters fed to STC stuff is now set in concrete for the remainder of this uptime.
    //Verify set speed. Nb: Since this happens seamlessly after user-adjusting speed (the motors were still running) there is no "blip"
    speedtrap();                 //Bust any wrong flywheel speeds now before firing is allowed. Starts motors if they weren't already and shuts them down afterward.
    //Fell through = Cleared for launch.
    //Initialize fire control and bolt speed
    runSpeed = runSpeedMax;      // Run the bolt at max speed.
    //clear flag
    firstRun = 0;
  }



  disableGovernorInterrupt();    //It shouldn't have been on - make very sure
  //initial debounce on trigger from idle state. Safety measure.
  prevTrigState = currTrigState;
  currTrigState = (digitalRead(trig_a_pin) && !digitalRead(trig_b_pin));
  if (currTrigState && prevTrigState) {
    //turn motor current on
    digitalWrite(enable_pin, LOW);
    //enable tachs
    enableTachInterrupts();
    //start flywheels
    OCR1A = 500; //go
    OCR1B = 500; //go


    ////Speed feedback STC section.
    //This project went at one point by "Orthomatic".

    //Record trigger down event timestamp
    lastTriggerDown = millis();
    //Wait for both drives to have vaild tach. This effectively checks that they are actively toggling their tach lines. These flags are cleared before we get here,
    //so the only thing that would have turned them on is the tach ISR.
    while (!(drive0TachValid && drive1TachValid && ((millis() - lastTriggerDown) <= failTime))) {
      delay(1);

    }
    if ((millis() - lastTriggerDown) >= failTime) {
      //Whoops. We exited the invalid-tach trap because one or more drives failed to produce any speed feedback in a reasonable time. Perhaps something got unplugged.
      //Disable tach:
      disableTachInterrupts();
      //Shut down drives:
      OCR1A = 230;
      OCR1B = 230;
      //Set pending NoTach alarm (Single Low beep).
      //Then trap trigger down state here. Require a trigger reset to reattempt.
      alarm_want = 1;
      while (READTRIGGER()) {
        delay(5);
      }
      goodTachCount = 0;
      lastTriggerUp = millis();
    } else {

      //We exited the invalid-tach trap because both tachs went valid. This is what's supposed to happen.
      //Wait for drives to reach speed and stay there for goodTachsToFire *consecutive* reads, and then fire. (0.95 - Robustness improvement)
      while ((goodTachCount < goodTachsToFire) && ((millis() - lastTriggerDown) <= failTime)) {
        //Increment goodTachCount every time both speeds are in range, with poll cycle about as long as the target period (little longer in practice)
        if ((pulseLength0 <= (speedSetpoint + speedOffsetMargin)) /*&& (pulseLength1 <= (speedSetpoint + 4*speedOffsetMargin))*/) {
          goodTachCount++;
        } else {
          goodTachCount = 0;                     //Strictly reject any out of range reads (This is in the range of 3-7 reads in a row we're looking for to fire so no sense in allowing "mulligans")
        }
        delayMicroseconds(speedSetpoint);        //Abuse target signal period as polling period
      }
      //We returned. But why? If it's a timeout, then goodTachCount will not have reached goodTachsToFire.
      if (goodTachCount < goodTachsToFire) {
          // We are getting stuck in here, why?
        //Whoops. We exited because: Drives failed to reach speed setpoint in a reasonable time.
        //Disable tach:
        disableTachInterrupts();
        //Shut down drives:
        OCR1A = 230;
        OCR1B = 230;
        //Set pending SpeedFail alarm ("Dead phone"), then trap trigger down state here. Require a trigger reset to reattempt.
        alarm_want = 2;
        while (READTRIGGER()) {
          delay(1);
        }
        goodTachCount = 0;
        lastTriggerUp = millis();
      } else {
        //Successful acceleration: start firing.
        //Already know speed is good at this point, mute tach ISRs
        disableTachInterrupts();
        fire();

        //first sealed-in shot is over. Check trigger *quickly* for downness (Nb: We may be commutating the pusher right now) and fire again if down and called for. Trigger is PD0, PD1.
        //This second fire() call may execute 0 times if isBurst is true and burstCounter starts as 1.
        while (READTRIGGER()) {
          fire();
        }
        //Here, firing is definitely OVER. Get the bolt back safely to home position (N.b.: Flywheel Drives are still enabled right now, so any spurious feed from this is safe)
        if (!decelerateBoltToSwitch()) {
          reverseBoltToSwitch();
        }
        //Reset tach cycle counter
        goodTachCount = 0;
        //Shut down drives only NOW:
        OCR1A = 230;
        OCR1B = 230;
        lastTriggerUp = millis(); //Not ACTUALLY trigger up any more due to the disconnector trap. A burst disconnect is a pseudo trigger up.
      }
    }
  } else {
    //Trigger UP high priority loop tasks - keep wheel drives off, flag tachs as invalid, turn off bolt current after boltShutdownTime ms.
    OCR1A = 230; //shutdown
    OCR1B = 230; //shutdown
    goodTachCount = 0;
    drive0TachValid = false;
    drive1TachValid = false;
    if ((millis() - lastTriggerUp) > boltShutdownTime) {
      //Play any pending STC alarm and clear it, then turn bolt motor current off.
      if (alarm_want) {
        if (alarm_want == 1) {
          stepper_beep_low();
        } else {
          stepper_fast_alarm();
        }
        alarm_want = 0;
        delay(60);          //If motor was just beeping, wait for vibration to decay before unlocking
      }
      digitalWrite(enable_pin, HIGH);
    }
  }
}

////Tach ISRs, take input pulse length, stuff it into the corresponding variable and flag it as fresh.
////TBD: micros() needs to go away because the resolution is garbage (but functional). Probably going to use Timer2.

ISR(INT1_vect) {
  //pin 2 tach input handler - motor 0
  lastPulseTime0 = thisPulseTime0;
  thisPulseTime0 = micros();
  pulseLength0 = thisPulseTime0 - lastPulseTime0;
  drive0TachValid = true;
}

ISR(INT0_vect) {
  //pin 3 tach input handler - motor 1
  lastPulseTime1 = thisPulseTime1;
  thisPulseTime1 = micros();
  pulseLength1 = thisPulseTime1 - lastPulseTime1;
  drive1TachValid = true;
}

////2 channel Governor config ISR (Easy enough to make 2 independent ones for apps that need 2 independent speed controlled drives)

ISR(TIMER1_COMPA_vect) {
  //Timer1 channel A compare match: Fires when hitting OCR1A (which is the falling edge of a PWM high period that started at overflow)
  //We have from the compare match until at least gov_ocr_t0h (shortest pulsewidth) ticks after overflow (the next compare match) to calculate and update OCR1x.
  
  if(gov_enable_strobe) {
      gov_packet_active = true;  
      if((gov_currentBit + 1) > 0){                               //Start shifting out packet, MSB first
          if(governor & (0x0001 << gov_currentBit)) {             //Mask the packet with a 1 LSLed i times to select the desired bit (start at 15)
             OCR1A = gov_ocr_t1h;                                 //T1H
             OCR1B = gov_ocr_t1h;
           } else {                                               //else: is 0
             OCR1A = gov_ocr_t0h;                                 //T0H
             OCR1B = gov_ocr_t0h;
           }                                                      //Nb: T0L/T1L is just the remainder of the period at 400Hz
           gov_currentBit--;                                      //ends after running with gov_currentBit = 0, thus leaving it at -1
      } else {                                                    //last data cycle has run through with gov_currentBit at 0, then decremented it to -1 after - do cleanup
          OCR1A = save_ocr1a;                                     //restore saved throttles (important to do this NOW!! before the next PWM cycle needs them!)
          OCR1B = save_ocr1b;
          gov_packet_active = false;                              //Active packet flag to false
          gov_enable_strobe = false;                              //Disable
      }
  }
  //Here is where the inactive state lands when this ISR is on and waiting for gov_enable_strobe.
  //Do nothing. Don't mod anything because foreground code is probably loading variables right now.
}
