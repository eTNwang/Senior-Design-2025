// DR_P2P_2023_OOPS
// OOPS --> doesn't work well --> Fix it!
// dead reckoning & simple point-to-point navigation
// BDK:2023-09-27 (updated from 2022)
// ESE421
// "#define" for constant values to avoid global variables
// this isn't great coding practice but better than globals!
//
////////////////////
// BEGIN PARAMETERS THAT MAY BE CHANGED
// waypoints
#define wayN 4
#define wayXvals 0.0, 40.0, 0.0, 40.0
#define wayYvals 0.0, 40.0, 40.0, 0.0
/*
const float wayXvalsNew[] = {
    0.0, 5.99212471, 11.9842461, 17.77023537, 23.34751064, 28.70527721, 34.09615637, 39.25385049, 
    44.27126583, 48.88520481, 53.28242597, 56.85424689, 60.59396917, 63.91556723, 66.90751253, 
    69.77104678, 72.48337587, 75.5125561, 77.96802845, 80.6668497, 83.04325846, 85.02103387, 
    86.67719476, 88.00088684, 89.00238634, 89.50000107, 89.5, 89.50003341, 88.99144571, 
    87.991733, 86.45938519, 84.95551024, 82.82370169, 80.51320772, 77.97233632, 75.03604903, 
    72.4850775, 69.49522166, 66.47608695, 63.4764561, 60.08154279, 56.54562188, 52.81637543, 
    48.57666933, 43.9473693, 38.75533278, 33.61734195, 28.23778194, 22.758409, 17.08114543, 
    11.29683123, 5.30470616, -0.6976646, -6.2527867, -11.83086374, -17.19721709, -22.56544857, 
    -27.5373791, -32.28750166, -36.88243729, -40.97364071, -44.52546753, -47.98530727, 
    -50.98760587, -53.98947773, -57.02975794, -59.63328833, -62.524499, -64.98468443, 
    -67.48615692, -69.63720279, -71.46829379, -72.96847138, -74.53080065, -75.00638793, 
    -75.49999976, -75.49999748, -74.99981571, -73.98222953, -73.00359235, -71.49791091, 
    -69.41691363, -67.25141296, -64.62424888, -61.96715728, -59.27063842, -56.5163846, 
    -53.47599563, -50.47624231, -47.47300407, -43.81446181, -40.19630745, -36.17683752, 
    -31.51522913, -26.85799914, -21.77146146, -16.52892943, -11.16816453, -5.58072187, 0.0
};

const float wayYvalsNew[] = {
    0.0, 0.000000000000563993297, 0.00000801259365, 0.497662989, 1.49919812, 
    3.03067413, 4.48220929, 6.4967029, 8.81739616, 11.4450163, 13.9794801, 
    16.9056196, 19.5448556, 22.4759573, 25.4761367, 28.7624412, 32.4698988, 
    36.0193494, 40.4410244, 44.7176328, 49.7118776, 54.8847809, 60.1909013, 
    65.6347348, 71.2120248, 76.9980307, 82.9901559, 88.9822944, 94.7637552, 
    100.341785, 105.699191, 111.06839, 116.177491, 121.239578, 125.57358, 
    129.487296, 133.512071, 137.115728, 140.072749, 143.065243, 145.964995, 
    148.984581, 151.641851, 154.45271, 157.024594, 158.959344, 161.021406, 
    162.500268, 163.73816, 164.498302, 165.0, 165.0, 165.024736, 163.969719, 
    162.970119, 161.459373, 159.953162, 157.490212, 155.197165, 152.557646, 
    150.023192, 146.995656, 144.175914, 141.185805, 138.195652, 134.956626, 
    130.973525, 127.306886, 122.593438, 117.932104, 112.830973, 107.597311, 
    102.22658, 96.8815936, 91.0864636, 85.2987996, 79.306676, 73.5217334, 
    67.9511067, 62.3643468, 56.9958957, 51.8657483, 46.7706618, 42.1599504, 
    37.8840363, 33.9873539, 30.1671075, 26.9233984, 23.9310328, 20.9404582, 
    18.0037791, 15.1622767, 12.5096884, 10.0049593, 7.49039108, 5.30411743, 
    3.49444062, 1.9702032, 0.993214311, 0.0
};
*/

const float wayXvalsNew[] = {
    0.0, -5.02529831, -10.8881119, -15.95268628, -21.94843882, -26.99957244, 
    -33.0381776, -37.92985962, -43.88318053, -49.01683583, -46.94584067, 
    -47.00000093, -48.00880175, -51.25975405, -57.03275255, -64.53179727, 
    -74.60009128, -85.67736455, -97.18613766, -103.89182846, -108.97068308, 
    -115.0039603, -120.02844104, -125.81251365, -130.95348439, -136.95420456, 
    -142.00350411, -148.04103531, -152.93021251, -158.89026997, -163.97016583, 
    -170.00352873, -175.02773134, -180.89202536, -184.61485412, -186.33290736, 
    -190.55276584, -196.01825925, -203.72597048, -213.34671171, -224.01729622, 
    -225.00000005, -225.0, -224.99998917, -219.81863524, -208.32173684, 
    -196.82485897, -185.32798111, -173.83110324, -162.33422537, -150.8373475, 
    -139.34046964, -127.84359177, -116.3467139, -104.84983603, -93.35295817, 
    -81.8560803, -70.35917104, -64.00007373, -64.0, -64.0, -64.0117956, 
    -73.70398309, -84.28779001, -93.24413901, -98.95628682, -103.99312673, 
    -106.98483463, -107.00000001, -107.0, -107.0, -107.0, -107.00000009, 
    -106.16745164, -95.12548208, -83.62863233, -72.5460775, -61.0510052, 
    -50.37490247, -39.70401458, -29.33456184, -19.42923372, -10.35994937, 
    -2.05430315, 3.98760218, 10.02276064, 14.97275019, 19.76213734, 
    23.60424173, 25.99272706, 27.00522549, 28.00000155, 27.9836268, 
    26.40467071, 24.68860044, 21.47224082, 16.97297995, 11.98119627, 
    6.49609332, 0.0, -223.0, -214.72487283, -206.44974566, -198.17461848, 
    -189.89949131, -181.62436414, -173.34923697, -165.07410979, 
    -156.79898262, -148.52385545, -140.24872828, -131.97360111, 
    -123.69847393, -115.42334676, -107.14821959, -98.87309242, 
    -90.59796524, -82.32283807, -74.0477109, -65.77257917, -57.92381491, 
    -49.64188341, -41.82427301, -34.33135365, -26.88057509, -28.56879439, 
    -32.00656688, -35.9854529, -39.99367645, -44.0608572, -47.79707941, 
    -51.47343093, -54.96794293, -58.98861069, -63.00593862, -67.04105024, 
    -73.52578692, -81.82436607, -90.09949349, -98.37462753, -106.1902546, 
    -106.99999756, -107.0, -107.0, -107.0, -107.0, -107.00000074, 
    -107.0281128, -110.63106359, -114.03535725, -117.98729485, 
    -121.98761303, -126.06848136, -129.71510248, -133.33696734, 
    -136.98828907, -140.98710854, -145.06727257, -148.87444686, 
    -152.60348491, -156.02476225, -159.96516286, -163.9903545, 
    -167.91473922, -171.69759626, -175.32326971, -178.98105155, 
    -182.79757626, -183.9999963, -184.0, -184.0, -184.0, -184.0, 
    -184.0, -184.0, -184.0, -184.0, -184.0, -184.0, -184.0, -184.0, 
    -184.0, -184.0, -184.0, -184.0, -184.00000003, -183.99958968, 
    -184.96855904, -188.19358042, -192.1710456, -197.03877214, 
    -204.09659325, -211.12821658, -219.00608155, -224.98653375, 
    -224.9999998, -225.0, -225.0, -224.9999383, -223.0
};

const float wayYvalsNew[] = {
    0.0, 7.07880948, 13.78465995, 20.88196688, 26.96791693, 33.99952123, 
    40.03798229, 47.83962555, 53.94642621, 60.89797727, 71.5615689, 
    83.08087984, 94.15989872, 104.29358222, 111.05368177, 116.54213836, 
    119.98700076, 121.00001587, 120.97513587, 126.94780573, 133.9546304, 
    140.00403301, 147.08604034, 153.94166973, 160.88915002, 166.9701892, 
    174.00374824, 180.04254625, 187.8466357, 193.94754091, 200.95336268, 
    207.00233045, 214.08444202, 220.78947509, 229.58531213, 240.37057298, 
    249.53114349, 256.4135485, 261.84749035, 265.01723, 267.01710604, 
    278.10496523, 289.60184312, 301.09871969, 308.00017189, 308.0, 308.0, 
    308.0, 308.0, 308.0, 308.0, 308.0, 308.0, 308.0, 308.0, 308.0, 
    308.00003139, 302.86225084, 291.3654467, 279.86856883, 268.37310002, 
    265.98312356, 263.77873185, 258.99173224, 253.24016413, 245.02659154, 
    234.76891837, 223.27832221, 211.78144434, 200.28456647, 188.7876886, 
    177.29081069, 166.15306879, 165.00000009, 164.99993224, 163.99966793, 
    163.99530891, 162.01378244, 160.01966623, 157.29782121, 153.45539634, 
    149.01632691, 144.04739262, 138.01439633, 131.96629588, 124.26292887, 
    115.33711769, 105.43167041, 94.92413556, 83.84664828, 72.76182014, 
    61.27172364, 50.4288708, 39.64281251, 29.47819441, 19.84486352, 
    11.59786661, 5.00078812, 0.0, -15.0, -15.0, -15.0, -15.0, -15.0, 
    -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, -15.0, 
    -15.0, -15.0, -15.0, -15.0, -15.0, -15.000011, -13.97067989, 
    -13.98710697, -12.88256379, -10.99414378, -8.97483513, -2.99468845, 
    3.00394482, 7.86429709, 13.3629477, 18.10753906, 23.69977387, 
    29.02275399, 34.9704656, 39.77592046, 45.26998967, 50.06410773, 
    48.94350104, 48.99999942, 49.0, 48.99998574, 50.17225598, 
    58.09644905, 66.37157505, 74.64670222, 82.9218294, 91.19695657, 
    99.472083, 107.71914683, 112.97941096, 119.01648403, 123.91239343, 
    129.41142038, 134.13789405, 139.94877337, 145.39742145, 150.99043753, 
    155.82418219, 161.19854516, 166.77253926, 171.99920789, 178.0126134, 
    182.93447708, 188.39353214, 193.81678034, 198.95077721, 204.38844634, 
    209.98323849, 215.35630063, 207.90152342, 199.62639817, 191.351271, 
    183.07614383, 174.80101665, 166.52588948, 158.25076231, 149.97563514, 
    141.70050796, 133.42538079, 125.15025362, 116.87512645, 108.59999927, 
    100.3248721, 92.04974493, 83.77461776, 75.49949059, 67.22436343, 
    58.94906628, 51.07533606, 44.71101333, 39.95184451, 35.96330806, 
    32.96112699, 29.95905246, 28.99997822, 25.54072486, 17.27208147, 
    8.99695439, 0.72182722, -7.5532744, -15.0
};


// END PARAMETERS THAT MAY BE CHANGED
// (DON'T CHANGE PARAMETERS BELOW THIS LINE)
////////////////////
//
// heading gain (heading is in radians in this code)
#define KQ 20.0
// wheel geometry
#define wheelR 3.45
#define wheelL 14.05
// math constant
#define PI 3.14159265
// ROMI libraries
#include <Romi32U4.h>
Romi32U4LCD lcd;
Romi32U4Buzzer buzzer;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4Motors motors;
Romi32U4Encoders encoders;

const char beepButtonA[] PROGMEM = "!c32";
const char beepButtonB[] PROGMEM = "!e32";
const char beepButtonC[] PROGMEM = "!g32";

// This function watches for button presses.  If a button is
// pressed, it beeps a corresponding beep and it returns 'A',
// 'B', or 'C' depending on what button was pressed.  If no
// button was pressed, it returns 0.  This function is meant to
// be called repeatedly in a loop.
char buttonMonitor()
{
  if (buttonA.getSingleDebouncedPress())
  {
    buzzer.playFromProgramSpace(beepButtonA);
    return 'A';
  }

  if (buttonB.getSingleDebouncedPress())
  {
    buzzer.playFromProgramSpace(beepButtonB);
    return 'B';
  }

  if (buttonC.getSingleDebouncedPress())
  {
    buzzer.playFromProgramSpace(beepButtonC);
    return 'C';
  }

  return 0;
}

float getIRdcm(int pinIR) {
  //
  // rough distance estimate
  // used only for emergency stop
  //
  float dcm = 5000.0/max(50,analogRead(pinIR));
  return dcm;
}

// define data structure for robot pose
struct robotPose{
  double X;
  double Y;
  double Q;
};

// define data structure for navigation error
struct wayError{
  double errD;
  double errQ;
};

//
// waypoint management (track goal waypoint)
// return heading to get to next waypoint
//
struct wayError getWayE(struct robotPose currentP) {
  static int kGoal = 0; // kGoal = index of target waypoint
  // double wayX[] = {wayXvals}; // wayXvals, wayYvals defined at top
  // double wayY[] = {wayYvals};

  // const float wayX = wayXvalsNew;
  // const float wayY = wayYvalsNew;

  double dist2Go = sqrt(pow(currentP.X - wayXvalsNew[kGoal] / 2,2) + pow(currentP.Y - wayYvalsNew[kGoal] / 2,2));
  if (dist2Go < 10) {
    kGoal = (kGoal + 1) % 200;
    Serial.print(kGoal);
    Serial.println("Hit goal");
  }
  Serial.print("Going to point ");
  Serial.println(kGoal);

  //
  // find heading to point at the waypoint
  //
  double desiredQ = atan2(wayYvalsNew[kGoal] / 2-currentP.Y,wayXvalsNew[kGoal] / 2-currentP.X);
  //
  // return distance error and heading error
  //
  struct wayError E;
  E.errD = dist2Go;
  E.errQ = desiredQ - currentP.Q;
  return E;
}
//
// dead reckoning
//
struct robotPose getPose(float deltaT) {
// static variables to integrate encoders to get X, Y, Theta
// "hat" --> best estimate of values
  static double Xhat=0.0;
  static double Yhat=0.0;
  static double Qhat=0.0;
  int cL = encoders.getCountsAndResetLeft();
  int cR = encoders.getCountsAndResetRight();
  double vL = (6.2832*wheelR*cL)/(1440.0*deltaT);
  double vR = (6.2832*wheelR*cR)/(1440.0*deltaT);
  double V = 0.5*(vR+vL);
  double dV = vR - vL;
  Xhat += deltaT * V * cos(Qhat);
  Yhat += deltaT * V * sin(Qhat);
  Qhat += deltaT * dV / wheelL;
  struct robotPose measuredP;
  measuredP.X = Xhat;
  measuredP.Y = Yhat;
  measuredP.Q = Qhat;
  return measuredP;
}


///////////////////////////////////////////////////////////////////
void setup()
{  
  lcd.clear();
  Serial.begin(115200); // Serial COM at hight baud rate
  delay(2000);
}


///////////////////////////////////////////////////////////////////
void loop()
{
  //
  // static --> only initialized on first call to loop function
  //            & previous value stays in memory on subsequent calls
  // this code fixes the frame rate to deltaT
  //
  static unsigned long millisLast = millis();
  float deltaT = 0.05;
  while ((millis() - millisLast) < 1000*deltaT) {}
  millisLast = millis();
  //
  // motorsGo = 0 initially & after emergency stop
  // B button toggles value
  //
  static byte motorsGo = 0;
  switch(buttonMonitor()) {
    case 'A':
      break;
    case 'B':
      motorsGo = 1-motorsGo;
      break;
    case 'C':
      break;
    }
  //
  // emergency stop for front IR reading
  // (won't see obstacles when going backwards!)
  //
  if (getIRdcm(A3) < 15) {motorsGo = 0;}
  //
  // get current pose
  // robotPose struct has X, Y, and Theta
  //
  robotPose P = getPose(deltaT);
  //
  // get error to next waypoint
  // wayError struct has distance to waypoint and unwrapped heading error
  //
  struct wayError wayE = getWayE(P);
  //
  ////////////////////
  // BEGIN CONTROL ALGORITHM
  // MAY NEED TO UPDATE FOR BETTER BEHAVIOR!
  //
  // pwmAvg = average pwm to control speed
  // pwmDel = 2*((right pwm) - (left pwm)) to control turn rate
  // set standard commands here then modify based on mode
  if (wayE.errQ > 3.14) {
    wayE.errQ = wayE.errQ - 6.28;
  } else if (wayE.errQ < -3.14) {
    wayE.errQ = wayE.errQ + 6.28;
  }
  
  int pwmDel = (int) KQ*wayE.errQ*4;
  int pwmAvg = 125/2;

  // turn first then move
  if (abs(wayE.errQ) > 0.3) {
    pwmAvg = 0;
  }

  Serial.print("PWM DEl: ");
  Serial.print(pwmDel);
  Serial.print(" Error: ");
  Serial.println(wayE.errQ);


  //
  // motorsGo: toggles to false for emergency stop or second B button press
  //
  if (!motorsGo) {
    pwmAvg = 0; pwmDel = 0;
  }
  //
  // END CONTROL ALGORITHM
  ////////////////////
  pwmDel = constrain(pwmDel,-200,200);
  motors.setSpeeds(pwmAvg-pwmDel, pwmAvg+pwmDel);
  //
  // display P.X, P.Y & P.Q
  //
  lcd.clear();
  if (motorsGo) {
    lcd.print(F("Go"));
  }
  else {
    lcd.print(F("No"));
  }
  char buf[5];
  sprintf(buf, "%+04d", (int)(57.3*P.Q));
  lcd.gotoXY(4,0);
  lcd.print(buf);
  sprintf(buf, "%+04d", (int)P.X);
  lcd.gotoXY(0,1);
  lcd.print(buf);
  sprintf(buf, "%+04d", (int)P.Y);
  lcd.gotoXY(4,1);
  lcd.print(buf);
}
