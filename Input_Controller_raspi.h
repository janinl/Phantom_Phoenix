/*
class CommanderInputController : 
public InputController
{
public:
  CommanderInputController();        // A reall simple constructor...

  virtual void     Init(void);
  virtual void     ControlInput(void);
  virtual void     AllowControllerInterrupts(boolean fAllow);

};
*/

void CommanderTurnRobotOff(void);


InputController g_InputController;

static short   g_BodyYOffset;
static short   g_BodyYShift;
bool g_fDynamicLegXZLength = false;  // Has the user dynamically adjusted the Leg XZ init pos (width)



//==============================================================================
// Constructor. See if there is a simple way to have one or more Input
//     controllers. Maybe register at construction time
//==============================================================================
/*
InputController::InputController()
{
}
*/

void InputController::Init()
{
}

void InputController::AllowControllerInterrupts(boolean fAllow)
{
}

//==============================================================================
// This is The main code to input function to read inputs from the Commander and then
//process any commands.
//==============================================================================
void InputController::ControlInput(void)
{
CommanderTurnRobotOff();
/*
  // See if we have a new command available...
  if(command.ReadMsgs() > 0){
    // If we receive a valid message than turn robot on...
    boolean fAdjustLegPositions = false;
    short sLegInitXZAdjust = 0;
    short sLegInitAngleAdjust = 0;

    if (!g_InControlState.fRobotOn ) {
      g_InControlState.fRobotOn = true;
      fAdjustLegPositions = true;
    }

    // [SWITCH MODES]

    // Cycle through modes...
    if ((command.buttons & BUT_LT) && !(buttonsPrev & BUT_LT)) {
      if (++ControlMode >= MODECNT) {
        ControlMode = WALKMODE;    // cycled back around...
        MSound( 2, 50, 2000, 50, 3000); 
      } 
      else {
        MSound( 1, 50, 2000);  
      }
#ifdef OPT_SINGLELEG      
      if (ControlMode != SINGLELEGMODE)
        g_InControlState.SelectedLeg=255;
      else {
        g_InControlState.SelectedLeg = 0;   // Select leg 0 when we go into this mode. 
        g_InControlState.PrevSelectedLeg = 255;
#ifdef DEBUG_SINGLELEG
        Serial.println("Single Leg Mode");  
#endif
      }
#endif
    }

    //[Common functions]
    //Switch Balance mode on/off 
    if ((command.buttons & BUT_L4) && !(buttonsPrev & BUT_L4)) {
      g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
      if (g_InControlState.BalanceMode) {
        MSound( 1, 250, 1500); 
      } 
      else {
        MSound( 2, 100, 2000, 50, 4000);
      }
    }

    //Stand up, sit down  
    if ((command.buttons & BUT_L5) && !(buttonsPrev & BUT_L5)) {
      if (g_BodyYOffset>0) 
        g_BodyYOffset = 0;
      else
        g_BodyYOffset = 35;
      fAdjustLegPositions = true;
      g_fDynamicLegXZLength = false;
    }

    // We will use L6 with the Right joystick to control both body offset as well as Speed...
    // We move each pass through this by a percentage of how far we are from center in each direction
    // We get feedback with height by seeing the robot move up and down.  For Speed, I put in sounds
    // which give an idea, but only for those whoes robot has a speaker
    int lx = command.leftH;
    int ly = command.leftV;

    if (command.buttons & BUT_L6 ) {
      // raise or lower the robot on the joystick up /down
      // Maybe should have Min/Max
      int delta = command.rightV/25;   
      if (delta) {
        g_BodyYOffset = max(min(g_BodyYOffset + delta, MAX_BODY_Y), 0);
        fAdjustLegPositions = true;
      }

      // Also use right Horizontal to manually adjust the initial leg positions.
      sLegInitXZAdjust = lx/10;        // play with this.
      sLegInitAngleAdjust = ly/8;
      lx = 0;
      ly = 0;

      // Likewise for Speed control
      delta = command.rightH / 16;   // 
      if ((delta < 0) && g_InControlState.SpeedControl) {
        if ((word)(-delta) <  g_InControlState.SpeedControl)
          g_InControlState.SpeedControl += delta;
        else 
          g_InControlState.SpeedControl = 0;
        MSound( 1, 50, 1000+g_InControlState.SpeedControl);  
      }
      if ((delta > 0) && (g_InControlState.SpeedControl < 2000)) {
        g_InControlState.SpeedControl += delta;
        if (g_InControlState.SpeedControl > 2000)
          g_InControlState.SpeedControl = 2000;
        MSound( 1, 50, 1000+g_InControlState.SpeedControl); 
      }

      command.rightH = 0; // don't walk when adjusting the speed here...
    }

#ifdef DBGSerial
    if ((command.buttons & BUT_R3) && !(buttonsPrev & BUT_R3)) {
      MSound(1, 50, 2000);
      g_fDebugOutput = !g_fDebugOutput;
    }
#endif    
    //[Walk functions]
    if (ControlMode == WALKMODE) {
      //Switch gates
      if (((command.buttons & BUT_R1) && !(buttonsPrev & BUT_R1))
        && abs(g_InControlState.TravelLength.x)<cTravelDeadZone //No movement
      && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
        && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone  ) {
        g_InControlState.GaitType = g_InControlState.GaitType+1;                    // Go to the next gait...
        if (g_InControlState.GaitType<NUM_GAITS) {                 // Make sure we did not exceed number of gaits...
          MSound( 1, 50, 2000);  
        } 
        else {
          MSound (2, 50, 2000, 50, 2250); 
          g_InControlState.GaitType = 0;
        }
        GaitSelect();
      }

      //Double leg lift height
      if ((command.buttons & BUT_RT) && !(buttonsPrev & BUT_RT)) {
        MSound( 1, 50, 2000);  
        HeightSpeedMode = (HeightSpeedMode + 1) & 0x3; // wrap around mode
        DoubleTravelOn = HeightSpeedMode & 0x1;
        if ( HeightSpeedMode & 0x2)
          g_InControlState.LegLiftHeight = 80;
        else
          g_InControlState.LegLiftHeight = 50;
      }

      // Switch between Walk method 1 && Walk method 2
      if ((command.buttons & BUT_R2) && !(buttonsPrev & BUT_R2)) {
#ifdef cTurretRotPin
        if ((++bJoystickWalkMode) > 2)
#else
          if ((++bJoystickWalkMode) > 1)
#endif 
            bJoystickWalkMode = 0;
        MSound (1, 50, 2000 + bJoystickWalkMode*250);
      }

      //Walking
      switch (bJoystickWalkMode) {
      case 0:    
        g_InControlState.TravelLength.x = -lx;
        g_InControlState.TravelLength.z = -ly;
        g_InControlState.TravelLength.y = -(command.rightH)/4; //Right Stick Left/Right 
        break;
      case 1:
        g_InControlState.TravelLength.z = (command.rightV); //Right Stick Up/Down  
        g_InControlState.TravelLength.y = -(command.rightH)/4; //Right Stick Left/Right 
        break;
#ifdef cTurretRotPin
      case 2:
        g_InControlState.TravelLength.x = -lx;
        g_InControlState.TravelLength.z = -ly;

        // Will use Right now stick to control turret.
        g_InControlState.TurretRotAngle1 =  max(min(g_InControlState.TurretRotAngle1+command.rightH/5, cTurretRotMax1), cTurretRotMin1);      // Rotation of turret in 10ths of degree
        g_InControlState.TurretTiltAngle1 =  max(min(g_InControlState.TurretTiltAngle1+command.rightV/5, cTurretTiltMax1), cTurretTiltMin1);  // tilt of turret in 10ths of degree
#endif
      }

      if (!DoubleTravelOn) {  //(Double travel length)
        g_InControlState.TravelLength.x = g_InControlState.TravelLength.x/2;
        g_InControlState.TravelLength.z = g_InControlState.TravelLength.z/2;
      }

    }

    //[Translate functions]
    g_BodyYShift = 0;
    if (ControlMode == TRANSLATEMODE) {
      g_InControlState.BodyPos.x =  SmoothControl(((lx)*2/3), g_InControlState.BodyPos.x, SmDiv);
      g_InControlState.BodyPos.z =  SmoothControl(((ly)*2/3), g_InControlState.BodyPos.z, SmDiv);
      g_InControlState.BodyRot1.y = SmoothControl(((command.rightH)*2), g_InControlState.BodyRot1.y, SmDiv);

      //      g_InControlState.BodyPos.x = (lx)/2;
      //      g_InControlState.BodyPos.z = -(ly)/3;
      //      g_InControlState.BodyRot1.y = (command.rightH)*2;
      g_BodyYShift = (-(command.rightV)/2);
    }

    //[Rotate functions]
    if (ControlMode == ROTATEMODE) {
      g_InControlState.BodyRot1.x = (ly);
      g_InControlState.BodyRot1.y = (command.rightH)*2;
      g_InControlState.BodyRot1.z = (lx);
      g_BodyYShift = (-(command.rightV)/2);
    }
#ifdef OPT_GPPLAYER
    //[GPPlayer functions]
    if (ControlMode == GPPLAYERMODE) {
      // Lets try some speed control... Map all values if we have mapped some before
      // or start mapping if we exceed some minimum delta from center
      // Have to keep reminding myself that commander library already subtracted 128...
      if (g_ServoDriver.FIsGPSeqActive() ) {
        if ((g_sGPSMController != 32767)  
          || (command.rightV > 16) || (command.rightV < -16))
        {
          // We are in speed modify mode...
          if (command.rightV >= 0)
            g_sGPSMController = map(command.rightV, 0, 127, 0, 200);
          else  
            g_sGPSMController = map(command.rightV, -127, 0, -200, 0);
          g_ServoDriver.GPSetSpeedMultiplyer(g_sGPSMController);
        }
      }

      //Switch between sequences
      if ((command.buttons & BUT_R1) && !(buttonsPrev & BUT_R1)) {
        if (!g_ServoDriver.FIsGPSeqActive() ) {
          if (GPSeq < 5) {  //Max sequence
            MSound (1, 50, 1500);  
            GPSeq = GPSeq+1;
          } 
          else {
            MSound (2, 50, 2000, 50, 2250);
            GPSeq=0;
          }
        }
      }
      //Start Sequence
      if ((command.buttons & BUT_R2) && !(buttonsPrev & BUT_R2)) {
        if (!g_ServoDriver.FIsGPSeqActive() ) {
          g_ServoDriver.GPStartSeq(GPSeq);
          g_sGPSMController = 32767;  // Say that we are not in Speed modify mode yet... valid ranges are 50-200 (both postive and negative... 
        }
        else {
          g_ServoDriver.GPStartSeq(0xff);    // tell the GP system to abort if possible...
          MSound (2, 50, 2000, 50, 2000);
        }
      }

    }
#endif // OPT_GPPLAYER

    //[Single leg functions]
#ifdef OPT_SINGLELEG      
    if (ControlMode == SINGLELEGMODE) {
      //Switch leg for single leg control
      if ((command.buttons & BUT_R1) && !(buttonsPrev & BUT_R1)) {
        MSound (1, 50, 2000);  
        if (g_InControlState.SelectedLeg<(CNT_LEGS-1))
          g_InControlState.SelectedLeg = g_InControlState.SelectedLeg+1;
        else
          g_InControlState.SelectedLeg=0;
      }

#if 0
      g_InControlState.SLLeg.x= (signed char)((int)((int)lx+128)/2); //Left Stick Right/Left
      g_InControlState.SLLeg.y= (signed char)((int)((int)command.rightV+128)/10); //Right Stick Up/Down
      g_InControlState.SLLeg.z = (signed char)((int)((int)ly+128)/2); //Left Stick Up/Down
#else
      // BUGBUG:: Need to figure out a decent range for these values... 
      g_InControlState.SLLeg.x = lx; //Left Stick Right/Left
      g_InControlState.SLLeg.y = command.rightV / 3 - 20; //Right Stick Up/Down
      g_InControlState.SLLeg.z = ly; //Left Stick Up/Down
#endif
#ifdef DEBUG_SINGLELEG
      Serial.print(g_InControlState.SLLeg.x, DEC);
      Serial.print(",");
      Serial.print(g_InControlState.SLLeg.y, DEC);
      Serial.print(",");
      Serial.println(g_InControlState.SLLeg.z, DEC);
#endif
      // Hold single leg in place
      if ((command.buttons & BUT_RT) && !(buttonsPrev & BUT_RT)) {
        MSound (1, 50, 2000);  
        g_InControlState.fSLHold = !g_InControlState.fSLHold;
      }
    }
#endif

    //Calculate walking time delay
    g_InControlState.InputTimeDelay = 128 - max(max(abs(lx), abs(ly)), abs(command.rightH));

    //Calculate g_InControlState.BodyPos.y
    g_InControlState.BodyPos.y = max(g_BodyYOffset + g_BodyYShift,  0);

    if (sLegInitXZAdjust || sLegInitAngleAdjust) {
      // User asked for manual leg adjustment - only do when we have finished any previous adjustment

        if (!g_InControlState.ForceGaitStepCnt) {
        if (sLegInitXZAdjust)
          g_fDynamicLegXZLength = true;

        sLegInitXZAdjust += GetLegsXZLength();  // Add on current length to our adjustment...
        // Handle maybe change angles...
        if (sLegInitAngleAdjust) 
          RotateLegInitAngles(sLegInitAngleAdjust);

        // Give system time to process previous calls
        AdjustLegPositions(sLegInitXZAdjust);
      }
    }    

    if (fAdjustLegPositions && !g_fDynamicLegXZLength)
      AdjustLegPositionsToBodyHeight();    // Put main workings into main program file

    // Save away the buttons state as to not process the same press twice.
    buttonsPrev = command.buttons;
    extPrev = command.ext;
    g_ulLastMsgTime = millis();
  } 
  else {
    // We did not receive a valid packet.  check for a timeout to see if we should turn robot off...
    if (g_InControlState.fRobotOn) {
      if ((millis() - g_ulLastMsgTime) > ARBOTIX_TO)
        CommanderTurnRobotOff();
    }
  }
*/
}

/*
boolean CommanderInputController::ProcessTerminalCommand(byte *psz, byte bLen)
{
}
*/





//==============================================================================
// CommanderTurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void CommanderTurnRobotOff(void)
{
  //Turn off
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.TravelLength.x = 0;
  g_InControlState.TravelLength.z = 0;
  g_InControlState.TravelLength.y = 0;
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
#ifdef OPT_SINGLELEG      
  g_InControlState.SelectedLeg = 255;
#endif
  g_InControlState.fRobotOn = 0;

#ifdef cTurretRotPin
  g_InControlState.TurretRotAngle1 = cTurretRotInit;      // Rotation of turrent in 10ths of degree
  g_InControlState.TurretTiltAngle1 = cTurretTiltInit;    // the tile for the turret
#endif

  g_fDynamicLegXZLength = false; // also make sure the robot is back in normal leg init mode...
}

