#include <scpiparser.h>
#include <Arduino.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(7,8,9,10,11,12);

struct scpi_parser_context ctx;

/* forward declarations of all functions called by SCPI commands */
scpi_error_t identify(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t get_voltage_ch1(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t get_voltage_ch2(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t get_current_ch1(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t get_current_ch2(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t set_voltage_ch1(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t set_voltage_ch2(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t set_current_ch1(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t set_current_ch2(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t get_status_ch1(struct scpi_parser_context *context, struct scpi_token *command);
scpi_error_t get_status_ch2(struct scpi_parser_context *context, struct scpi_token *command);

// flags for catching long button presses
int prevBtn = -1; // -1 means no previous button
int currBtn = -1; // -1 means no current button
int currPress = 0; // 0 means there was no press during this loop

// flag to indicate relay initialization
int RelayInit = 0; // 0: initializing, 1: completed

// variables to detect and ignore long button presses
unsigned long mseci   = millis();
unsigned long msecf   = millis();
unsigned long msecDel = 0;
unsigned long lcd_delayi = millis();
unsigned long lcd_delayf = millis();
unsigned long lcd_delay = 0;
unsigned long last_pressi = millis();
unsigned long last_pressf = millis();
unsigned long last_press  = 0;
unsigned long relay_delayi = millis();
unsigned long relay_delayf = millis();
unsigned long relay_delay  = 0;
unsigned long ch1_pwm_delayi = millis();
unsigned long ch1_pwm_delayf = millis();
unsigned long ch1_pwm_delay  = 0;
unsigned long ch2_pwm_delayi = millis();
unsigned long ch2_pwm_delayf = millis();
unsigned long ch2_pwm_delay  = 0;

String ErrorReport = ""; // string to display errors on LCD (top row)
String ErrorReport2 = ""; // string to display errors on LCD (bottom row)
String CommandString = ""; // used to hold the SCPI command to execute
String setCommand    = "VOLTAGE"; // used to build SCPI command (whether to set VOLTAGE or CURRENT)
String setValue      = ""; // used to build SCPI command (value to set)
int setChannel    = 1; // flag indicating currently selected channel (1 or 2)
int keypadDone    = 1; // flag indicating if user is entering data (1:nothing 0:entering)

// pins for reading voltage and current
int CH1VoltPin    = A4;
int CH1CurrPin    = A5;
int CH2VoltPin    = A6;
int CH2CurrPin    = A7;

// pins for setting voltage and current
int CH1SetPin     = 13; // PWM pin
int CH2SetPin     = 6; // PWM pin
int CH1RelayLoad  = 22;
int CH1RelayVolt  = 24;
int CH2RelayLoad  = 23;
int CH2RelayVolt  = 25;

// voltage and current measurement values, for returning values to user
float CH1VoltRead  = 0.0;
float CH1CurrRead  = 0.0;
float CH2VoltRead  = 0.0;
float CH2CurrRead  = 0.0;

// target voltage and current values
float CH1VoltSet  = 0.0;
float CH1CurrSet  = 0;
float CH2VoltSet  = 0.0;
float CH2CurrSet  = 0;

// enable/disable flags for each channel (0: disabled, 1: enabled)
bool CH1Enabled = false;
bool CH2Enabled = false;

// PWM cycle width values, used to control output voltage
int   CH1VoltPWM  = 255;
int   CH2VoltPWM  = 255;

// enable flags for each channel, read from and written to by SCPI commands
bool CH1Enabled = false;
bool CH2Enabled = false;

// storage space for partial incoming SCPI commands, and flag indicating a complete command
String SerialInputUSB = "";
bool SerialInputUSBComplete = false;
String SerialInputBT = "";
bool SerialInputBTComplete = false;

int row[4] = {A0, A1, A2, A3};
int col[4] = {2, 3, 4, 5};

//these keys will represent what is being pressed on the keypad
char key[4][4] = { {'1', '2', '3', 'C'},
                   {'4', '5', '6', 'N'},
                   {'7', '8', '9', 'I'},
                   {'0', '.', 'E', 'V'} };

float vRef = 0.0;

void setup()
{
  /* declared pointers for the heads of the source and measure SCPI command trees */
  struct scpi_command *source;
  struct scpi_command *measure;
  struct scpi_command *state;

  /* initialize the used SCPI library */
  scpi_init(&ctx);

  /* register the "identify" SCPI command
   * format is (parent command, command level, long name, name length, short name, name length, function)
   *     parent command: the command above this one in the command tree, called as PARENT:CHILD
   *     command level: whether this command is added at the same level as the parent, or as a child
   *     long name: the long form of the SCPI command name
   *     name length: the number of characters in the long name (to avoid line ending oddities)
   *     short name: the short form of the SCPI command name (can be identical to long name)
   *     name length: the number of characters in the short name
   *     function: the name of the function to call when receiving this SCPI command */
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "*IDN?", 5, "*IDN?", 5, identify);

  /* register the head of the "source" and "measure" SCPI command trees */
  source = scpi_register_command(ctx.command_tree, SCPI_CL_CHILD, "SOURCE", 6, "SOUR", 4, NULL);
  measure = scpi_register_command(ctx.command_tree, SCPI_CL_CHILD, "MEASURE", 7, "MEAS", 4, NULL);
  state = scpi_register_command(ctx.command_tree, SCPI_CL_CHILD, "STATUS", 6, "STAT", 4, NULL);

  /* register all members of the "source" command tree (voltage/current for each channel) */
  scpi_register_command(source, SCPI_CL_CHILD, "VOLTAGE1", 8, "VOLT1", 5, set_voltage_ch1);
  scpi_register_command(source, SCPI_CL_CHILD, "VOLTAGE2", 8, "VOLT2", 5, set_voltage_ch2);
  scpi_register_command(source, SCPI_CL_CHILD, "CURRENT1", 8, "CURR1", 5, set_current_ch1);
  scpi_register_command(source, SCPI_CL_CHILD, "CURRENT2", 8, "CURR2", 5, set_current_ch2);

  /* register all members of the "measure" command tree (voltage/current for each channel) */
  scpi_register_command(measure, SCPI_CL_CHILD, "VOLTAGE1?", 9, "VOLT1?", 6, get_voltage_ch1);
  scpi_register_command(measure, SCPI_CL_CHILD, "VOLTAGE2?", 9, "VOLT2?", 6, get_voltage_ch2);
  scpi_register_command(measure, SCPI_CL_CHILD, "CURRENT1?", 9, "CURR1?", 6, get_current_ch1);
  scpi_register_command(measure, SCPI_CL_CHILD, "CURRENT2?", 9, "CURR2?", 6, get_current_ch2);

  /* register all members of the "state" command tree */
  scpi_register_command(state, SCPI_CL_CHILD, "CHANNEL1?", 9, "CH1?", 4, get_status_ch1);
  scpi_register_command(state, SCPI_CL_CHILD, "CHANNEL2?", 9, "CH2?", 4, get_status_ch2);

  /* reserve 256 bytes for the SCPI command input buffer (well above requirements) */
  SerialInputUSB.reserve(256);

  Serial.begin(9600);

  //Setup LCD Screen
  lcd.begin(16, 2);

  // Setup channel 1 and 2 Relays
  pinMode(CH1RelayLoad, OUTPUT);
  pinMode(CH1RelayVolt, OUTPUT);
  pinMode(CH2RelayLoad, OUTPUT);
  pinMode(CH2RelayVolt, OUTPUT);
  pinMode(CH1SetPin, OUTPUT);
  pinMode(CH2SetPin, OUTPUT);

  relay_delayi = millis();  // begin counting
  digitalWrite(CH1RelayLoad, LOW); //Start with relay opened
  digitalWrite(CH1RelayVolt, LOW);
  digitalWrite(CH2RelayLoad, LOW);
  digitalWrite(CH2RelayVolt, LOW);

  //Set Voltage and Current Read Inputs
  pinMode(CH1VoltPin, INPUT); //Voltage (V) Channel (1) Read
  pinMode(CH1CurrPin, INPUT); //Current (I) Channel (1) Read
  pinMode(CH2VoltPin, INPUT); //Voltage (V) Channel (2) Read
  pinMode(CH2CurrPin, INPUT); //Current (I) Channel (2) Read

  //Set the row to INPUT with the internal pull-up resistors
  for(int i = 0; i < 4; i++)
    digitalWrite(row[i], INPUT_PULLUP); //Set the analog pins for input with pullup resistor

  //set the columns to output
  for(int i = 0; i < 4; i++)
    pinMode(col[i], OUTPUT);
}

void loop()
{
  char line_buffer[256];
  unsigned char read_length;

  relay_delayf = millis();
  relay_delay  = relay_delayf - relay_delayi;

  // Wait about 100ms to allow inductors and capacitors
  // to charge. This will lower the "drive" of the buck converter.
  if(relay_delay >= 100 && RelayInit == 0){
    RelayInit = 1;

    if(CH1Enabled){
      digitalWrite(CH1RelayLoad, LOW);
      digitalWrite(CH1RelayVolt, LOW);
    }

    if(CH2Enabled){
      digitalWrite(CH2RelayLoad, LOW);
      digitalWrite(CH2RelayVolt, LOW);
    }
  }

  /* if a complete SCPI command is in the buffer, execute the command and reset the buffer */
  if (SerialInputUSBComplete == true) {
	  scpi_execute_command(&ctx, SerialInputUSB.c_str(), SerialInputUSB.length() - 1);
	  SerialInputUSB = "";
	  SerialInputUSBComplete = false;
  }

  if (SerialInputBTComplete == true) {
	  scpi_execute_command(&ctx, SerialInputBT.c_str(), SerialInputBT.length() - 1);
	  SerialInputBT = "";
	  SerialInputBTComplete = false;
  }

  //Used for LCD Display and Keypad Readback
  ReadVoltage();
  ReadCurrent();
  AdjustVoltCurr();
  RelayUpdate();
  KeyPad();
  UpdateLCD();
}

/* runs in between every iteration of loop() */
void serialEvent1()
{
	/* if there is anything in the serial buffer, process it */
	while (Serial1.available()) {
		/* add the new character to the SCPI command buffer */
		char NewCharacter  = (char) Serial1.read();
		SerialInputUSB += NewCharacter;

		/* if a newline character is read, assume complete command and break */
		if (NewCharacter == '\n') {
			SerialInputUSBComplete = true;
			break;
		}
	}
}

void serialEvent2()
{
	/* if there is anything in the serial buffer, process it */
	while (Serial2.available()) {
		/* add the new character to the SCPI command buffer */
		char NewCharacter  = (char) Serial2.read();
		SerialInputBT += NewCharacter;

		/* if a newline character is read, assume complete command and break */
		if (NewCharacter == '\n') {
			SerialInputBTComplete = true;
			break;
		}
	}
}

/* returns an identification string for this device */
scpi_error_t identify(struct scpi_parser_context *context, struct scpi_token *command)
{
	Serial.println("SCPI Configurable DC Power Supply V1.0");

	scpi_free_tokens(command);
	return SCPI_SUCCESS;
}

/* returns the measured voltage of channel 1 */
scpi_error_t get_voltage_ch1(struct scpi_parser_context *context, struct scpi_token *command)
{
	Serial.println(CH1VoltRead, 4);

	scpi_free_tokens(command);
	return SCPI_SUCCESS;
}

/* returns the measured voltage of channel 2 */
scpi_error_t get_voltage_ch2(struct scpi_parser_context *context, struct scpi_token *command)
{
	Serial.println(CH2VoltRead, 4);

	scpi_free_tokens(command);
	return SCPI_SUCCESS;
}

/* returns the measured current of channel 1 */
scpi_error_t get_current_ch1(struct scpi_parser_context *context, struct scpi_token *command)
{
	Serial.println(CH1CurrRead, 4);

	scpi_free_tokens(command);
	return SCPI_SUCCESS;
}

/* returns the measured current of channel 2 */
scpi_error_t get_current_ch2(struct scpi_parser_context *context, struct scpi_token *command)
{
	Serial.println(CH2CurrRead, 4);

	scpi_free_tokens(command);
	return SCPI_SUCCESS;
}

/* returns the state of channel 1 */
scpi_error_t get_status_ch1(struct scpi_parser_context *context, struct scpi_token *command)
{
    if (CH1Enabled)
    {
        Serial.println("true");
    }
    else
    {
        Serial.println("false");
    }

    scpi_free_tokens(command);
    return SCPI_SUCCESS;
}

/* returns the state of channel 2 */
scpi_error_t get_status_ch2(struct scpi_parser_context *context, struct scpi_token *command)
{
    if (CH2Enabled)
    {
        Serial.println("true");
    }
    else
    {
        Serial.println("false");
    }

    scpi_free_tokens(command);
    return SCPI_SUCCESS;
}

/* sets the target voltage for channel 1 */
scpi_error_t set_voltage_ch1(struct scpi_parser_context *context, struct scpi_token *command)
{
	/* declaration of parsed numeric input */
	struct scpi_numeric output_numeric;

	/* loop through token list until a numeric result is found */
	while (command != NULL && command->type == 0)
	{
		command = command->next;
	}

	/* parse the numeric result into an scpi_numeric struct */
	output_numeric = scpi_parse_numeric(command->value, command->length, 0, 0, 5);
	/* if the numeric result has no unit, or the unit is specified as V, use the value */
	if (output_numeric.length == 0 || (output_numeric.length == 1 && output_numeric.unit[0] == 'V'))
	{
		/* restrict the voltage between 2V and 14V */
		CH1VoltSet = (unsigned char) constrain(output_numeric.value, 2.f, 14.f);
	}
	else // if the numeric result has the wrong unit
	{
		/* generate an internal error for an invalid unit */
		scpi_error error;
		error.id = -3;
		error.description = "Invalid Unit";
		error.length = 12;

		/* place the error on the error stack */
		scpi_queue_error(&ctx, error);
	}

	/* consume the SCPI command and exit the function */
	scpi_free_tokens(command);
	return SCPI_SUCCESS;
}

/* sets the target voltage for channel 2 */
scpi_error_t set_voltage_ch2(struct scpi_parser_context *context, struct scpi_token *command)
{
	/* declaration of parsed numeric input */
	struct scpi_numeric output_numeric;

	/* loop through token list until a numeric result is found */
	while (command != NULL && command->type == 0)
	{
		command = command->next;
	}

	/* parse the numeric result into an scpi_numeric struct */
	output_numeric = scpi_parse_numeric(command->value, command->length, 0, 0, 5);
	/* if the numeric result has no unit, or the unit is specified as V, use the value */
	if (output_numeric.length == 0 || (output_numeric.length == 1 && output_numeric.unit[0] == 'V'))
	{
		/* restrict the voltage between 2V and 14V */
		CH2VoltSet = (unsigned char) constrain(output_numeric.value, 2.f, 14.f);
	}
	else // if the numeric result has the wrong unit
	{
		/* generate an internal error for an invalid unit */
		scpi_error error;
		error.id = -3;
		error.description = "Invalid Unit";
		error.length = 12;

		/* place the error on the error stack */
		scpi_queue_error(&ctx, error);
	}

	/* consume the SCPI command and exit the function */
	scpi_free_tokens(command);
	return SCPI_SUCCESS;
}

/* sets the target current for channel 1 */
scpi_error_t set_current_ch1(struct scpi_parser_context *context, struct scpi_token *command)
{
	/* declaration of parsed numeric input */
	struct scpi_numeric output_numeric;

	/* loop through token list until a numeric result is found */
	while (command != NULL && command->type == 0)
	{
		command = command->next;
	}

	/* parse the numeric result into an scpi_numeric struct */
	output_numeric = scpi_parse_numeric(command->value, command->length, 0, 0, 5);
	/* if the numeric result has no unit, or the unit is specified as V, use the value */
	if (output_numeric.length == 0 || (output_numeric.length == 1 && output_numeric.unit[0] == 'A'))
	{
		/* restrict the current between 0A and 1.5A */
		CH1VoltSet = (unsigned char) constrain(output_numeric.value, 0.f, 1.5f);
	}
	else // if the numeric result has the wrong unit
	{
		/* generate an internal error for an invalid unit */
		scpi_error error;
		error.id = -3;
		error.description = "Invalid Unit";
		error.length = 12;

		/* place the error on the error stack */
		scpi_queue_error(&ctx, error);
	}

	/* consume the SCPI command and exit the function */
	scpi_free_tokens(command);
	return SCPI_SUCCESS;
}

/* sets the target current for channel 1 */
scpi_error_t set_current_ch2(struct scpi_parser_context *context, struct scpi_token *command)
{
	/* declaration of parsed numeric input */
	struct scpi_numeric output_numeric;

	/* loop through token list until a numeric result is found */
	while (command != NULL && command->type == 0)
	{
		command = command->next;
	}

	/* parse the numeric result into an scpi_numeric struct */
	output_numeric = scpi_parse_numeric(command->value, command->length, 0, 0, 5);
	/* if the numeric result has no unit, or the unit is specified as V, use the value */
	if (output_numeric.length == 0 || (output_numeric.length == 1 && output_numeric.unit[0] == 'A'))
	{
		/* restrict the current between 0A and 1.5A */
		CH2VoltSet = (unsigned char) constrain(output_numeric.value, 0.f, 1.5f);
	}
	else // if the numeric result has the wrong unit
	{
		/* generate an internal error for an invalid unit */
		scpi_error error;
		error.id = -3;
		error.description = "Invalid Unit";
		error.length = 12;

		/* place the error on the error stack */
		scpi_queue_error(&ctx, error);
	}

	/* consume the SCPI command and exit the function */
	scpi_free_tokens(command);
	return SCPI_SUCCESS;
}

void KeyPad(){
  //Loop through the columns
  currPress = 0;
  mseci = millis();
  last_pressf = millis(); //When was the last keypress?
  last_press  = (last_pressf - last_pressi);
  //if it has been 15 seconds, clear the variables
  if(last_press >= 15000){
    setValue   = ""; //Reset variables
    keypadDone = 1;  // 1: Done 0: Still Entering
  }

  for(int i = 0; i < 4; i++){
    //set the column low
    digitalWrite(col[i], LOW);

    //loop through the rows
    for(int j = 0; j < 4; j++){

      /*
       * Read each row button. If the row is low then the current
       * column corresponds to the button that is read low. Other
       * buttons on the row will be high due to the other columns
       * being set high.
       */
      if(digitalRead(row[j]) == 0){
        currPress = 1;
        currBtn = row[j];
        msecDel = mseci - msecf; //mseci is always updated for each loop, msecf is in the past


        //Prevent repeated keystrokes by holding key down
        //also prevent debouncing with time counting
        if(currBtn != prevBtn && msecDel >= 300){
          keypadDone = 0;

          last_pressi = millis(); //Keep track of the last keypress
          msecf = millis(); //update last key press time

          //print the key to the serial monitor for debugging purposes.
          printKeyPad(key[i][j]);

        }

        //Update for next loop check
        prevBtn = currBtn;
      }
    }

    //set the column back to high
    digitalWrite(col[i], HIGH);
  }

  // There is no long-press, clear variables
  if(currPress == 0){
    currBtn = -1;
    prevBtn = -1;
  }
}

/***********************************************
 *  Function: printKeyPad()
 *  Description: Convert keypad presses to SCPI
 *  commands.
 ************************************************/
void printKeyPad(char k){

  if(k == 'C'){
    Serial.println("Key: Channel Select");
    setCommand = "CHANNEL";
  }else if(k == 'N'){
    Serial.println("Key: Channel Enable/Disable");

    if(setChannel == 1){
        if(CH1Enabled)
          CH1Enabled = false;
        else
          CH1Enabled = true;
    }else{
        if(CH2Enabled)
          CH2Enabled = false;
        else
          CH2Enabled = true;
    }

    //Clear any error reports from over current draw
    ErrorReport = "";
    ErrorReport2 = "";
    setValue = "";
    keypadDone = 1;

  }else if(k == 'V'){
    Serial.println("Key: Set Voltage");

    if(setChannel == 1)
      setCommand = "VOLTAGE";
    else
      setCommand = "VOLTAGE1";

  }else if(k == 'I'){
    Serial.println("key: Set Current");

    if(setChannel == 1)
      setCommand = "CURRENT";
    else
      setCommand = "CURRENT1";
  }else if(k == 'E'){
    Serial.println("Key: Enter");
    sendKeypadValue();

    setValue = "";  //Clear the command string
    keypadDone = 1;
    SetVoltCurr();
  }else{
    // Only use 4 keys, such as 12.50
    if(setValue.length() < 4)
        setValue += k;

    //Update LCD on keypress
    Serial.print("Value: ");
    Serial.println(setValue);
  }
}

/********************************************************
 *  Function: sendKeypadValue()
 *  Description: After the user hits 'Enter' on the
 *  keypad, this function tries to perform the necessary
 *  action.
 *********************************************************/
void sendKeypadValue(){
    if(setCommand != "CHANNEL"){
      //SCPI command to send
      CommandString = ":SOURCE:"+setCommand+" "+setValue+";";

      //Make sure to only send commands that are within range
      if(checkRangeCommand() == 1){

        /*********************/
        /* SET COMMAND VALUE */
        /*********************/
        if(setChannel == 1){
          if(setCommand == "VOLTAGE")
            CH1VoltSet = setValue.toFloat();
          else if(setCommand == "CURRENT")
            CH1CurrSet = setValue.toFloat();

        }else if(setChannel == 2){
          if(setCommand == "VOLTAGE1")
            CH1VoltSet = setValue.toFloat();
          else if(setCommand == "CURRENT1")
            CH1CurrSet = setValue.toFloat();
        }

      }
    }else
      setChannel = setValue.toInt();
}

/***********************************************
 *  Function: checkRangeCommand()
 *  Description: Check if the voltage and current
 *  are within specification.
 *  Where: Voltage [0, 14.0], Current [0, 1.5]
 ************************************************/
int checkRangeCommand(){
  if(setValue == "")
    return 0;

  if(setCommand == "VOLTAGE" || setCommand == "VOLTAGE1"){

    //make sure that the voltage is within spec.
    if(setValue.toFloat() <= 14.0)
      return 1;

  }else if(setCommand == "CURRENT" || setCommand == "CURRENT1"){

    //Make sure the current is within spec
    if(setValue.toFloat() <= 1.5)
      return 1;

  }else if(setCommand = "CHANNEL"){
    // Channel 1 or 2
    if(setValue.toFloat() == 1 || setValue.toFloat() == 2)
      return 1;
  }

  //Default
  return 0;
}

/***********************************************
 *  Function: UpdateLCD()
 *  Description: Display updated information to the
 *  LCD. Displays what the user is inputting as
 *  well as what the channels read.
 ************************************************/
void UpdateLCD(){
  lcd_delayi = millis();

  //calculate the time diff. from last LCD update
  //lcd_delayi is a larger number than lcd_delayf
  lcd_delay = lcd_delayi - lcd_delayf;

  if(lcd_delay > 300){

    if(ErrorReport != "" && lcd_delay > 500){
      clearLCD();
      lcd.setCursor(0, 0);
      lcd.print(ErrorReport);
      lcd.setCursor(0, 1);
      lcd.print(ErrorReport2);

      lcd_delayf = millis();
    }else{

        if(keypadDone == 0){
          clearLCD();

          //Show which option
          lcd.setCursor(0, 0);
          lcd.print(setCommand);
          lcd.setCursor(7, 0);
          lcd.print(":");
          //display pressed value
          lcd.setCursor(8, 0);
          lcd.print(setValue);

          lcd_delayf = millis();
        }else if(lcd_delay > 500){
          clearLCD();

          lcd.setCursor(0, 0);
          lcd.print("C1");
          lcd.setCursor(3, 0);
          lcd.print("V:");
          lcd.setCursor(5,0);
          lcd.print(CH1VoltRead, 1);//Display CH1 voltage

          lcd.setCursor(10, 0);
          lcd.print("I:");
          lcd.setCursor(12, 0);
          lcd.print(CH1CurrRead, 2); //Display CH1 Current(I)

          lcd.setCursor(0, 1);
          lcd.print("C2");
          lcd.setCursor(3, 1);
          lcd.print("V:");
          lcd.setCursor(5,1);
          lcd.print(CH2VoltRead, 1);//Display CH2 voltage

          lcd.setCursor(10, 1);
          lcd.print("I:");
          lcd.setCursor(12, 1);
          lcd.print(CH2CurrRead, 2); //Display CH2 Current(I)

          lcd_delayf = millis();
        }
    }
  }
}

/***********************************************
 *  Function: clearLCD()
 *  Description: The screen of the LCD needs to
 *  be cleared so old information is not left on
 *  the screen.
 ************************************************/
void clearLCD(){
  for(int i = 0; i < 16; i++){
    lcd.setCursor(i, 0);
    lcd.print(" ");
  }
  for(int i = 0; i < 16; i++){
    lcd.setCursor(i, 1);
    lcd.print(" ");
  }
}

/***********************************************
 *  Function: ReadVoltage()
 *  Description: The voltage from channel 1 and 2
 *  are read, converted from analog, then scaled
 *  with voltage divider resistors. The final value
 *  is the Voltage (I) and is output to the LCD screen.
 ************************************************/
void ReadVoltage(){
  float v1  = 0.0, v2 = 0.0;

  if(CH1Enabled){
    for(int i = 0; i < 10; i++){
      v1 += analogRead(CH1VoltPin);
    }

    v1 /= 10; //Get the average of 10 reads
    v1 /= 1023; //Convert to float
    v1 *= 5; //Reading with a 5V reference
    v1 *= 3.55; //Voltage divider resistors
    v1 *= 0.99; //Scale the voltage read to fix output

    CH1VoltRead = v1;
  }else
    CH1VoltRead = 0.0;

  if(CH2Enabled){
    for(int i = 0; i < 10; i++){
      v2 += analogRead(CH2VoltPin);
    }
    v2 /= 10; //Get the average of 10 reads
    v2 /= 1023; //Convert to float
    v2 *= 5; //Reading with a 5V reference
    v2 *= 3.55; //Voltage divider resistors
    v2 *= 0.99; //Scale the voltage read to fix output

    //Update Voltage Reading Output
    CH2VoltRead = v2;
  }else
    CH2VoltRead = 0.0;

}

/***********************************************
 *  Function: ReadCurrent()
 *  Description: The voltage from channel 1 and 2
 *  are read, converted from analog, then scaled
 *  with voltage divider resistors. The final value
 *  is the Current (I) and is output to the LCD screen.
 ************************************************/
void ReadCurrent(){
  float c1  = 0.0, c2 = 0.0;

  if(CH1Enabled){
    for(int i = 0; i < 10; i++){
      c1 += analogRead(CH1CurrPin);
    }

    //c1 = analogRead(CH1CurrPin);
    c1 /= 10; //Get the average of 10 reads
    c1 /= 1023; //Convert from Analog
    c1 *= 5; //Reading with a 5V reference
    c1 *= 1.845; ///= 0.542; // *= (1000 + 860)/1000; //Voltage Divider
    c1 /= 5.3;//scale down the 50V/V Gain

    CH1CurrRead = c1;
  }else
    CH1CurrRead = 0.0;

  if(CH2Enabled){
    for(int i = 0; i < 10; i++){
      c2 += analogRead(CH2CurrPin);
    }
    c2 /= 10; //Get the average of 10 reads
    c2 /= 1023; //Convert from Analog
    c2 *= 5; //Reading with a 5V reference
    c2 *= 1.845; ///= 0.542; //*= (1000 + 860)/1000; //Voltage Divider
    c2 /= 5.3;//scale down the 50V/V Gain

    CH2CurrRead = c2;
  }else
    CH2CurrRead = 0.0;

  if( ( c1 >= 1.56 || c1 >  CH1CurrSet) && CH1Enabled){
    CH1Enabled = false;
    digitalWrite(CH1RelayLoad, LOW);
    digitalWrite(CH1RelayVolt, LOW);

    ErrorReport = "Max Current " + String(c1);
    ErrorReport2 = "Channel 1";
  }

  if( ( c2 >= 1.56 || c2 > CH2CurrSet) && CH2Enabled){
    CH2Enabled = false;
    digitalWrite(CH2RelayLoad, LOW);
    digitalWrite(CH2RelayVolt, LOW);

    ErrorReport = "Max Current "+String(c2);
    ErrorReport2 = "Channel 2";
  }
}

void RelayUpdate(){
  if(CH1Enabled){
    digitalWrite(CH1RelayLoad, HIGH);
    digitalWrite(CH1RelayVolt, HIGH);
  }else{
    digitalWrite(CH1RelayLoad, LOW);
    digitalWrite(CH1RelayVolt, LOW);
  }

  if(CH2Enabled){
    digitalWrite(CH2RelayLoad, HIGH);
    digitalWrite(CH2RelayVolt, HIGH);
  }else{
    digitalWrite(CH2RelayLoad, LOW);
    digitalWrite(CH2RelayVolt, LOW);
  }
}

void AdjustVoltCurr(){
  float CH1VoltDiff = CH1VoltSet - CH1VoltRead;
  float CH2VoltDiff = CH2VoltSet - CH1VoltRead;
  float CH1CurrDiff = CH1CurrSet - CH1CurrRead;
  float CH2CurrDiff = CH2CurrSet - CH2CurrRead;
  ch1_pwm_delayi    = millis();
  ch2_pwm_delayi    = millis();

  ch1_pwm_delay = ch1_pwm_delayf - ch1_pwm_delayi;
  ch2_pwm_delay = ch2_pwm_delayf - ch2_pwm_delayi;

  if(CH1Enabled && ch1_pwm_delay >= 100){
    if(CH1VoltDiff < -0.2 && CH1VoltPWM < 255)
      CH1VoltPWM += 1;
    else if(CH1VoltDiff > 0.2 && CH1VoltPWM > 0)
      CH1VoltPWM -= 1;

    ch1_pwm_delayf = millis();
    analogWrite(CH1SetPin, CH1VoltPWM);
  }

  if(CH2Enabled && ch2_pwm_delay >= 100){
    if(CH2VoltDiff < -0.2 && CH2VoltPWM < 255)
      CH2VoltPWM += 1;
    else if(CH2VoltDiff > 0.2 && CH2VoltPWM > 0)
      CH2VoltPWM -= 1;

    ch1_pwm_delayf = millis();
    analogWrite(CH2SetPin, CH2VoltPWM);
  }
}

void SetVoltCurr(){
  if(CH1Enabled){
    CH1VoltPWM = (255/2)*CH1VoltSet/14;
    analogWrite(CH1SetPin, CH1VoltPWM);
  }else
    analogWrite(CH1SetPin, 255);


  if(CH2Enabled){
    CH2VoltPWM = (255/2)*CH2VoltSet/14;
    analogWrite(CH2SetPin, CH2VoltPWM);
  }else
    analogWrite(CH2SetPin, 255);
}
