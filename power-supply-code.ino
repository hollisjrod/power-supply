#include <scpiparser.h>
#include <Arduino.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(7,8,9,10,11,12);

struct scpi_parser_context ctx;

scpi_error_t identify(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t get_voltage(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t get_voltage_2(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t get_voltage_3(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t set_voltage(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t set_voltage_2(struct scpi_parser_context* context, struct scpi_token* command);

// Flags for controlling long-press
int prevBtn = -1; // -1 means no previous button
int currBtn = -1; // -1 means no current button
int currPress = 0; // 0 means there was no press during this loop

//used to detect unwanted long-hold on buttons
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

String ErrorReport = "";    // Display any errors to LCD
String CommandString = "";  // Used to hold the string to send to SCPI
String setCommand    = "VOLTAGE";  // Set Voltage, Current
String setValue      = ""; //Set the value of voltage or current
int setChannel    = 1; //Select channel 1 or 2
int keypadDone    = 1; //Tell the system if user is entering data 1:nothing 0:entering

// The Pins used for the channels
int CH1VoltPin    = A4;
int CH1CurrPin    = A5;
int CH2VoltPin    = A6;
int CH2CurrPin    = A7;
int CH1Relay      = 22;
int CH2Relay      = 23;

//These Values are to be read back to the user.
float CH1VoltRead  = 0.0;
float CH1CurrRead  = 0.0;
float CH2VoltRead  = 0.0;
float CH2CurrRead  = 0.0;

//Limit Voltage and Current
float CH1VoltSet  = 0.0;
float CH1CurrSet  = 0.0;
float CH2VoltSet  = 0.0;
float CH2CurrSet  = 0.0;
int   CH1EN       = 0; // 0 Disabled channel, 1 Enabled CHANNEL
int   CH2EN       = 0;

int row[4] = {A0, A1, A2, A3};
int col[4] = {2, 3, 4, 5};

//these keys will represent what is being pressed on the keypad
char key[4][4] = { {'1', '2', '3', 'C'},
                   {'4', '5', '6', 'V'},
                   {'7', '8', '9', 'I'},
                   {'0', '.', 'N', 'E'} };

int shutdown_cmd = 0; //Command to break the main loop before turning off system.

float volt1 = 0.0;
float vRef = 0.0;

void setup()
{
  pinMode(A0, INPUT);
  analogReference(EXTERNAL);
  volt1 = analogRead(A0);


  struct scpi_command* source;
  struct scpi_command* measure;
  //struct scpi_command* sys;

  /* First, initialise the parser. */
  scpi_init(&ctx);

  /*
   * After initialising the parser, we set up the command tree.  Ours is
   *
   *  *IDN?         -> identify
   *  :SOURCE
   *    :VOLTage    -> set_voltage
   *    :VOLTage1   -> set_voltage_2
   *  :MEASure
   *    :VOLTage?   -> get_voltage
   *    :VOLTage1?  -> get_voltage_2
   *    :VOLTage2?  -> get_voltage_3
   */
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "*IDN?", 5, "*IDN?", 5, identify);
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "OFF", 3, "OFF", 3, shutdown_system);

  source  = scpi_register_command(ctx.command_tree, SCPI_CL_CHILD, "SOURCE", 6, "SOUR", 4, NULL);
  measure = scpi_register_command(ctx.command_tree, SCPI_CL_CHILD, "MEASURE", 7, "MEAS", 4, NULL);
  //sys     = scpi_register_command(ctx.command_tree, SCPI_CL_CHILD, "SYSTEM", 5, "SYST", 3, NULL);

  scpi_register_command(source, SCPI_CL_CHILD, "VOLTAGE", 7, "VOLT", 4, set_voltage);
  scpi_register_command(source, SCPI_CL_CHILD, "VOLTAGE1", 8, "VOLT1", 5, set_voltage_2);

  scpi_register_command(measure, SCPI_CL_CHILD, "VOLTAGE?", 8, "VOLT?", 5, get_voltage);
  scpi_register_command(measure, SCPI_CL_CHILD, "VOLTAGE1?", 9, "VOLT1?", 6, get_voltage_2);
  scpi_register_command(measure, SCPI_CL_CHILD, "VOLTAGE2?", 9, "VOLT2?", 6, get_voltage_3);

  Serial.begin(9600);

  //Setup LCD Screen
  lcd.begin(16, 2);
  lcd.setCursor(0, 1);

  // Setup channel 1 and 2 Relays
  pinMode(CH1Relay, OUTPUT);
  pinMode(CH2Relay, OUTPUT);
  relay_delayi = millis();  // begin counting
  digitalWrite(CH1Relay, HIGH); //Start with relay opened
  digitalWrite(CH2Relay, HIGH);

  pinMode(A4, INPUT); //Voltage (V) Channel (1) Read
  pinMode(A5, INPUT); //Current (I) Channel (1) Read

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

  //check for keypad presses
  KeyPad();

  //vRef  = readAref();

  volt1 = analogRead(A0);
  volt1 = volt1/1023.0;// - 0.206;
  //volt1 = volt1/vRef;

  Serial.print("Voltage: ");
  Serial.print(volt1);
  Serial.print("\tvRef: ");
  Serial.println(vRef);
  delay(500);

  while(Serial.available() > 0){
    /* Read in a line and execute it. */
    read_length = Serial.readBytesUntil('\n', line_buffer, 256);
    if(read_length > 0)
    {
      scpi_execute_command(&ctx, line_buffer, read_length);
    }
  }

  relay_delayf = millis();

  relay_delay  = relay_delayf - relay_delayi;

  // Wait about 100ms to allow inductors and capacitors
  // to charge. This will lower the "drive" of the buck converter.
  if(relay_delay >= 100){
    if(CH1EN)
      digitalWrite(CH1Relay, LOW);
    if(CH2EN)
      digitalWrite(CH2Relay, LOW);
  }

  //Used for LCD Display and Keypad Readback
  ReadVoltage();
  ReadCurrent();
  KeyPad();
  UpdateLCD();
  detectShort();
}


/*
 * Respond to *IDN?
 */
scpi_error_t identify(struct scpi_parser_context* context, struct scpi_token* command)
{
  scpi_free_tokens(command);

  Serial.println("PC Controlled DC Power Supply V1.0");
  return SCPI_SUCCESS;
}

/**
 * Read the voltage on A0.
 */
scpi_error_t get_voltage(struct scpi_parser_context* context, struct scpi_token* command)
{
  float voltage;

  voltage = analogRead(0) * 5.0f/1024;
  Serial.println(voltage,4);

  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * Read the voltage on A1.
 */
scpi_error_t get_voltage_2(struct scpi_parser_context* context, struct scpi_token* command)
{
  float voltage;

  voltage = analogRead(1) * 5.0f/1024;
  Serial.println(voltage,4);

  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * Read the voltage on A2.
 */
scpi_error_t get_voltage_3(struct scpi_parser_context* context, struct scpi_token* command)
{
  float voltage;

  voltage = analogRead(2) * 5.0f/1024;
  Serial.println(voltage,4);

  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * Set the voltage using PWM on pin 3.
 */
scpi_error_t set_voltage(struct scpi_parser_context* context, struct scpi_token* command)
{
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  unsigned char output_value;

  args = command;

  while(args != NULL && args->type == 0)
  {
    args = args->next;
  }

  output_numeric = scpi_parse_numeric(args->value, args->length, 0, 0, 5);
  if(output_numeric.length == 0 ||
    (output_numeric.length == 1 && output_numeric.unit[0] == 'V'))
  {
    output_value = (unsigned char)constrain(output_numeric.value / 5.0f * 256.0f, 0, 255);
  }
  else if(output_numeric.length == 2 &&
    output_numeric.unit[0] == 'N' && output_numeric.unit[1] == 'T')
  {
    output_value = (unsigned char)constrain(output_numeric.value, 0, 255);
  }
  else
  {
    scpi_error error;
    error.id = -200;
    error.description = "Command error;Invalid unit";
    error.length = 26;
    Serial.print(output_numeric.length);

    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  analogWrite(3, output_value);

  scpi_free_tokens(command);

  return SCPI_SUCCESS;
}

/**
 * Set the voltage using PWM on pin 5.
 */
scpi_error_t set_voltage_2(struct scpi_parser_context* context, struct scpi_token* command)
{
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  unsigned char output_value;

  args = command;

  while(args != NULL && args->type == 0)
  {
    args = args->next;
  }

  output_numeric = scpi_parse_numeric(args->value, args->length, 0, 0, 5);
  if(output_numeric.length == 0 ||
    (output_numeric.length == 1 && output_numeric.unit[0] == 'V'))
  {
    output_value = (unsigned char)constrain(output_numeric.value / 5.0f * 256.0f, 0, 255);
  }
  else if(output_numeric.length == 2 &&
    output_numeric.unit[0] == 'N' && output_numeric.unit[1] == 'T')
  {
    output_value = (unsigned char)constrain(output_numeric.value, 0, 255);
  }
  else
  {
    scpi_error error;
    error.id = -200;
    error.description = "Command error;Invalid unit";
    error.length = 26;
    Serial.print(output_numeric.length);

    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  analogWrite(5, output_value);

  scpi_free_tokens(command);

  return SCPI_SUCCESS;
}

scpi_error_t shutdown_system(struct scpi_parser_context* context, struct scpi_token* command)
{
  scpi_free_tokens(command);

  shutdown_cmd = 1;

  return SCPI_SUCCESS;
}

float readAref (void) {
  float volt;
/*
#if defined (__AVR_ATmega8__)
#elif defined (__AVR_ATmega168__)
#elif defined (__AVR_ATmega168A__)
#elif defined (__AVR_ATmega168P__)
#elif defined (__AVR_ATmega328__)
#elif defined (__AVR_ATmega328P__)*/

  // set reference to AREF, and mux to read the internal 1.1V
  // REFS1 = 0, REFS0 = 0, MUX3..0 = 1110
  ADMUX = _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

  // Enable the ADC
  ADCSRA |= _BV(ADEN);

  // Wait for voltage to become stable after changing the mux.
  delay(20);

  // Start ADC
  ADCSRA |= _BV(ADSC);

  // wait for the ADC to finish
  while (bit_is_set(ADCSRA,ADSC));

  // Read the ADC result
  // The 16-bit ADC register is 'ADC' or 'ADCW'
  unsigned int raw = ADCW;

  // Calculate the Aref.
  volt = 1.1/(float) raw * 1023.0;

/*#elif defined (__AVR_ATmega32U4__)
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
#endif*/


  // Try to return to normal.
  analogReference(EXTERNAL);
  analogRead(A0);            // the mux is set, throw away ADC value
  delay(20);                 // wait for voltages to become stable

  return volt;
}

/***********************************************
 *  Function: KeyPad()
 *  Description: Give an input
 ************************************************/
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
 *  Description: Convert keypad presses to system
 *  setting commands.
 ************************************************/
void printKeyPad(char k){

  if(k == 'C'){
    Serial.println("Key: Channel Select");
    setCommand = "CHANNEL";
  }else if(k == 'N'){
    Serial.println("Key: Channel Enable/Disable");

    if(setChannel == 1)
      setCommand = "ENABLE";
    else
      setCommand = "ENABLE1"

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
        if(setChannel == 0){

          if(setCommand == "VOLTAGE")
            CH1VoltSet = setValue.toFloat();
          else if(setCommand == "CURRENT")
            CH1CurrSet = setValue.toFloat();
          else if(setCommand == "ENABLE")
            CH1EN != CH1EN;

        }else if(setChannel == 1){

          if(setCommand == "VOLTAGE")
            CH1VoltSet = setValue.toFloat();
          else if(setCommand == "CURRENT")
            CH1CurrSet = setValue.toFloat();
          else if(setCommand == "ENABLE")
            CH2EN != CH2EN;
        }

      }
    }else
      setChannel = setValue.toInt();
}

/***********************************************
 *  Function: checkRangeCommand()
 *  Description: Check if the voltage and current
 *  are within specification of the "set command".
 *  Where: Voltage [0, 14.0]V, Current [0, 1.5]A
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
  String lcdText = "";
  lcd_delayi = millis();

  //calculate the time diff. from last LCD update
  //lcd_delayi is a larger number than lcd_delayf
  lcd_delay = lcd_delayi - lcd_delayf;

  if(lcd_delay > 10){
    clearLCD();

    if(keypadDone == 0){

      //Show which option
      lcd.setCursor(0, 0);
      lcd.print(setCommand);
      lcd.setCursor(7, 0);
      lcd.print(":");
      //display pressed value
      lcd.setCursor(8, 0);
      lcd.print(setValue);

    }else{
      lcdText = String(lcd_delay);

      lcd.setCursor(0, 0);
      lcd.print("C1");
      lcd.setCursor(3, 0);
      lcd.print("V:");
      lcd.setCursor(5,0);
      lcd.print(CH1VoltRead, 1);//Display CH1 voltage

      lcd.setCursor(10, 0);
      lcd.print("I:");
      lcd.setCursor(12, 0);
      lcd.print(CH1CurrRead, 1); //Display CH1 Current(I)

      lcd.setCursor(0, 1);
      lcd.print("C2");
      lcd.setCursor(3, 1);
      lcd.print("V:");
      lcd.setCursor(5,1);
      lcd.print(CH2VoltRead, 1);//Display CH2 voltage

      lcd.setCursor(10, 1);
      lcd.print("I:");
      lcd.setCursor(12, 1);
      lcd.print(CH2CurrRead, 1); //Display CH2 Current(I)

      lcd_delayf = millis();
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

  for(int i = 0; i < 10; i++){
    v1 += analogRead(CH1VoltPin);
    //v2 += analogRead(CH2VoltPin);
  }

  v1 /= 10; //Get the average of 10 reads
  v1 /= 1023; //Convert to float
  v1 *= 5; //Reading with a 5V reference
  v1 *= 7.263/2.18; //Voltage divider resistors

  for(int i = 0; i < 10; i++){
    v2 += analogRead(CH1VoltPin);
    //v2 += analogRead(CH2VoltPin);
  }

  v2 /= 10; //Get the average of 10 reads
  v2 /= 1023; //Convert to float
  v2 *= 5; //Reading with a 5V reference
  v2 *= 7.263/2.18; //Voltage divider resistors

  //Update Voltage Reading Output
  CH1VoltRead = v1;
  CH2VoltRead = v2;
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

  for(int i = 0; i < 10; i++){
    c1 += analogRead(CH1CurrPin);
  }

  c1 /= 10; //Get the average of 10 reads
  c1 /= 1023; //Convert from Analog
  c1 *= 5; //Reading with a 5V reference
  c1 *= (1000 + 860)/1000; //Voltage Divider
  c1 /= 5;//scale down the 50V/V Gain

  for(int i = 0; i < 10; i++){
    c2 += analogRead(CH1CurrPin);
  }

  c2 /= 10; //Get the average of 10 reads
  c2 /= 1023; //Convert from Analog
  c2 *= 5; //Reading with a 5V reference
  c2 *= (1000 + 860)/1000; //Voltage Divider
  c2 /= 5;//scale down the 50V/V Gain

  CH1CurrRead = c1;
  CH2CurrRead = c2;
}

/*
Vtarget = Vold(Itarget/Iold)
if =1, then no adj.

*/
/***********************************************
 *  Function: targetVoltage()
 *  Description:  If the voltage goes above or below
 *  the target voltage, then adjust the PWM.
 ************************************************/
void targetVoltage(){

}

/***********************************************
 *  Function: targetCurrent()
 *  Description: If the current goes above or below
 *  the target current, then adjust the PWM.
 ************************************************/
void targetCurrent(){

}

/***********************************************
 *  Function: detectShort()
 *  Description: If the current goes over 1.5A then
 *  the output of the channel needs to be opened.
 *  If the current does exceed 1.5A by 100mA, then
 *  send a signal to relay to open the circuit.
 ************************************************/
void  detectShort(){
  if(CH1CurrRead >= 1.6){
    /*RELAY CUTOFF*/
    digitalWrite(CH1Relay, HIGH);
  }

  if(CH2CurrRead >= 1.6){
    /*RELAY CUTOFF*/
    digitalWrite(CH2Relay, HIGH);
  }
}
