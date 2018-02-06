#include <scpiparser.h>
#include <Arduino.h>

struct scpi_parser_context ctx;

scpi_error_t identify(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t get_voltage(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t get_voltage_2(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t get_voltage_3(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t set_voltage(struct scpi_parser_context* context, struct scpi_token* command);
scpi_error_t set_voltage_2(struct scpi_parser_context* context, struct scpi_token* command);

int row[4] = {2, 3, 4, 5};
int col[4] = {6, 7, 8, 9};

//these keys will represent what is being pressed on the keypad
char key[4][4] = { {'1', '2', '3', 'C'},
                   {'4', '5', '6', 'V'},
                   {'7', '8', '9', 'I'},
                   {'0', '.', 'R', 'E'} };

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

 

  /*
   * Next, we set our outputs to some default value.
   */
  analogWrite(3, 0);
  analogWrite(5, 0);

  Serial.begin(9600);

  //Set the row to INPUT with the internal pull-up resistors
  for(int i = 0; i < 4; i++)
    pinMode(row[i], INPUT_PULLUP);

  //set the columns to output
  for(int i = 0; i < 4; i++)
    pinMode(col[i], OUTPUT);
}

void loop()
{
  char line_buffer[256];
  unsigned char read_length;

  //check for keypad presses
  check_keypad();

  
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

void check_keypad()
{
  //Loop through the columns
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
        //print the key to the serial monitor for debugging purposes.
        Serial.print("Key: ");
        Serial.println(key[i][j]);

        //Prevent repeated keystrokes by holding key down
        while(digitalRead(row[j]) == 0)
          continue;
      }
    }

    //set the column back to high
    digitalWrite(col[i], HIGH);    
  }

  //delay for one ms before going through the loop again.
  delay(1);
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
