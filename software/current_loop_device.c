/*	

	Software for two channel current loop device

	Calibration:
	Adjust DAC output so that current in multimeter 
	Fluke shows 20 mA and record DAC ja ADC values
	repeat this for both channels.

*/

//#include <Wire.h>

//comment this Adafruit for real device
//#include "Adafruit_LiquidCrystal.h"


// for real device
#include <Wire.h>
#include "DFRobot_MCP4725.h"
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h> //Watchdog Timer library

//comment this Adafruit for real device
//Adafruit_LiquidCrystal lcd(0x20);  // set the LCD address to 0x20 for a 16 chars and 2 line display

// for real device 
DFRobot_MCP4725 DAC1;
DFRobot_MCP4725 DAC2;
LiquidCrystal_I2C lcd(0x27,16,2); //I2C address 0x27, 16 column and 2 rows


void write_lcd(float torgue, int rpm, bool dac_select, int error);
void write_dac(float torgue, int rpm);
void init_lcd(void);
void ISR_pushButton(void);
void ISR_encoderChange(void);
int open_loop_error_check(void);
//void print_all_values_to_serial_terminal(float torgue, int rpm, bool dac_select);

#define CLK_PIN 2 //encoder A output
#define DT_PIN 3  //encoder B output
#define SW_PIN 4  //encoder switch pin
#define limit_for_fast_increment_in_ms 100 //time limit in milliseconds before encoder values ​​are read faster
#define calibration_value_for_ADC_ch0 1023 //this is tested when loop current accurately 20.0 mA
#define calibration_value_for_ADC_ch1 1023 //this is tested when loop current accurately 20.0 mA
#define calibration_value_for_DAC_ch0 5000 //this is tested when loop current accurately 20.0 mA
#define calibration_value_for_DAC_ch1 5000 //this is tested when loop current accurately 20.0 mA
#define REF_VOLTAGE1 5135.0 //uncomment for real device, update the actual voltage measured by multimeter
#define REF_VOLTAGE2 5044.0 //uncomment for real device, update the actual voltage measured by multimeter

#define maximum_value_for_torgue 50.00
#define step_for_torgue 0.50
#define minimum_step_for_torgue 0.05
#define maximum_value_for_rpm 3000
#define step_for_rpm 30
#define minimum_step_for_rpm 3

//global variables
bool DAC_select=0;
float torgue=0;
int rpm=0;

void setup()
{
     //uncommetn thise for real device
    Wire.begin(); 
    lcd.init(); 
    lcd.backlight(); 
    
    DAC1.init(MCP4725A0_IIC_Address0, REF_VOLTAGE1);  //DAC1
    DAC2.init(MCP4725A0_IIC_Address1, REF_VOLTAGE2);  //DAC2
    DAC1.outputVoltage(0.0);
    DAC2.outputVoltage(0.0);
	
    Serial.begin(115200);
    
    pinMode(SW_PIN,INPUT);
	  pinMode(DT_PIN,INPUT);
	  pinMode(CLK_PIN,INPUT);
	  delay(100);
	  //lcd.begin(16, 2); //comment this for real divice
	  init_lcd(); //initalize LCD so that values are 0
    Serial.println("setup ready");
	  delay(1000);
	  attachInterrupt(digitalPinToInterrupt(DT_PIN), ISR_encoderChange, CHANGE);
	  attachInterrupt(digitalPinToInterrupt(CLK_PIN), ISR_encoderChange, CHANGE); 
    wdt_enable(WDTO_1S);  //Enable watchdog timer with a timeout of 1 second

}

void loop()
{
  wdt_reset(); 
 	//interrupt reads values for torgue and rpm from rotary encoder
	//interrupt reads DAC_select so values 0 or 1
	int error=open_loop_error_check(); //check if current loop ok
	write_lcd(torgue,rpm,DAC_select,error); //writes torgue, rpm values for LCD and DAC_selec asterisk
	write_dac(torgue,rpm); //write values to the real DAC and current loop output
	print_all_values_to_serial_terminal(torgue,rpm,DAC_select);
  read_push_button_state();
}


void init_lcd(void)
{
	lcd.setCursor(0, 0);
	lcd.print("     ");
	lcd.setCursor(0, 0);
	lcd.print(torgue);
	lcd.setCursor(6, 0);
	lcd.print("Nm");

	lcd.setCursor(0, 1);
	lcd.print("     ");
	lcd.setCursor(0, 1);
	lcd.print(rpm);
	lcd.setCursor(6, 1);
	lcd.print("rpm");

	if(DAC_select==0)
	{
		lcd.setCursor(12, 1);
		lcd.print(" ");
		lcd.setCursor(12, 0);
		lcd.print("*");
	}
	else
	{
		lcd.setCursor(12, 0);
		lcd.print(" ");
		lcd.setCursor(12, 1);
		lcd.print("*");
	}	
}




void write_lcd(float torgue, int rpm, bool DAC_select, int error)
{

	static float old_torgue=0;
	static int old_rpm=0;
	static bool old_DAC_select=0;
	static int error_old=0;

	if(torgue!=old_torgue||error!=error_old) //write only if value changed or error status changed
	{
		lcd.setCursor(0, 0);
		lcd.print("     ");
		lcd.setCursor(0, 0);
		if(error==0||error==2)
			lcd.print(torgue);
		else if(error==1||error==3)
			lcd.print("OL"); //error = 1 or 3 so open loop in torgue channel
		lcd.setCursor(6, 0);
		lcd.print("Nm");
	}

	if(rpm!=old_rpm||error!=error_old) //write only if value changed or error status changed
	{
		lcd.setCursor(0, 1);
		lcd.print("     ");
		lcd.setCursor(0, 1);
		if(error==0||error==1)
			lcd.print(rpm);
		else if(error==2||error==3) //error = 2 or 3 so open loop in rpm channel
			lcd.print("OL");
		lcd.setCursor(6, 1);
		lcd.print("rpm");
	}

	if(DAC_select!=old_DAC_select) //write only if value changed
	{
		if(DAC_select==0)
		{
			lcd.setCursor(12, 1);
			lcd.print(" ");
			lcd.setCursor(12, 0);
			lcd.print("*");
		}
		else
		{
			lcd.setCursor(12, 0);
			lcd.print(" ");
			lcd.setCursor(12, 1);
			lcd.print("*");
		}
	}

	//store old values to static variables
	old_torgue=torgue;
	old_rpm=rpm;
	old_DAC_select=DAC_select;
	error_old=error;
}



void read_push_button_state(void) //if push-button pressed on encoder
{
    static long timex=0;

    if(digitalRead(SW_PIN)==0&&millis()>timex+200)
	  {
        DAC_select=!DAC_select;
        timex=millis();
    }
}

void ISR_encoderChange(void)
{

//https://www.youtube.com/watch?v=fgOfSHTYeio

static unsigned long _lastIncReadTime = micros(); 
static unsigned long _lastDecReadTime = micros(); 
int _pauseLength = 25000;

  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(CLK_PIN)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(DT_PIN)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) 
  {        // Four steps forward

    if((micros() - _lastIncReadTime) < _pauseLength) 
    {
      if(DAC_select==1&&rpm<=(maximum_value_for_rpm-step_for_rpm))
				rpm=rpm+step_for_rpm;
			else if(DAC_select==1&&rpm>(maximum_value_for_rpm-step_for_rpm))
				rpm=maximum_value_for_rpm;

      if(DAC_select==0&&torgue<=(maximum_value_for_torgue-step_for_torgue))
				torgue=torgue+step_for_torgue;
			else if(DAC_select==0&&torgue>(maximum_value_for_torgue-step_for_torgue))
				torgue=maximum_value_for_torgue;	
    }
    else
    {
      if(DAC_select==1&&rpm<=(maximum_value_for_rpm-minimum_step_for_rpm))
				rpm=rpm+minimum_step_for_rpm;
      
      if(DAC_select==0&&torgue<=(maximum_value_for_torgue-minimum_step_for_torgue))
				torgue=torgue+minimum_step_for_torgue;
    }
    _lastIncReadTime = micros();
 
    encval = 0;
  }
  else if( encval < -3 ) 
  {        // Four steps backward

    if((micros() - _lastDecReadTime) < _pauseLength) 
    {
			if(DAC_select==1&&rpm>=step_for_rpm)
				rpm=rpm-step_for_rpm;
			else if(DAC_select==1&&rpm<step_for_rpm)
				rpm=0;
      
      if(DAC_select==0&&torgue>=step_for_torgue)
				torgue=torgue-step_for_torgue;
      else if(DAC_select==0&&torgue<step_for_torgue)
				torgue=0;
    }
    else
    {
			if(DAC_select==1&&rpm>=minimum_step_for_rpm)
				rpm=rpm-minimum_step_for_rpm;

      if(DAC_select==0&&torgue>=minimum_step_for_torgue)
				torgue=torgue-minimum_step_for_torgue;
    }
    _lastDecReadTime = micros();
    encval = 0;
  }
  
		if(torgue>=maximum_value_for_torgue) //for safety 
			torgue=maximum_value_for_torgue;
		else if(torgue<0)
			torgue=0;

		if(rpm>=maximum_value_for_rpm) //for safety
			rpm=maximum_value_for_rpm;
		else if(rpm<0)
			rpm=0;

}

void write_dac(float torgue, int rpm)
{
	static int rpm_old=0;
	static float torgue_old=0;

	if(rpm!=rpm_old) //if different than old value
	{
		//for example calibrated value 4087 and wanted 3000 rpm so 4087/3000*3000=4087
		int dac_rpm=(float)calibration_value_for_DAC_ch1/maximum_value_for_rpm*rpm;
        //here real DAC write
        DAC2.outputVoltage(dac_rpm);	
	}

	if(torgue!=torgue_old) //if different than old value
	{
        //for example calibrated value 4081 and wanted 50.00 torgue so 4081/50*50=4081
		int dac_torgue=(float)calibration_value_for_DAC_ch0/maximum_value_for_torgue*torgue;
        //here real DAC write
        DAC1.outputVoltage(dac_torgue);	
	}
	torgue_old=torgue;
	rpm_old=rpm;
}

int open_loop_error_check(void)
{
	//calculates feedback value from ADC (torgue and rpm) and scales to 0-4095
	int feedback_adc_value_for_torgue=(float)calibration_value_for_DAC_ch0/calibration_value_for_ADC_ch0*analogRead(A0);
	int feedback_adc_value_for_rpm=(float)calibration_value_for_DAC_ch1/calibration_value_for_ADC_ch1*analogRead(A1);
	int error_ch0=0;
	int error_ch1=0;

	//calculates comparable DAC value from wanted torgue and scales to 0-4095
	int dac_value_for_torgue=(float)calibration_value_for_DAC_ch0/maximum_value_for_torgue*torgue;
	
	//calculates comparable DAC value from wanted rpm and scales to 0-4095
	int dac_value_for_rpm=(float)calibration_value_for_DAC_ch1/maximum_value_for_rpm*rpm;

	//if wanted torgue > 100/4095*5V=0,122V and feeback less than 20/4095*5V=0,024V gives alarm
	if(dac_value_for_torgue>100&&feedback_adc_value_for_torgue<20)
		error_ch0=1; //maybe open loop error in CH0

	//if wanted rpm > 100/4095*5V=0,122V and feeback less than 20/4095*5V=0,024V gives alarm
	if(dac_value_for_rpm>100&&feedback_adc_value_for_rpm<20)
		error_ch1=2; //open loop error in CH1

	/*
	error = 1 torgue channel error
	error = 2 rpm channel error
	eroor = 3 torgue and rpm channel error	
	*/

	return (error_ch0+error_ch1);
}

void print_all_values_to_serial_terminal(float torgue, int rpm, bool dac_select)
{
    static int rpm_old=0;
	static float torgue_old=0;
    static bool old_DAC_select=0;

	if(rpm!=rpm_old) //if different than old value
	    {
        int dac_rpm=(float)calibration_value_for_DAC_ch1/maximum_value_for_rpm*rpm;
		Serial.print("RPM 0-3000:");
		Serial.println(rpm);
		Serial.print("DAC rpm 0-5000:");
		Serial.println(dac_rpm);
        }
    
    if(torgue!=torgue_old) //if different than old value
	    {        
        int dac_torgue=(float)calibration_value_for_DAC_ch0/maximum_value_for_torgue*torgue;
        Serial.print("Torgue 0-50.00:");
		Serial.println(torgue);	
		Serial.print("DAC torgue 0-5000:");
		Serial.println(dac_torgue);		
        }

    if(DAC_select!=old_DAC_select) //write only if value changed
	    {
        Serial.print("DAC select: ");
		Serial.println(DAC_select);
        }

        torgue_old=torgue;
	    rpm_old=rpm;
        old_DAC_select=DAC_select;
}