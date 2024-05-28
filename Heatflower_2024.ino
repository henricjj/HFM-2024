#include <Adafruit_MCP9600.h>
#include <Adafruit_MCP9601.h>

// Setup Library
#include <math.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"
#include "HX711.h"
#include <Adafruit_GFX.h> 
#include <Arduino.h> //display
#include <LiquidCrystal_I2C.h> 


/**********************************************
 ***Define variables and it's initial value ***
***********************************************/
// -----------------These can be changed----------------
float Area           = 0.03;    // [m^2] //REMEMBER TO RESET TO 0.04 BEFORE NEXT GROUP
float set_thickness  = 30;      // [m]
float set_T_cool     = 12.5;      // [C] 
float set_T_hot      = 27.5;      // [C]
float S = 0;                    //S calculated by hand     
                                 //as k_ref(TH-TL)/(b_ref*E)
                                 //how to in detail in the 
                                 //technical manual

// -----------Nothing Bellow should be changed---------------
float read_T_cool    = 0.0;       // [C]
float read_T_hot     = 0.0;       // [C]
float k_value        = 0.0;       // [W/m*C]
float read_q         = 0.0;       // [uV]
float q_value        = 0.0;       // [Watt/m^2]
float weight         = 0.0;       // [Kg]
float pressure       = 0.0;       // [KPa]
float set_thickness_m= 0.0;       // [m]
bool sstate = false;              //for steady state function
int count = 0;                    //for steady state function
int cycle_count=0;                //for steady state function
int sstate_sum = 0;               //for steady state function
float errorProcent[40]={};        //for steady state function
float sstate_array[40] = {};      //for steady state function
float error;                      //for steady state function



int menuCounter = 0; //counts the clicks of the rotary encoder between menu items (0-3 in this case)
int menu1_Value = set_T_hot; //value within menu 1
int menu2_Value = set_T_cool; //value within menu 2
int menu3_Value = set_thickness; //value within menu 3
float menu4_Value = pressure; //value within menu 4
float menu5_Value = k_value; //value within menu 5


bool menu1_selected = false; //enable/disable to change the value of menu item
bool menu2_selected = false;
bool menu3_selected = false;
bool menu4_selected = false;
//Note: if a menu is selected ">" becomes "X".

//Defining pins
//Arduino interrupt pins: 2, 3.
const int RotaryCLK = 2; //CLK pin on the rotary encoder
const int RotaryDT = 10; //DT pin on the rotary encoder
const int PushButton = 3; //Button to enter/exit menu

//Statuses for the rotary encoder
int CLKNow;
int CLKPrevious;

int DTNow;
int DTPrevious;

bool refreshLCD = true; //refreshes values
bool refreshSelection = false; //refreshes selection (> / X)



// Setup Heatflux sensor
#define I2C_adress (0x67)
Adafruit_MCP9600 heatflux_amplifier;

// Setup cold temperature sensor
#define I2C_ADDRESS (0x60)
Adafruit_MCP9600 mcp;

//---------------------------------------------
// Setup Loadcell Amplifier  (A0)
#define DOUT  A0
#define CLK  A1
HX711 scale;
float calibration_factor = 5658; 


//---------------------------------------------
// Setup temperatursensor
//#define SENSOR_COOL_BUS 8
#define SENSOR_HOT_BUS 9
//OneWire oneWireCool(SENSOR_COOL_BUS); 
OneWire oneWireHot(SENSOR_HOT_BUS); 
//DallasTemperature temp_sensor_cool(&oneWireCool); 
DallasTemperature temp_sensor_hot(&oneWireHot);     


//------------------------------------------
//set upp display 
LiquidCrystal_I2C lcd(0x27, 20, 4);

//---------------------------------------------
// Setup Peltier and Heatfoil
// NO -- Connected when set to HIGH
 int relay_heatfoil = 6;  
 int relay_peltier  = 11; 

 //---------------------------------------------
 // PID

double Kp, Ki, Kd;
double Ts, last_time;
double integral, previous, output = 0, Ki_integral = 100;
double outMax = 255, outMin = 0;
double filtered_read_q = 0, alpha = 0.1, filtered_q_value = 0;


void setup() {
  pinMode(2, INPUT_PULLUP); //RotaryCLK
  pinMode(10, INPUT_PULLUP); //RotaryDT
  pinMode(3, INPUT_PULLUP); //Button
  pinMode(1, OUTPUT);
  //------------------------------------------------------
  lcd.init();                      // initialize the lcd   
  lcd.backlight();
  //------------------------------------------------------
  lcd.setCursor(0,0); //Defining positon to write from first row, first column .
  lcd.print("HFM");
  lcd.setCursor(0,1); //Second row, first column
  lcd.print("Bachelor project 24"); 
  lcd.setCursor(0,2); //Second row, first column
  lcd.print("version 1.7"); 
  delay(5000); //wait 2 sec
  
  lcd.clear(); //clear the whole LCD
  
  printLCD(); //print the stationary parts on the screen
  //------------------------------------------------------
  //Store states of the rotary encoder
  CLKPrevious = digitalRead(RotaryCLK);
  DTPrevious = digitalRead(RotaryDT);
      
  attachInterrupt(digitalPinToInterrupt(RotaryCLK), rotate, CHANGE); //CLK pin is an interrupt pin
  attachInterrupt(digitalPinToInterrupt(PushButton), pushButton, FALLING); //PushButton pin is an interrupt pin

 Serial.begin(115200);

  //Heatfoil & peltier define output pin
  pinMode(relay_heatfoil, OUTPUT);
  pinMode(relay_peltier, OUTPUT);
  //pinMode(13, OUTPUT);


  
  
  //---------------------------------------------
  // Heatflux and T_cool sensor 
  
   while (!Serial) {
        //delay(10);
      }
  
      // Initialise the driver with I2C_ADDRESS and the default I2C bus. 
      if (! heatflux_amplifier.begin(I2C_adress)) {
          Serial.println("Sensor not found. Check wiring!");
          while (1);
      }

      if (! mcp.begin(I2C_ADDRESS)) {
        Serial.println("Sensor not found. Check wiring!");
        while (1);
    }
  
  
   heatflux_amplifier.setADCresolution(MCP9600_ADCRESOLUTION_18);
   heatflux_amplifier.setThermocoupleType(MCP9600_TYPE_T);
  
   heatflux_amplifier.setFilterCoefficient(7);
   heatflux_amplifier.setAlertTemperature(1, 30);
   heatflux_amplifier.configureAlert(1, true, true);  // alert 1 enabled, rising temp
   heatflux_amplifier.enable(true);
  
   mcp.setADCresolution(MCP9600_ADCRESOLUTION_18);
   mcp.setThermocoupleType(MCP9600_TYPE_K);

   mcp.setFilterCoefficient(3);
   mcp.setAlertTemperature(1, 30);
   mcp.configureAlert(1, true, true);  // alert 1 enabled, rising temp
   mcp.enable(true);

  //---------------------------------------------
   //Loadcell Amplifier
    scale.begin(DOUT, CLK);
    scale.set_scale();
    scale.tare(); //Reset the scale to 0
    long zero_factor = scale.read_average(); //Get a baseline reading
    
  //---------------------------------------------
  //Temperature sensor
   //temp_sensor_cool.begin();
   temp_sensor_hot.begin(); 
   //---------------------------------------------
  //PID
  Kp = 100;
  Ki = 0.15;
  Kd = 0.01;
  last_time = 0;

}

void loop() 
{
  if(refreshLCD == true) //If we are allowed to update the LCD ...
  {
    updateLCD(); // ... we update the LCD ...

    //... also, if one of the menus are already selected...
    if(menu1_selected == true || menu2_selected == true || menu3_selected == true || menu4_selected == true)
    {
     // do nothing
    }
    else
    {
      updateCursorPosition(); //update the position
    }
    
    refreshLCD = false; //reset the variable - wait for a new trigger
  }

  if(refreshSelection == true) //if the selection is changed
  {
    updateSelection(); //update the selection on the LCD
    refreshSelection = false; // reset the variable - wait for a new trigger
  }
  
  
  Serial.println("--------------------------------------------------"); //beginning of 1 cycle
  
  // ---------------------Temperaturesensor------------------- 
  //temp_sensor_cool.requestTemperatures();
  temp_sensor_hot.requestTemperatures();
  
  float temp_cool = mcp.readThermocouple();
  float temp_hot  = temp_sensor_hot.getTempCByIndex(0);
  
  // Kalibrering av temperatursensorer
  read_T_cool =  mcp.readThermocouple();
  read_T_hot  =  temp_sensor_hot.getTempCByIndex(0);
  
  
    Serial.print("Temperature T: ");
    Serial.print(read_T_cool);
    Serial.println(" C");
    //Serial.print('\n');
    //delay(200);
    Serial.print("Temperature H: ");
    Serial.print(read_T_hot);
    Serial.println(" C");
   // Serial.print('\n');
    //delay(200);
    
  
  //---------------------------------------------
  // Regulation of heatfoil, turns off when tempeture goes higher than the set temperature.
  if (read_T_hot >= set_T_hot) {  
    digitalWrite(relay_heatfoil, LOW);
    
  }
  else {
    digitalWrite(relay_heatfoil, HIGH);
    
  }
  
  // Safty regulation of heatfoil, turns of when tempeture goes higher than 70 degrees.
  if (read_T_hot >= 70) { 
    digitalWrite(relay_heatfoil, LOW);
    
  }
  
  //--------------------------------------------- 
  // PID controller, peltier.

  double now = millis();
  Ts = (now - last_time)/1000;
  double PID_error = (set_T_cool - read_T_cool)*-1;  
  double proportional = PID_error;
  if (abs(PID_error) < 6) {
       integral += PID_error*Ts;
  }
  Ki_integral = Ki*integral;
  if (Ki_integral > outMax){
   Ki_integral = outMax;
  }
 else if(Ki_integral < outMin){
   Ki_integral = outMin;
  }
  double derivative = (PID_error - previous)/Ts;
  previous = PID_error;
  double output = Kp*proportional + Ki_integral + Kd*derivative;
  //double output = 250;
  if (output > outMax) {
    output = outMax;
  }
  else if (output < outMin) {
    output = outMin;
  }
  //output = pid(error);
  analogWrite(relay_peltier, output);
  last_time = now;  
 //
  //---------------------------------------------
  //Heatflux sensor outputh
  
  read_q = -heatflux_amplifier.readADC() * 2;    // spänningen från sensorn i uV
  filtered_read_q = alpha*read_q+(1-alpha)*filtered_read_q;
  filtered_q_value = filtered_read_q / (5.5);                     
  q_value = read_q / (5.5);                      // Delar med känsligheten, men spänningen är i uV så de tar ut varandra
  
  Serial.print("Voltage:");  
  Serial.print(read_q);
  Serial.println(" uV");
  Serial.print("HeatFlow:"); 
  Serial.print(q_value);
  Serial.println(" W/m^2");
  Serial.print("HeatFlow filt:"); 
  Serial.print(filtered_q_value);
  Serial.println(" W/m^2");
 
  //---------------Loadcell Amplifier output------------------------------
  scale.set_scale(calibration_factor); 
  weight = (scale.get_units(1)*100*2)*0.001;   
  pressure = ((weight*9.81)/Area)*0.001;      
    Serial.print("Load: ");
    Serial.print(weight); 
    Serial.print(" Kg  , ");
    Serial.print(pressure);
    Serial.print(" kPa");
    Serial.println();
    //delay(1000);
  
  //-------------------Calculation of k_value--------------------------
  set_thickness_m=set_thickness*0.001;
  k_value = (filtered_q_value * set_thickness_m)/(read_T_hot - read_T_cool);
  Serial.print("k-värde:");
  Serial.print(k_value);
  Serial.println(" W/m^2K");
  
  // --------------Calibrated K------------------------------------
  float calibrated_k = (S*read_q*set_thickness_m)/(read_T_hot - read_T_cool);
  
  Serial.print("S: ");
  Serial.println(S);
  Serial.print("Calibrated K: ");
  Serial.print(calibrated_k);
  Serial.println(" W/m^2K");
  //--------------Steady state function--------------------------
 float sumError = 0; 
 float meanError;
  
  if (cycle_count == 30){
    
    if (count <40){
      sstate_array[count]= filtered_q_value;
      count++;
    }
    else {
      count=0;
    }
    cycle_count=0;
  }
  for (int i = 0; i<39 ; ++i){
    error=abs((sstate_array[i]-sstate_array[i+1])/sstate_array[i]);
    errorProcent[i]=error;
    //Serial.print(errorProcent[i]);
    sumError+=errorProcent[i];
  }
  meanError=sumError/40;
  if(meanError <= 0.005 && abs(PID_error) <= 0.1 && abs(read_T_hot-set_T_hot) <= 0.5){
    sstate=true;
    Serial.println("READY, STEADY STATE IS ACHIEVED");
  }
  else{
    sstate=false;
  }
    cycle_count++;
   //--------------------------End off steadystate function-------------------------------------
  
  refreshLCD = true; //refresh screen each cycle
  
  Serial.print("Measurments taken ");
  Serial.println(count);
  
  Serial.print("30 is next measurment ");
  Serial.println(cycle_count);

  Serial.print("Mean Error last 40 measurments ");
  Serial.println(meanError,12);
  //Serial.println(output);
   // Serial.plot(read_T_cool);
  //Serial.println(Ki_integral);
  //Serial.println(filtered_q_value);
  //Serial.println(q_value);



  


  

}

void rotate()
{  
  //-----MENU1--------------------------------------------------------------
  if(menu1_selected == true)
  {
  CLKNow = digitalRead(RotaryCLK); //Read the state of the CLK pin
  // If last and current state of CLK are different, then a pulse occurred  
  if (CLKNow != CLKPrevious  && CLKNow == 1)
  {
    // If the DT state is different than the CLK state then
    // the encoder is rotating in A direction, so we increase
    if (digitalRead(RotaryDT) != CLKNow) 
    {
      if(set_T_hot < 75) //we do not go above 75
      {
        set_T_hot++;  
      }
      else
      {
        set_T_hot = 20;  
      }      
    } 
    else 
    {
      if(set_T_hot < 20) //we do not go below 20
      {
          set_T_hot = 75;
      }
      else
      {
      // Encoder is rotating B direction so decrease
      set_T_hot--;      
      }      
    }    
    menu1_Value=set_T_hot;
  }
  CLKPrevious = CLKNow;  // Store last CLK state
  }
  //---MENU2---------------------------------------------------------------
  else if(menu2_selected == true)
  {
    CLKNow = digitalRead(RotaryCLK); //Read the state of the CLK pin
  // If last and current state of CLK are different, then a pulse occurred  
  if (CLKNow != CLKPrevious  && CLKNow == 1)
  {
    // If the DT state is different than the CLK state then
    // the encoder is rotating in A direction, so we increase
    if (digitalRead(RotaryDT) != CLKNow) 
    {
      if(set_T_cool < 40) //we do not go above 40
      {
        set_T_cool++;  
      }
      else
      {
        set_T_cool = 5;  
      }      
    } 
    else 
    {
      if(set_T_cool < 5) //we do not go below 5
      {
          set_T_cool = 40;
      }
      else
      {
      // Encoder is rotating in B direction, so decrease
      set_T_cool--;      
      }      
    }    
    menu2_Value=set_T_cool;
  }
  CLKPrevious = CLKNow;  // Store last CLK state
  }
  //---MENU3---------------------------------------------------------------
  else if(menu3_selected == true)
  {
    CLKNow = digitalRead(RotaryCLK); //Read the state of the CLK pin
  // If last and current state of CLK are different, then a pulse occurred  
  if (CLKNow != CLKPrevious  && CLKNow == 1)
  {
    // If the DT state is different than the CLK state then
    // the encoder is rotating in A direction, so we increase
    if (digitalRead(RotaryDT) != CLKNow) 
    {
      if(set_thickness < 100) //we do not go above 100
      {
        set_thickness++ ;  
      }
      else
      {
        set_thickness = 0;  
      }      
    } 
    else 
    {
      if(set_thickness < 1) //we do not go below 0
      {
          set_thickness = 1;
      }
      else
      {
      // Encoder is rotating B direction so decrease
      set_thickness--;      
      }      
    }    
    menu3_Value=set_thickness;
  }
  CLKPrevious = CLKNow;  // Store last CLK state
  }
  //---MENU4----------------------------------------------------------------
  else if(menu4_selected == true)
  {
    CLKNow = digitalRead(RotaryCLK); //Read the state of the CLK pin
  // If last and current state of CLK are different, then a pulse occurred  
  
  scale.tare();
  CLKPrevious = CLKNow;  // Store last CLK state
  }
  else //MENU COUNTER----------------------------------------------------------------------------
  {
  CLKNow = digitalRead(RotaryCLK); //Read the state of the CLK pin
  // If last and current state of CLK are different, then a pulse occurred  
  if (CLKNow != CLKPrevious  && CLKNow == 1)
  {
    // If the DT state is different than the CLK state then
    // the encoder is rotating in A direction, so we increase
    if (digitalRead(RotaryDT) != CLKNow) 
    {
      if(menuCounter < 3) //we do not go above 3
      {
        menuCounter++;  
      }
      else
      {
        menuCounter = 0;  
      }      
    } 
    else 
    {
      if(menuCounter < 1) //we do not go below 0
      {
          menuCounter = 3;
      }
      else
      {
      // Encoder is rotating CW so decrease
      menuCounter--;      
      }      
    }    
  }
  CLKPrevious = CLKNow;  // Store last CLK state
  }

  //Refresh LCD after changing the counter's value
  refreshLCD = true;
}
//-------------------------------------------------------------------------
// PID function
 //double pid(double error)
//{
  //double proportional = error;
 // integral += error*Ts;
 // double Ki_integral = Ki*integral;
 // if (Ki_integral > outMax){
  // Ki_integral = outMax;
  //}
 //else if(Ki_integral < outMin){
 //  Ki_integral = outMin;
  //}
  //double derivative = (error - previous)/Ts;
 // previous = error;
 // double output = Kp*proportional + Ki_integral+Kd*derivative;
 // if (output > outMax) {
  //  output = outMax;
  //}
 // else if (output < outMin) {
 //   output = outMin;
 // }
 // serial.println(Ki_integral);
 // return output;
//}

void pushButton()
{
  switch(menuCounter)
  {
     case 0:
     menu1_selected = !menu1_selected;  //we change the status of the variable to the opposite
     break;

     case 1:
     menu2_selected = !menu2_selected;
     break;

     case 2:
     menu3_selected = !menu3_selected;
     break;

     case 3:
     menu4_selected = !menu4_selected;
     break;
  } 
  
  refreshLCD = true; //Refresh LCD after changing the value of the menu
  refreshSelection = true; //refresh the selection ("X")
}

void printLCD()
{
  //These are the values which are not changing the operation
  
  lcd.setCursor(1,0); //1st line, 2nd block
  lcd.print("TH"); //text
  //----------------------
  lcd.setCursor(1,1); //2nd line, 2nd block
  lcd.print("TC"); //text
  //----------------------
  lcd.setCursor(1,2); //3rd line, 2nd block
  lcd.print("t"); //text
  //----------------------
  lcd.setCursor(1,3); //4th line, 2nd block
  lcd.print("P"); //text
  //----------------------
  lcd.setCursor(13,0); //
  lcd.print("q: "); //counts - text 
   //----------------------
  lcd.setCursor(13,1); //
  lcd.print("k: "); //counts - text
     //---------------------
  lcd.setCursor(8,3); //
  lcd.print("[kPa] "); //counts - text
   //----------------------
  lcd.setCursor(7,2); //
  lcd.print("[mm] "); //counts - text


}

void updateLCD()
{  
  lcd.setCursor(15,0); //1st line, 18th block
  lcd.print(filtered_q_value); //counter (0 to 3)
  
  lcd.setCursor(15,1); //1st line, 18th block
  lcd.print(k_value); //counter (0 to 3)

  lcd.setCursor(4,0); //1st line, 10th block
  lcd.print("   "); //erase the content by printing space over it
  lcd.setCursor(4,0); //1st line, 10th block
  lcd.print(menu1_Value); //print the value of menu1_Value variable

  lcd.setCursor(4,1);
  lcd.print("   ");
  lcd.setCursor(4,1);
  lcd.print(menu2_Value); //
  
  lcd.setCursor(4,2);
  lcd.print("   ");
  lcd.setCursor(4,2);
  lcd.print(menu3_Value); //

  lcd.setCursor(3,3);
  lcd.print("   ");
  lcd.setCursor(3,3);
  lcd.print(pressure); 

  lcd.setCursor(7,0);
  lcd.print("   ");
  lcd.setCursor(7,0); //
  lcd.print(read_T_hot); //counts - text

  lcd.setCursor(7,1);
  lcd.print("   ");
  lcd.setCursor(7,1); //
  lcd.print(read_T_cool); //counts - text
   

  if (sstate==false) {
    lcd.setCursor(14,2);
    lcd.print("      ");
    lcd.setCursor(14,2);
    lcd.print(" "); 
   }
  if(sstate==true){
    lcd.setCursor(14,2);
    lcd.print("   ");
    lcd.setCursor(14,2);
    lcd.print("Ready"); 
  }
    
}


void updateCursorPosition()
{
  //Clear display's ">" parts 
  lcd.setCursor(0,0); //1st line, 1st block
  lcd.print(" "); //erase by printing a space
  lcd.setCursor(0,1);
  lcd.print(" "); 
  lcd.setCursor(0,2);
  lcd.print(" "); 
  lcd.setCursor(0,3);
  lcd.print(" "); 
     
  //Place cursor to the new position
  switch(menuCounter) //this checks the value of the counter (0, 1, 2 or 3)
  {
    case 0:
    lcd.setCursor(0,0); //1st line, 1st block
    lcd.print(">"); 
    break;
    //-------------------------------
    case 1:
    lcd.setCursor(0,1); //2nd line, 1st block
    lcd.print(">"); 
    break;
    //-------------------------------    
    case 2:
    lcd.setCursor(0,2); //3rd line, 1st block
    lcd.print(">"); 
    break;
    //-------------------------------    
    case 3:
    lcd.setCursor(0,3); //4th line, 1st block
    lcd.print(">"); 
    break;
  }
}

void updateSelection()
{
  //When a menu is selected ">" becomes "X"

  if(menu1_selected == true)
  {
    lcd.setCursor(0,0); //1st line, 1st block
    lcd.print("X"); 
  }
  //-------------------
   if(menu2_selected == true)
  {
    lcd.setCursor(0,1); //2nd line, 1st block
    lcd.print("X"); 
  }
  //-------------------
  if(menu3_selected == true)
  {
    lcd.setCursor(0,2); //3rd line, 1st block
    lcd.print("X"); 
  }
  //-------------------
  if(menu4_selected == true)
  {
    lcd.setCursor(0,3); //4th line, 1st block
    lcd.print("X"); 
  }
}
