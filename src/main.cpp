
#include "Arduino.h"
#include "config.h"

// set up the 'control' feed
AdafruitIO_Feed *control_feed = io.feed("control");
AdafruitIO_Feed *error_feed = io.feed("error");
AdafruitIO_Feed *actual_feed = io.feed("actual-value");
AdafruitIO_Feed *desired_feed = io.feed("desired-value");
AdafruitIO_Feed *kp_feed = io.feed("kp");
AdafruitIO_Feed *ki_feed = io.feed("ki");
AdafruitIO_Feed *kd_feed = io.feed("kd");

double Kp = 1;
double Ki = 0;
double Kd = 0;

double sample_period =10;
unsigned long last_time;
double total_error = 0;
double last_error = 0;
bool error_type = true; //false is negative error true is positive error
double control_signal;
double actual_ph_value = 0;
double desired_ph_value = 5.0;
double error;


void pid_control() {
  unsigned long current_time = millis();
  unsigned long delta_time = current_time - last_time;

  if (delta_time >= sample_period) {
    error = desired_ph_value - actual_ph_value;
   // if((error) == 0.00) total_error = 0;
  //  else  total_error += error;
    
  if (total_error >= 255) total_error = 255;
  if (total_error <= 0) total_error = 0;

  //check if error is positive or negative;
  if (error < 0) error_type = false;
  else if (error > 0) error_type = true;
  //-------------------------------------
  double delta_error = error - last_error;
  last_error = error;
  
  control_signal = (Kp * error) + (Ki * total_error * sample_period) + ((Kd / sample_period) * delta_error);
  if (control_signal >= 255) control_signal = 255;
  if (control_signal <= -255) control_signal = -255;
  
  last_time = current_time;

  }
}

void handleDesiredValue(AdafruitIO_Data *data){
  Serial.print("Received -> ");
  Serial.println(data->value());
  desired_ph_value = atof(data->value());
}

void handlekpValue(AdafruitIO_Data *data){
  Serial.print("Received -> ");
  Serial.println(data->value());
  Kp= atof(data->value());
}

void handlekiValue(AdafruitIO_Data *data){
  Serial.print("Received -> ");
  Serial.println(data->value());
  Ki= atof(data->value());
}

void handlekdValue(AdafruitIO_Data *data){
  Serial.print("Received -> ");
  Serial.println(data->value());
  Kd= atof(data->value());
}

void setup() {

  Serial.begin(115200);
  while(! Serial);
  Serial.print("Connecting to Adafruit IO");


  io.connect();
  desired_feed->onMessage(handleDesiredValue);
  kp_feed->onMessage(handlekpValue);
  ki_feed->onMessage(handlekiValue);
  kd_feed->onMessage(handlekdValue);

  while(io.mqttStatus() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  desired_feed->get();
  kp_feed->get();
  ki_feed->get();
  kd_feed->get();

  Serial.println();
  Serial.println(io.statusText());

}

void loop() {

  io.run();

  pid_control();

  Serial.print(error);
  Serial.print(",");
  Serial.print(control_signal);
  Serial.print(",");
  Serial.print(desired_ph_value);
  Serial.print(",");
  Serial.println(actual_ph_value);

  control_feed->save(control_signal);
  error_feed->save(error);
  actual_feed->save(actual_ph_value);

   if(error_type == true){ //positive error
      actual_ph_value++;
    }
  if(error_type == false){ //negative error
      actual_ph_value--;
    }

  delay(6000);

}
