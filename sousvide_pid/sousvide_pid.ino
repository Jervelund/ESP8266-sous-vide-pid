#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include "max6675.h" // Thermocouple library


// --- User settings --- //
// PID settings
double pid_setpoint[]    = { 0.0  }; // Degrees Celcius
double pid_dt[]          = { 5.0  }; // Time seconds
double pid_kp[]          = { 0.1  }; // Kp constant
double pid_ki[]          = { 0.0  }; // Ki constant
double pid_kd[]          = { 0.0  }; // Kd constant
double pid_anti_windup[] = { 5000 }; // Anti windup - clips integral value

// WiFi credentials
const char* ssid     = "SSID";
const char* password = "Password";

// Influx config - assuming you want to capture data using InfluxDB
const char* influxhost = "192.168.1.2"; // IP of server running Influx
const uint16_t influxUDPport = 8086;

// Time settings
unsigned long time_output_window_size = 20000; // time in milis
unsigned long time_output_window_size_start;
unsigned long time_pid = 1000; // Time in ms
unsigned long time_pid_start;
unsigned long time_temp = 500;
unsigned long time_temp_start;
unsigned long time_udp = 5000;
unsigned long time_udp_start;

// Pin constants
int thermoMISO = D7;
int thermoCS = D6;
int thermoSCLK = D8;
int transmitPin = D4;
int heaterPin = D3;

// --- Program constants and variables --- //
// Default PID values
double pid_previous_error[] = {0};
double pid_integral[]       = {0};
double pid_derivative[]     = {0};
double pid_output[]         = {0};
double pid_out_p[]          = {0};
double pid_out_i[]          = {0};
double pid_out_d[]          = {0};

// Placeholders for temperatures used as inputs for PID (allows async operation)
double temperatures[] = {0.0};
uint8_t rolling_idx = 0;
double rolling[16]; // Make sure to initialize all
double rolling_sum = 0;
double scaledOut = 0;
uint8_t heaterPinStatus = 0;

// Setup UDP class for transmitting packets to Influx
WiFiUDP udp;

// Setup MAX6675 library
MAX6675 thermocouple(thermoSCLK, thermoCS, thermoMISO);

// Setup ESP webserver on port 80
ESP8266WebServer server(80);

void http_html() {
  String msg = "<html><body>";
  msg += "<form>Setpoint: <input type=number name=setpoint value=\""+String(pid_setpoint[0], 5)+"\" /></form>";
  msg += "<form>Kp: <input type=number name=kp value=\""+String(pid_kp[0], 5)+"\" /></form>";
  msg += "<form>Ki: <input type=number name=ki value=\""+String(pid_ki[0], 5)+"\" /></form>";
  msg += "<form>Kd: <input type=number name=kd value=\""+String(pid_kd[0], 5)+"\" /></form>";
  msg += "<form>dt: <input type=number name=dt value=\""+String(pid_dt[0], 5)+"\" /></form>";
  msg += "<form>integ: <input type=number name=integ value=\""+String(pid_integral[0], 5)+"\" /></form>";
  msg += "<form>time_pid: <input type=number name=time_pid value=\""+String(time_pid)+"\" /></form>";
  msg += "<form>time_output_window: <input type=number name=time_window value=\""+String(time_output_window_size)+"\" /></form>";
  msg += "<form>time_udp: <input type=number name=time_udp value=\""+String(time_udp)+"\" /></form>";
  msg += "</body></html>";
  server.send(200, "text/html", msg);
}

void http_json() {
  String msg = "{";
  msg += "\"setpoint\":"+String(pid_setpoint[0], 5);
  msg += ",\"kp\":"+String(pid_kp[0], 5);
  msg += ",\"ki\":"+String(pid_ki[0], 5);
  msg += ",\"kd\":"+String(pid_kd[0], 5);
  msg += ",\"dt\":"+String(pid_dt[0], 5);
  msg += ",\"integ\":"+String(pid_integral[0], 5);
  msg += ",\"time_pid\":"+String(time_pid);
  msg += ",\"time_output_window_size\":"+String(time_output_window_size);
  msg += ",\"time_udp\":"+String(time_udp);
  msg += "}";
  server.send(200, "application/json", msg);
}

void http_set() {
  for (uint8_t i=0; i<server.args(); i++){
    String n = server.argName(i);
    String v = server.arg(i);
    if(n == "setpoint")
      pid_setpoint[0] = v.toFloat();
    if(n == "kp")
      pid_kp[0] = v.toFloat();
    if(n == "ki")
      pid_ki[0] = v.toFloat();
    if(n == "kd")
      pid_kd[0] = v.toFloat();
    if(n == "dt")
      pid_dt[0] = v.toFloat();
    if(n == "integ")
      pid_integral[0] = v.toFloat();
    if(n == "time_pid")
      time_pid = v.toInt();
    if(n == "time_window")
      time_output_window_size = v.toInt();
    if(n == "time_udp")
      time_udp = v.toInt();
  }
  http_html();
}

uint8_t MAC_array[6];
char MAC_char[18];

char buf[600]; // Should hold data for 3 lines of measurements
char* buf_idx = buf; // Setup buffer index for first transmission

// Function to process an iteration of PID control loop
void processPID(uint8_t i, double input){
  double error = pid_setpoint[i] - input;
  double integ = pid_integral[i] + error * pid_dt[i];
  pid_integral[i] = constrain(integ,-pid_anti_windup[i],pid_anti_windup[i]); // Constrain integral to windup limit
  pid_derivative[i] = (error - pid_previous_error[i]) / pid_dt[i];
  pid_out_p[i] = pid_kp[i] * error;
  pid_out_i[i] = pid_ki[i] * pid_integral[i];
  pid_out_d[i] = pid_kd[i] * pid_derivative[i];
  pid_output[i] = pid_out_p[i] + pid_out_i[i] + pid_out_d[i];
  pid_previous_error[i] = error;
}

char next_sep = ' ';

// Functions for preparing UDP packets for Influx
void start_series(char* str){
  *buf_idx++ = '\n'; // Newline seperates measurements
  strcpy(buf_idx, str);
  buf_idx += strlen(str); // Increment by size of str, so idx points to end
  next_sep = ' ';
}

// Original function: http://forum.arduino.cc/index.php?topic=44262.0 (fixed to pad decimals correcly)
void append_point(char* str, double f){
  *buf_idx++ = next_sep; // Append seperator
  strcpy(buf_idx, str); // Copy string
  buf_idx += strlen(str); // Move buffer pointer
  *buf_idx++ = '='; // Append equals
  // If double is megative, but has no integer component, make sure to get sign right.
  if(f > -1 && f < 0) *buf_idx++ = '-';
  // Magic to append integers
  long heiltal = (long)f;
  itoa(heiltal, buf_idx, 10);
  while (*buf_idx != '\0') buf_idx++;
  // Magic to append decimals
  long desimal = abs((long)((f - heiltal) * 100000));
  if(desimal != 0){ // Only print decimals if we actually have something to print, in the decimals
    *buf_idx++ = '.';
    if(desimal < 10000)  *buf_idx++ = '0';
    if(desimal < 1000)   *buf_idx++ = '0';
    if(desimal < 100)    *buf_idx++ = '0';
    if(desimal < 10)     *buf_idx++ = '0';
    itoa(desimal, buf_idx, 10);
    buf_idx += strlen(buf_idx); // Increment by number of decimals printed
    // Remove unnecessary zeroes
    if(buf_idx[-1] == '0'){*--buf_idx = '\x00';
      if(buf_idx[-1] == '0'){*--buf_idx = '\x00';
        if(buf_idx[-1] == '0'){*--buf_idx = '\x00';
          if(buf_idx[-1] == '0'){*--buf_idx = '\x00';
            if(buf_idx[-1] == '0'){*--buf_idx = '\x00';
              if(buf_idx[-1] == '0'){*--buf_idx = '\x00';
                if(buf_idx[-1] == '.'){*--buf_idx = '\x00';}
              }
            }
          }
        }
      }
    }
  }
  next_sep = ','; // Setup next seperator, between fields
}

void append_pid(uint8_t i){
  append_point("setpoint", pid_setpoint[i]);
  append_point("dt",       pid_dt[i]);
  append_point("kp",       pid_kp[i]);
  append_point("ki",       pid_ki[i]);
  append_point("kd",       pid_kd[i]);
  append_point("err",      pid_previous_error[i]);
  append_point("integ",    pid_integral[i]);
  append_point("deriv",    pid_derivative[i]);
  append_point("out",      pid_output[i]);
  append_point("out_p",    pid_out_p[i]);
  append_point("out_i",    pid_out_i[i]);
  append_point("out_d",    pid_out_d[i]);
}

void transmitUDP(){
  // Print to console
  Serial.println(buf+1); // Skip first char, which is always garbage
  // Send to influxDB
  udp.beginPacket(influxhost, influxUDPport);
  udp.write(buf+1, buf_idx-buf-1); // buf_idx-buf = length of packet. Minus 1 as garbage at start should not be sent.
  udp.endPacket();
  // Resets buffer pointer
  buf_idx = buf;
}

// Implementation specific functions
void readTemperatures(){
    // This should query all sensors for temperatures and store them in the temperatures array
    rolling[rolling_idx++] = thermocouple.readCelsius();
    rolling_idx &= 0xF; // Mask to 16 length
    temperatures[0] = 0;
    for(uint8_t i=0;i<16;i++){
      temperatures[0] += rolling[i];
    }
    temperatures[0] /= 16;
}


void setup(void) {
  Serial.begin(115200);

  WiFi.macAddress(MAC_array);
  for (int i = 0; i < sizeof(MAC_array); ++i){
    sprintf(MAC_char,"%s%02x:",MAC_char,MAC_array[i]);
  }
  Serial.println(MAC_char);

  // We start by connecting to the specified WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  Serial.println();
  WiFi.begin(ssid, password);
  // Initialize all points for rolling average
  rolling[rolling_idx++] = 0; // thermocouple.readCelsius();
  for(;rolling_idx < 16;rolling_idx++){
    rolling[rolling_idx] = rolling[0];
  }
  rolling_idx = 0;

  while (WiFi.status() != WL_CONNECTED && WiFi.localIP().toString() == "0.0.0.0") {
    delay(500);
    if(WiFi.status() == WL_CONNECT_FAILED)
      Serial.println("  CONNECT FAILED");
    if(WiFi.status() == WL_NO_SSID_AVAIL)
      Serial.println("  NO SSID");
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  if (MDNS.begin("sousvide")) {
    Serial.println("MDNS responder started");
    Serial.println("Reachable at: http://sousvide.local");
  }
  server.on("/", http_html);
  server.on("/json", http_json);
  server.on("/pid", http_set);
  server.begin();
  Serial.println("HTTP server started");
  Serial.println();

  // Setup time windows - do this here, as we otherwise would do 4-5 pid runs as setup time is quite long.
  time_output_window_size_start = millis();
  time_pid_start = millis();
  time_udp_start = millis();
}

// --- Main loop --- //
unsigned long now_diff = 0;
void loop(void) {
  unsigned long now = millis();
  if(now - time_temp_start > time_temp){
    time_temp_start += time_temp;
    // Read temperature
    readTemperatures();
  }

  if(now - time_pid_start > time_pid){
    time_pid_start += time_pid;

    // Process PID
    processPID(0, temperatures[0]); // Process PID#0 with specified temperature

    // Set output
    // Output for PWM controlled peripherals:
    ///// uint8_t out = (uint8_t)constrain(pid_output[0], 0.0, 255.0, ); // Make sure we don't attempt to output something negative
    ///// append_point("heater", out);
    scaledOut = pid_output[0]*time_output_window_size; // Scale output to window size
  }

  if(now - time_udp_start > time_udp){
    time_udp_start += time_udp;
    start_series("sous");
    append_point("plate", temperatures[0]);
    start_series("pid");
    append_pid(0);
    append_point("out_scaled", scaledOut);
    append_point("heater", heaterPinStatus);
    append_point("window_state", now_diff);
    // Send measurements to influx
    transmitUDP();
  }

  // Set output for outer loop - http://playground.arduino.cc/Code/PIDLibraryRelayOutputExample
  now_diff = now - time_output_window_size_start; // Calculate elapsed time in current window
  if(now - time_output_window_size_start > time_output_window_size) { //shift the output window
    time_output_window_size_start += time_output_window_size;
    now_diff = now - time_output_window_size_start; // Recalculate elapsed time in current window
  }
  if(scaledOut > now_diff){ // If scaledOut is less than elapsed time in current window
    if(heaterPinStatus == 0){
      heaterPinStatus = 1;
      digitalWrite(heaterPin,HIGH);
    }
  } else {
    if(heaterPinStatus == 1){
      heaterPinStatus = 0;
      digitalWrite(heaterPin,LOW);
    }
  }

  // Make sure to handle any HTTP requests
  server.handleClient();
}

