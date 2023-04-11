#define MEASURE_CHOICE 25
#define SETTINGS_CHOICE 45
#define MENU_ARROW_X_POS 31

#define DONT_STOP_MEASURE_CHOICE 65
#define STOP_MEASURE_CHOICE 85
#define MEASURE_ARROW_Y_POS 60
#define MEASUREMENT_X_POS 10
#define CO_Y_POS 15
#define CO2_Y_POS 30
#define PM_Y_POS 45
#define CO_NUM_OFFSET 35
#define CO2_NUM_OFFSET 35
#define PM_NUM_OFFSET 35
#define CO_PPM_OFFSET 85
#define CO2_PPM_OFFSET 85
#define PM_PPM_OFFSET 85
#define TEN_SEC 20
#define THIRTY_SEC 55
#define ONE_MIN 92
#define RECORD_TIME_OPTION_Y_POS 30

void clearScreen() 
{
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

void printMenuScreen(int arrowPosition) 
{
  u8g2.setDrawColor(1);
  //u8g2.setFont(u8g2_font_tinytim_tr);  // choose a suitable font
  u8g2.setFont(u8g2_font_8x13_mr);  // choose a suitable font
  u8g2.drawStr(40,25,"Medir");  // write something to the internal memory
  u8g2.drawStr(40,45,"Ajustes");
  u8g2.drawStr(MENU_ARROW_X_POS,arrowPosition,">");
  u8g2.setDrawColor(0);
  if (arrowPosition == MEASURE_CHOICE) 
  {
      u8g2.drawBox(MENU_ARROW_X_POS, SETTINGS_CHOICE - 9, 8, 9);
  }
  else 
  {
      u8g2.drawBox(MENU_ARROW_X_POS, MEASURE_CHOICE - 9, 8, 9);
  }
  u8g2.sendBuffer();          // transfer internal memory to the display 
}

void printMeasureScreen(int arrowPosition, double co, double co2, String pm2_5) 
{
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_8x13_mr);  // choose a suitable font
  u8g2.drawStr(MEASUREMENT_X_POS, CO_Y_POS, " CO:");
  u8g2.drawStr(MEASUREMENT_X_POS, CO2_Y_POS, "CO2:");
  u8g2.drawStr(MEASUREMENT_X_POS, PM_Y_POS, " PM:");
  u8g2.drawStr(MEASUREMENT_X_POS+CO_NUM_OFFSET, CO_Y_POS, String(co, 0).c_str());
  u8g2.drawStr(MEASUREMENT_X_POS+CO2_NUM_OFFSET, CO2_Y_POS, String(co2, 0).c_str());
  u8g2.drawStr(MEASUREMENT_X_POS+PM_NUM_OFFSET, PM_Y_POS, pm2_5.c_str());
  u8g2.setFont(u8g2_font_tinytim_tr);
  u8g2.drawStr(MEASUREMENT_X_POS+CO_PPM_OFFSET, CO_Y_POS, "ppm");
  u8g2.drawStr(MEASUREMENT_X_POS+CO2_PPM_OFFSET, CO2_Y_POS, "ppm");
  u8g2.drawStr(MEASUREMENT_X_POS+PM_PPM_OFFSET, PM_Y_POS, "ppm");
  u8g2.drawStr(25,MEASURE_ARROW_Y_POS,"Salir?");  // write something to the internal memory
  u8g2.drawStr(STOP_MEASURE_CHOICE,MEASURE_ARROW_Y_POS,"Si");  // write something to the internal memory
  u8g2.drawStr(DONT_STOP_MEASURE_CHOICE,MEASURE_ARROW_Y_POS,"No");  // write something to the internal memory
  u8g2.drawStr(arrowPosition-6,MEASURE_ARROW_Y_POS,">");
  u8g2.setDrawColor(0);
  if (arrowPosition == DONT_STOP_MEASURE_CHOICE) 
  {
      u8g2.drawBox(STOP_MEASURE_CHOICE - 7, MEASURE_ARROW_Y_POS - 6, 6, 6);
  }
  else 
  {
      u8g2.drawBox(DONT_STOP_MEASURE_CHOICE - 7, MEASURE_ARROW_Y_POS - 6, 6, 6);
  }
  u8g2.sendBuffer();          // transfer internal memory to the display
}

void printOptions(int arrowPosition)
{
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_8x13_mr);  // choose a suitable font
  u8g2.drawStr(35, 10, "Ajustes"); // write something to the internal memory

  u8g2.setFont(u8g2_font_tinytim_tr);
  u8g2.drawStr(10, 20, "Tiempo entre Medidas");
  u8g2.drawStr(20, 30, "10sec  30sec  1min");
  u8g2.drawStr(25,MEASURE_ARROW_Y_POS,"Salir?");  // write something to the internal memory
  u8g2.drawStr(STOP_MEASURE_CHOICE,MEASURE_ARROW_Y_POS,"Si");  // write something to the internal memory
  u8g2.drawStr(DONT_STOP_MEASURE_CHOICE,MEASURE_ARROW_Y_POS,"No");  // write something to the internal memory
  if (arrowPosition == TEN_SEC) 
  {
      u8g2.drawStr(arrowPosition-6, 30,">");
      u8g2.setDrawColor(0);
      u8g2.drawBox(THIRTY_SEC-7, RECORD_TIME_OPTION_Y_POS-6, 6, 6);
      u8g2.drawBox(ONE_MIN-7, RECORD_TIME_OPTION_Y_POS-6, 6, 6);
      u8g2.drawBox(STOP_MEASURE_CHOICE - 7, MEASURE_ARROW_Y_POS - 6, 6, 6);
      u8g2.drawBox(DONT_STOP_MEASURE_CHOICE - 7, MEASURE_ARROW_Y_POS - 6, 6, 6);
  }
  else if (arrowPosition == THIRTY_SEC)
  {
      u8g2.drawStr(arrowPosition-6, 30,">");
      u8g2.setDrawColor(0);
      u8g2.drawBox(TEN_SEC-7, RECORD_TIME_OPTION_Y_POS-6, 6, 6);
      u8g2.drawBox(ONE_MIN-7, RECORD_TIME_OPTION_Y_POS-6, 6, 6);
      u8g2.drawBox(STOP_MEASURE_CHOICE - 7, MEASURE_ARROW_Y_POS - 6, 6, 6);
      u8g2.drawBox(DONT_STOP_MEASURE_CHOICE - 7, MEASURE_ARROW_Y_POS - 6, 6, 6);
  }
  else if (arrowPosition == ONE_MIN)
  {
      u8g2.drawStr(arrowPosition-6, 30,">");
      u8g2.setDrawColor(0);
      u8g2.drawBox(TEN_SEC-7, RECORD_TIME_OPTION_Y_POS-6, 6, 6);
      u8g2.drawBox(THIRTY_SEC-7, RECORD_TIME_OPTION_Y_POS-6, 6, 6);
      u8g2.drawBox(STOP_MEASURE_CHOICE - 7, MEASURE_ARROW_Y_POS - 6, 6, 6);
      u8g2.drawBox(DONT_STOP_MEASURE_CHOICE - 7, MEASURE_ARROW_Y_POS - 6, 6, 6);
  }
  else if (arrowPosition == DONT_STOP_MEASURE_CHOICE) 
  {
      u8g2.drawStr(arrowPosition-6, MEASURE_ARROW_Y_POS,">");
      u8g2.setDrawColor(0);
      u8g2.drawBox(TEN_SEC-7, RECORD_TIME_OPTION_Y_POS-6, 6, 6);
      u8g2.drawBox(THIRTY_SEC-7, RECORD_TIME_OPTION_Y_POS-6, 6, 6);
      u8g2.drawBox(ONE_MIN-7, RECORD_TIME_OPTION_Y_POS-6, 6, 6);
      u8g2.drawBox(STOP_MEASURE_CHOICE - 7, MEASURE_ARROW_Y_POS - 6, 6, 6);
  }
  else 
  {
      u8g2.drawStr(arrowPosition-6, MEASURE_ARROW_Y_POS,">");
      u8g2.setDrawColor(0);
      u8g2.drawBox(TEN_SEC-7, RECORD_TIME_OPTION_Y_POS-6, 6, 6);
      u8g2.drawBox(THIRTY_SEC-7, RECORD_TIME_OPTION_Y_POS-6, 6, 6);
      u8g2.drawBox(ONE_MIN-7, RECORD_TIME_OPTION_Y_POS-6, 6, 6);
      u8g2.drawBox(DONT_STOP_MEASURE_CHOICE - 7, MEASURE_ARROW_Y_POS - 6, 6, 6);
  }
  u8g2.sendBuffer();          // transfer internal memory to the display
}
void printError()
{
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_8x13_mr);  // choose a suitable font
  u8g2.drawStr(5, 15, "Por favor");
  u8g2.drawStr(5, 30, "inserte una ");
  u8g2.drawStr(5, 45, "tarjeta SD"); // write something to the internal memory

  u8g2.sendBuffer();          // transfer internal memory to the display
}
