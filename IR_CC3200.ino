
#include "IRRemotereceiver.h"

IRLibrary irlibrary;


void setup()
{ 
  Serial.begin(9600);
  irlibrary.init_IR();      
}

void loop()
{
irlibrary.monitor_IR();
}
