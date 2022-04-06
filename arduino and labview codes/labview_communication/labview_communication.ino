char CHAR_from_labv;
String STRING_from_labv;
String analog1;
int serialBLUEvalue;
int BLUE=3;
int GREEN=10;
int readA1;

void setup() 
{
  Serial.begin(9600);
  pinMode(BLUE,OUTPUT);
  pinMode(GREEN,OUTPUT);
}

void loop() 
{
  if (Serial.available()) 
  {
   CHAR_from_labv=Serial.read();
    STRING_from_labv += CHAR_from_labv;
    
    if(CHAR_from_labv=='\n')
    {
     if (STRING_from_labv[0]=='a')
      {digitalWrite(GREEN,HIGH);}
      else if (STRING_from_labv[0]=='x') {
        digitalWrite(GREEN,LOW);
        }

      for (int i=0;i<3;i++){
      analog1 += STRING_from_labv[i+1];
     }
      
      serialBLUEvalue=analog1.toInt();
      analogWrite(BLUE,serialBLUEvalue);
 
      analog1="";
      STRING_from_labv="";
      
      readA1 =analogRead(A1);      
      Serial.print(readA1);
      Serial.print('\t');
      Serial.println();
      delay(50);
     } 
  }
}
