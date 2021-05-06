#define iButtonPin A3
#define VRpinGnd 5
#define ACpin 6
//OneWire ibutton (iButtonPin);
byte addr[8]; // временный буфер
byte keyID[8]; // ID ключа для записи
byte halfT;

void setup() {
  Serial.begin(9600);
  pinMode(ACpin, INPUT);
}

void loop() {
  searchCyfral();
}

unsigned long pulseACompA(bool pulse, byte Average = 80, unsigned long timeOut = 1500){  // pulse HIGH or LOW
  bool AcompState;
  unsigned long tEnd = micros() + timeOut;
  do {
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1 << ADSC)); // Wait until the ADSC bit has been cleared
    if (ADCH > 200) return 0;
    if (ADCH > Average) AcompState = HIGH;  // читаем флаг компаратора
      else AcompState = LOW;
    if (AcompState == pulse) {
      tEnd = micros() + timeOut;
      do {
          ADCSRA |= (1<<ADSC);
          while(ADCSRA & (1 << ADSC)); // Wait until the ADSC bit has been cleared
        if (ADCH > Average) AcompState = HIGH;  // читаем флаг компаратора
          else AcompState = LOW;
        if (AcompState != pulse) return (unsigned long)(micros() + timeOut - tEnd);  
      } while (micros() < tEnd);
      return 0;                                                 //таймаут, импульс не вернуся оратно
    }             // end if
  } while (micros() < tEnd);
  return 0;
}

void ADCsetOn(){
  ADMUX = (ADMUX&0b11110000) | 0b0011 | (1<<ADLAR);// (1 << REFS0);          // подключаем к AC Линию A3 ,  левое выравние, измерение до Vcc
  ADCSRB = (ADCSRB & 0b11111000) | (1<<ACME);                // источник перезапуска ADC FreeRun, включаем мультиплексор AC
  ADCSRA = (ADCSRA & 0b11111000) |0b011 | (1<<ADEN) | (1<<ADSC);// | (1<<ADATE);      // 0b011 делитель скорости ADC, // включаем ADC и запускаем ADC и autotriger ADC 
}

void ACsetOn(){
  ACSR |= 1<<ACBG;                            // Подключаем ко входу Ain0 1.1V для Cyfral/Metacom
  ADCSRA &= ~(1<<ADEN);                       // выключаем ADC
  ADMUX = (ADMUX&0b11110000) | 0b0011;        // подключаем к AC Линию A3
  ADCSRB |= 1<<ACME;                          // включаем мультиплексор AC
}

bool read_cyfral(byte* buf, byte CyfralPin){
  unsigned long ti; byte i=0, j = 0, k = 0;
  analogRead(iButtonPin);
  ADCsetOn(); 
  byte aver = calcAverage();
  unsigned long tEnd = millis() + 30;
  do{
    ti = pulseACompA(HIGH, aver);
    if ((ti == 0) || (ti > 260) || (ti < 10)) {i = 0; j=0; k = 0; continue;}
    if ((i < 3) && (ti > halfT)) {i = 0; j = 0; k = 0; continue;}      //контроль стартовой последовательности 0b0001
    if ((i == 3) && (ti < halfT)) continue;      
    if (ti > halfT) bitSet(buf[i >> 3], 7-j);
      else if (i > 3) k++; 
    if ((i > 3) && ((i-3)%4 == 0) ){        //начиная с 4-го бита проверяем количество нулей каждой строки из 4-и бит
      if (k != 1) {for (byte n = 0; n < (i >> 3)+2; n++) buf[n] = 0; i = 0; j = 0; k = 0; continue;}        //если нулей больше одной - начинаем сначала 
      k = 0; 
    }
    j++; if (j>7) j=0;
    i++;
  } while ((millis() < tEnd) && (i < 36));
  if (i < 36) return false;
  return true;
}

bool searchCyfral(){
  byte buf[8];
  for (byte i = 0; i < 8; i++) {addr[i] =0; buf[i] = 0;}
  if (!read_cyfral(addr, iButtonPin)) return false;
  if (!read_cyfral(buf, iButtonPin)) return false;
  for (byte i = 0; i < 8; i++) 
    if (addr[i] != buf[i]) return false;

  for (byte i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX); Serial.print(":");
    keyID[i] = addr[i];                                         // копируем прочтенный код в ReadID
  }
  Serial.println(F(" Type: Cyfral "));
  return true;  
}

byte calcAverage(){
  unsigned int sum = 127; byte preADCH = 0, j = 0; 
  for (byte i = 0; i<255; i++) {
    ADCSRA |= (1<<ADSC);
    delayMicroseconds(10);
    while(ADCSRA & (1 << ADSC)); // Wait until the ADSC bit has been cleared
    sum += ADCH;
  }
  sum = sum >> 8;
  unsigned long tSt = micros();
  for (byte i = 0; i<255; i++) {
    delayMicroseconds(4);
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1 << ADSC)); // Wait until the ADSC bit has been cleared
    if (((ADCH > sum)&&(preADCH < sum)) | ((ADCH < sum)&&(preADCH > sum))) {
      j++;
      preADCH = ADCH;
    }   
  }
  halfT = (byte)((micros() - tSt) / j);
  return (byte)sum;
}
