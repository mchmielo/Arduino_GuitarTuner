#include <TimerOne.h>
#include <LiquidCrystal.h>
//#include <util/delay.h>
#include "pitches.h"

#define CONST 3.322038403
#define TRIGGER_LEVEL 100


LiquidCrystal lcd(12, 11, 8, 7, 6, 5);

enum options{
	tuner,
	metronome,
	sound,
	menu
};

enum pins{
	leftButton,
	menuButton,
	rightButton
};

uint8_t clippingLED = A1;
uint8_t lowSignalLED = A5;

/*********************** ZMIENNE MENU ***************/

uint8_t timeToExpireLEFT = 0;		// czas pomiedzy zareagowaniem na przycisniecie przycisku w lewo
uint8_t timeToExpireMENU = 0;		// czas pomiedzy zareagowaniem na przycisniecie przycisku menu
uint8_t timeToExpireRIGHT = 0;		// czas pomiedzy zareagowaniem na przycisniecie przycisku w prawo
pins currPin = leftButton;			// zmienna okresla ktory kolejny przycisk bedzie multipleksowany
byte timeToHideMenu = 0;			// czas, po ktorym ukryte zostanie menu
uint8_t menuScreen = 0;			// numer obecnego ekranu menu
boolean menuChangeUpper = true;	// czy gorna linia wyswietlacza ma zostac zaktualizowana
boolean menuChangeLower = true;	// czy dolna lina ma zostac zaktualizowana

/*********************** ZMIENNE METRONOMU ***************/

int loudSpeaker = 9;						// pin glosnika
options prevOption = tuner;	// poprzednia wybrana opcja 
options currOption = tuner;	// terazniejsza opcja - od niej zalezy ktory ekran zostanie wlaczyony
										    // oraz  jaka akcja zostanie wykonana
int metroTempo = 120;				// tempo metronomu w BPM
int duration;								// ile milisekund ma brzmiec sygnal metronomu

/*************** ZMIENNE STROJENIA DO DZWIEKU ***************/

int currNote = NOTE_A4;			// wybrany dzwiek w trybie strojenia do dzwieku

/************** ZMIENNE POTRZEBNE DO STROIKA ****************/

//tablica przechowuje nazwy wszystkich poltonow
char notesNames[12][3] = { { "C " }, { "C#" }, { "D " }, { "D#" }, { "E " }, { "F " },
{ "F#" }, { "G " }, { "G#" }, { "A " }, { "A#" }, { "H " } };

int cents = 0;				// centy - odleglosc od najblizszego poltonu
int trend = 1;				// w dol lub w gore od dzwieku A = 440 Hz

int note = 0;
boolean clipping = 0;		// zmienna okresla czy sygnal jest przesterowany
boolean lowSignal = 0;
int freqTol = 50;			// tolerancja czestotliwosci
byte counter = 0;			
int meanAmp = 0;			// srednia amplituda
char noteBar[16];			

byte newData = 0;			// obecny odczyt ADC
byte prevData = 0;			// poprzedni odczyt z ADC
unsigned int time = 0;		// licznik czasu pomiedzy zboczami, wartosc kopiowana jest do timer[]
int timer[10];				// tablica przechowujaca kolejne momenty czasowe
int slope[10];				// tablica przechowujaca kolejne zbocza sygnalu
unsigned int totalTimer;	// potrzebne do obliczenia okresu
unsigned int period;		// zmienna przechowujaca okres sygnalu
byte index = 0;				// obecny indeks w tablicy
volatile float frequency = 0;			// terazniejszy odczyt czestotliwosci
volatile float prevFreq = 0;			// poprzedni odczyt czestotliwosci
int maxSlope = 0;			// maksymalna chwilowa wartosc sygnalu wejsciowego
int newSlope;				// przechowywanie nadchodzacych zboczy sygnalu

/**************ZMIENNE DO WYKRYWANIA ZGODNYCH SYGNALOW *********/
							
byte noMatch = 0;			// zliczanie ile niepodobnych sygnalow nastapilo po sobie, jesli za duzo wtedy reset
byte slopeTol = 1;			// tolerancja zbocza	-	mozna zmieniac aby uzyskac lepsze wykrywanie sygnalu
int timerTol = 2;			// tolerancja czasu do wykrywania podobnych sygnalow - mozna zmieniac aby uzyskac lepsze wykrywanie sygnalu

/************** ZMIENNE DO WYKRYWANIA AMPLITUDY ****************/
unsigned int ampTimer = 0;
byte maxAmp = 0;
byte checkMaxAmp;
byte ampThreshold = 30;				// zwiekszyc w przypadku duzych szumow


void setup(){
	


  lcd.begin(16, 2);
  Serial.begin(9600);
  pinMode(2, INPUT_PULLUP);    // przyciski podlaczone do przerwan zewnetrznych
  pinMode(3, INPUT_PULLUP);
  pinMode(A2, OUTPUT);        // 3 multipleksowane przyciski
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  digitalWrite(A2, LOW);      // stan aktywny LOW
  digitalWrite(A3, HIGH);
  digitalWrite(A4, HIGH);

  pinMode(clippingLED, OUTPUT);
  pinMode(lowSignalLED, OUTPUT);
  
  cli();//stop interrupts

  /******************* USTAWIENIE TIMER0 - MULTIPLEKSOWANIE PRZYCISKOW *********/
  Timer1.initialize(100000);
  Timer1.attachInterrupt(buttonsInterrupt);

  /***** PRZERWANIE ZEWNETRZNE GDY PRZYCISNIETY KTORYKOLWIEK PRZYCISK **********/
  attachInterrupt(0, buttonPressed, FALLING);
  
  /********************* USTAWIENIE PROBKOWANIA SYGNALU ANALOGOWEGO ***********/
  //set up continuous sampling of analog pin 0 at 38.5kHz

  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;

  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only

  ADCSRA |= (1 << ADPS2) | (1 << ADPS1); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements
  
  /*
  //Set Initial Timer value
  TCNT0 = 0;
  //Place TOP timer value to Output compare register
  OCR0A = 99;
  //Set CTC mode
  //and make toggle PD6/OC0A pin on compare match
  TCCR0A |= (1 << WGM01);

  // Select Vref=AVcc
  //and set left adjust result
  ADMUX |= (1 << REFS0) | (1 << ADLAR);
  //set prescaller to 32
  //enable autotriggering
  //enable ADC interupt
  //and enable ADC
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0) | (1 << ADATE) | (1 << ADIE) | (1 << ADEN);
  //set ADC trigger source - Timer0 compare match A
  ADCSRB |= (1 << ADTS1) | (1 << ADTS0);

  //select ADC channel with safety mask
  ADMUX = (ADMUX & 0xF0) | (A0 & 0x0F);*/

  sei();//allow interrupts
}
void buttonsInterrupt(){
	if (timeToExpireLEFT > 0)
		timeToExpireLEFT--;
	if (timeToExpireMENU > 0)
		timeToExpireMENU--;
	if (timeToExpireRIGHT > 0)
		timeToExpireRIGHT--;
	switch (currPin){
	case leftButton:
		digitalWrite(A2, LOW);
		digitalWrite(A3, HIGH);
		digitalWrite(A4, HIGH);
		currPin = menuButton;
		break;
	case menuButton:
		digitalWrite(A2, HIGH);
		digitalWrite(A3, LOW);
		digitalWrite(A4, HIGH);
		currPin = rightButton;
		break;
	case rightButton:
		digitalWrite(A2, HIGH);
		digitalWrite(A3, HIGH);
		digitalWrite(A4, LOW);
		currPin = leftButton;
		break;
	}
}
ISR(ADC_vect) {//when new ADC value ready
	prevData = newData;//store previous value
	newData = ADCH;//get value from A0
	meanAmp = (meanAmp + newData) / 2;
	if (prevData < TRIGGER_LEVEL && newData >= TRIGGER_LEVEL){//if increasing and crossing midpoint
		newSlope = newData - prevData;//calculate slope
		if (abs(newSlope - maxSlope)<slopeTol){//if slopes are ==
			//record new data and reset time
			slope[index] = newSlope;
			timer[index] = time;
			time = 0;
			if (index == 0){//new max slope just reset
				PORTB |= B00010000;//set pin 12 high
				noMatch = 0;
				index++;//increment index
			}
			else if (abs(timer[0] - timer[index])<timerTol && abs(slope[0] - newSlope)<slopeTol){//if timer duration and slopes match
				//sum timer values
				totalTimer = 0;
				for (byte i = 0; i<index; i++){
					totalTimer += timer[i];
				}
				period = totalTimer;//set period
				//reset new zero index values to compare with
				timer[0] = timer[index];
				slope[0] = slope[index];
				index = 1;//set index to 1
				//PORTB |= B00010000;//set pin 12 high
				noMatch = 0;
			}
			else{//crossing midpoint but not match
				index++;//increment index
				if (index > 9){
					reset();
				}
			}
		}
		else if (newSlope>maxSlope){//if new slope is much larger than max slope
			maxSlope = newSlope;
			time = 0;//reset clock
			noMatch = 0;
			index = 0;//reset index
		}
		else{//slope not steep enough
			noMatch++;//increment no match counter
			if (noMatch>9){
				reset();
			}
		}
	}
	if (newData < 1023 && newData > 5){		// pomiar bez przesterowania
		clipping = 0;
		lowSignal = 0;
	}
	else{
		if (newData == 1023)
			clipping = 1;						// currently clipping
		else
			lowSignal = 1;
	}

	time++;//increment timer at rate of 38.5kHz

	ampTimer++;//increment amplitude timer
	if (abs(TRIGGER_LEVEL - ADCH)>maxAmp){
		maxAmp = abs(TRIGGER_LEVEL - ADCH);
	}
	if (ampTimer >= 1000){
		ampTimer = 0;
		checkMaxAmp = maxAmp;
		maxAmp = 0;
	}
}

void reset(){				// reset zmiennych tunera
	index = 0;
	noMatch = 0;
	maxSlope = 0;
}


void checkClipping(){		// 
	if (clipping){			// if currently clipping
		digitalWrite(clippingLED, HIGH);	// wylacz powiadomienie o przesterowaniu
	}
	else digitalWrite(clippingLED, LOW);
	if (lowSignal){
		digitalWrite(lowSignalLED, HIGH);
	}
	else digitalWrite(lowSignalLED, LOW);
}

void findNote(float freq){
	int halfTones, mCents, centsSign = 1;
	if (freq > 440.0){
		cents = static_cast<int>(1200 * CONST*log10(freq / 440.0));
		trend = 1;
	}
	else{
		cents = static_cast<int>(1200 * CONST*log10(440.0 / freq));
		trend = -1;
	}
	mCents = cents % 100;
	halfTones = static_cast<int>(static_cast<double>(cents) / 100.0);
	if (trend < 0){
		if (mCents < 50)
			centsSign = -1;
		else{
			centsSign = 1;
			mCents = 100 - mCents;
			halfTones++;
		}
		note = ((9 + (static_cast<int>(halfTones / 12)) * 12 - halfTones)) % 12;
	}
	else{
		if (mCents < 50)
			centsSign = 1;
		else{
			centsSign = -1;
			mCents = 100 - mCents;
			halfTones++;
		}
		note = (9 + (trend*halfTones)) % 12;
	}
	mCents = centsSign*mCents;
	cents = mCents;

	if (note < 0){
		note = -note;
	}
}

void evalNoteBar(){
	int pos = static_cast<int>((cents + 100)*1.4 / 20);
	for (int i = 0; i < 16; ++i){
		noteBar[i] = ' ';
	}
	noteBar[pos] = notesNames[note][0];
	if (notesNames[note][1] == '#'){
		noteBar[pos + 1] = '#';
	}
	if ((pos - 9) >= 0){
		int pos2 = (note + 12 - 1) % 12;
		noteBar[pos - 9] = notesNames[pos2][0];
		if (notesNames[pos2][1] == '#'){
			noteBar[pos - 8] = '#';
		}
	}
	else if ((pos + 8) <= 14){
		int pos2 = (note + 1) % 12;
		noteBar[pos + 8] = notesNames[pos2][0];
		if (notesNames[pos2][1] == '#'){
			noteBar[pos + 9] = '#';
		}
	}
	for (int i = 0; i < 16; ++i){
		if (noteBar[i] != ' ' && !(noteBar[i] >= 65 && noteBar[i] < 73) && noteBar[i] != '#'){
			noteBar[i] = ' ';
		}
	}
}

void buttonPressed(){
	if (currOption == menu)
		timeToHideMenu = 40;
	if (digitalRead(A2) == LOW){
		if (!timeToExpireLEFT){			// gdy nie bylo wcisnietego wczesniej przycisku
			timeToExpireLEFT = 3;		// 300ms
			leftPressed();
		}
	}
	else
	if (digitalRead(A3) == LOW){
		if (!timeToExpireMENU){			// gdy nie bylo wcisnietego wczesniej przycisku
			timeToExpireMENU = 3;		// 300ms
			menuPressed();
		}
	}
	else if (digitalRead(A4) == LOW){
		if (!timeToExpireRIGHT){			// gdy nie bylo wcisnietego wczesniej przycisku
			timeToExpireRIGHT = 3;		// 300ms
			rightPressed();
		}
	}
}
/****** PRZYCISK MENU *******/

void menuPressed(){
	
	switch (currOption){
	case menu:        // gdy juz sie jest w menu - drugie wcisniecie menu to potwierdzenie
		prevOption = currOption;
		currOption = static_cast<options>(menuScreen);
		timeToHideMenu = 0;
		break;
	default:          // wejscie do menu z kazdego innego ekranu
		timeToHideMenu = 40;
		prevOption = currOption;
		currOption = menu;
		menuScreen = static_cast<uint8_t>(prevOption);
		break;
	}
	menuChangeUpper = true;
	menuChangeLower = true;
}
/****** PRZYCISK W LEWO *******/
void leftPressed(){
	
	menuChangeLower = true;
	switch (currOption){
	case tuner:
		menuChangeLower = false;
		break;
	case metronome:
		if (metroTempo >= 40)
			metroTempo--;
		break;
	case sound:
		if (currNote > 0)
			currNote--;
		break;
	case menu:
		if (menuScreen > 0){
			menuScreen--;
			menuChangeUpper = true;
		}
		break;
	}
}
/****** PRZYCISK W PRAWO *******/
void rightPressed(){
	menuChangeLower = true;
	switch (currOption){
	case tuner:
		menuChangeLower = false;
		break;
	case metronome:
		if (metroTempo <= 250)
			metroTempo++;
		break;
	case sound:
		if (currNote < 88)
			currNote++;
		break;
	case menu:
		if (menuScreen < 2){
			menuChangeUpper = true;
			menuScreen++;
		}
		break;
	}
}

void playSound(){
	if (menuChangeUpper){
		lcd.setCursor(0, 0);
		lcd.print("Wybrany dzwiek :");
	}
	if (menuChangeLower){
		findNote(notes[currNote]);
		lcd.setCursor(0, 1);
		lcd.print("   ");
		lcd.print(notesNames[note][0]);
		lcd.print(notesNames[note][1]);
		lcd.print(", ");
		lcd.print(notes[currNote]);
		lcd.print("Hz   ");
	}
	menuChangeUpper = false;
	menuChangeLower = false;
	tone(loudSpeaker, notes[currNote]);
}


void metronom(){
	int duration = static_cast<int>((static_cast<double>(60.0 / metroTempo)* 1000.0) / 8.0);
	if (menuChangeUpper){
		lcd.setCursor(0,0);
		lcd.print("    Tempo:      ");
	}
	if (menuChangeLower){
		lcd.setCursor(0, 1);
		lcd.print("      ");
		lcd.print(metroTempo);
                if(metroTempo <= 99)
		    lcd.print("        ");
                else lcd.print("       ");
	}
	menuChangeUpper = false;
	menuChangeLower = false;
	Serial.println(duration);

	tone(loudSpeaker, 98, duration);
	delay(duration * 8);
	noTone(loudSpeaker);

}

void showMenu(){
	timeToHideMenu--;
		
	if (timeToHideMenu == 0){
		currOption = prevOption;        // przywrocenie poprzedniego widoku po chwili nieaktywnosci
		menuChangeUpper = true;
		menuChangeLower = true;
	}
	else{
		
		switch (menuScreen){
		case tuner:
			if (menuChangeUpper){
				lcd.setCursor(0, 0);
				lcd.print("  Strojenie   ->");
			}
			if (menuChangeLower){
				lcd.setCursor(0, 1);
				lcd.print("                ");
			}
			break;
		case metronome:
			if (menuChangeUpper){
				lcd.setCursor(0, 0);
				lcd.print("<-  Metronom  ->");
			}
			if (menuChangeLower){
				lcd.setCursor(0, 1);
				lcd.print("                ");
			}
			break;
		case sound:
			if (menuChangeUpper){
				lcd.setCursor(0, 0);
				lcd.print("<-  Strojenie   ");
				lcd.setCursor(0, 1);
				lcd.print("   do dzwieku   ");
			}
			break;
		}
		menuChangeUpper = false;
		menuChangeLower = false;
	}
}

void tune(){
	checkClipping();
	if (checkMaxAmp>ampThreshold){
		prevFreq = frequency;
		frequency = (38462/2) / float(period);//calculate frequency timer rate/period
		if (abs(prevFreq - frequency) <= freqTol){
			counter++;
		}
		else
			counter = 0;
		Serial.print("Newdata= ");
		Serial.print(newData);
		Serial.print(", period= ");
		Serial.print(period);
		Serial.print(", index= ");
		Serial.print(index);
		Serial.print(", counter=");
		Serial.println(counter);
		if (counter >= 2){
			period = 0;
			counter = 0;
			note = 0;
			cents = 0;
			findNote(frequency);
			//print results
			Serial.print(frequency);
			Serial.println(" hz");
			Serial.print(notesNames[note][0]);
			Serial.print(notesNames[note][1]);
			if (cents > 0)
				Serial.print(" +");
			Serial.println(cents);

			lcd.setCursor(0, 0);
			int amp = meanAmp / 16;
			evalNoteBar();
			lcd.print(noteBar);
			lcd.setCursor(0, 1);
			lcd.print(notesNames[note][0]);
			if (notesNames[note][1] == '#')
				lcd.print("#");
			else lcd.print(" ");
			if (cents > 0)
				lcd.print(" +");
			else
				lcd.print(" ");
			lcd.print(cents);
			lcd.print("          ");
		}
	}
	else{
		Serial.print("newData= ");
		Serial.println(newData);
		lcd.setCursor(0, 0);
		lcd.print("   MEGA stroik  ");
		lcd.setCursor(0, 1);
		lcd.print("                ");
		Serial.println("MEGA stroik");
	}
}

void loop(){
/*	if (currOption != tuner){
		ADCSRA &= ~(1 << ADEN);
	}
	else{
        ADCSRA |= (1 << ADEN);
		ADCSRA |= (1 << ADSC); //start ADC measurements
	}*/
	if (currOption != sound)
		noTone(loudSpeaker);
	switch (currOption){
	case tuner:
		tune();
		break;
	case metronome:
		metronom();
		break;
	case sound:
		playSound();
		break;
	case menu:
		showMenu();
		break;
	}
	if (currOption != metronome)
		delay(200);
	//lcd.setCursor(0, 1);
	//lcd.print(prevFreq);
	//lcd.setCursor(5, 1);
	//lcd.print(frequency);
	//lcd.setCursor(12, 1);
	//lcd.print(period);
}
