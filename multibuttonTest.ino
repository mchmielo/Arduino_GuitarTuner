#include <LiquidCrystal.h>
#include <util/delay.h>
#include "pitches.h"

#define CONST 3.322038403

LiquidCrystal lcd(12, 11, 8, 7, 6, 5);

volatile boolean togglePin1 = false;
volatile int pin = 0;
volatile int count = 0;
volatile uint8_t timeToExpireLEFT = 0;
volatile uint8_t timeToExpireMENU = 0;
volatile uint8_t timeToExpireRIGHT = 0;

int loudSpeaker = 9;

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

//after this time menu will be hidden
volatile pins currPin = leftButton;
volatile byte timeToHideMenu = 0;
volatile uint8_t menuScreen = 0;
volatile boolean menuChangeUpper = true;
volatile boolean menuChangeLower = true;

volatile options prevOption = metronome;
volatile options currOption = metronome;

// metronome
volatile int metroTempo = 120;  //BPM

// sound
volatile int currNote = 46;
int duration;

char notesNames[12][3] = { { "C " }, { "C#" }, { "D " }, { "D#" }, { "E " }, { "F " },
{ "F#" }, { "G " }, { "G#" }, { "A " }, { "A#" }, { "H " } };
int cents = 0, trend = 1, note = 0;

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
  
  cli();//stop interrupts

  /******************* USTAWIENIE TIMER0 - MULTIPLEKSOWANIE PRZYCISKOW *********/
  //set timer0 interrupt at 2kHz
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0 = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 249;// = (16*10^6) / (250*256) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS02);
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  attachInterrupt(0, buttonPressed, FALLING);
    
  sei();//allow interrupts
}

ISR(TIMER0_COMPA_vect){
	if (timeToExpireLEFT > 0)
		timeToExpireLEFT--;
	if (timeToExpireMENU > 0)
		timeToExpireMENU--;
	if (timeToExpireRIGHT > 0)
		timeToExpireRIGHT--;
  switch(currPin){
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

void buttonPressed(){
	if (currOption == menu)
		timeToHideMenu = 40;
	if (digitalRead(A2) == LOW){
		if (!timeToExpireLEFT){			// gdy nie bylo wcisnietego wczesniej przycisku
			timeToExpireLEFT = 75;		// 300ms
			leftPressed();
		}
	}
	else
	if (digitalRead(A3) == LOW){
		if (!timeToExpireMENU){			// gdy nie bylo wcisnietego wczesniej przycisku
			timeToExpireMENU = 75;		// 300ms
			menuPressed();
		}
	}
	else if (digitalRead(A4) == LOW){
		if (!timeToExpireRIGHT){			// gdy nie bylo wcisnietego wczesniej przycisku
			timeToExpireRIGHT = 75;		// 300ms
			rightPressed();
		}
	}
	count = 0;
  
}



/******* OBSLUGA PRZYCISKOW  ****/
/*void buttonPressed(){
	static byte count = 0;
	if (count > 10){
		count = 0;
	}
	if (count == 0){
		if (digitalRead(A2) == LOW){
			leftPressed();
		}
		else
		if (digitalRead(A3) == LOW){
			menuPressed();
		}
		else if (digitalRead(A4) == LOW){
			rightPressed();
		}
	}
	count++;
}*/
/****** PRZYCISK MENU *******/

void menuPressed(){
	
	switch (currOption){
	case menu:        // gdy juz sie jest w menu - drugie wcisniecie menu to potwierdzenie
		prevOption = currOption;
		currOption = static_cast<options>(menuScreen);
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
		lcd.print("    ");
		lcd.print(metroTempo);
		lcd.print("    ");
	}
	menuChangeUpper = false;
	menuChangeLower = false;
	Serial.println(duration);

	tone(loudSpeaker, 98, duration);
	_delay_ms(duration * 8);
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
			if (menuChangeUpper)
				lcd.setCursor(0, 0);
				lcd.print("  Strojenie   ->");
			if (menuChangeLower){
				lcd.setCursor(0, 1);
				lcd.print("                ");
			}
			break;
		case metronome:
			if (menuChangeUpper)
				lcd.setCursor(0, 0);
				lcd.print("<-  Metronom  ->");
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

void loop(){


	switch (currOption){
	case tuner:
		//tune();
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
		_delay_ms(100);
	lcd.setCursor(10, 1);
	lcd.print(timeToExpireLEFT);
	lcd.setCursor(13, 1);
	lcd.print(timeToExpireRIGHT);
	/*
	duration = static_cast<int>((static_cast<double>(60.0 / metroTempo)* 1000.0)/8.0);

	tone(9, 98, duration);

	_delay_ms(duration*8);
	noTone(9);
	
	count++;
	lcd.setCursor(0,0);
	if (count > 3){
		lcd.print("Duration:");
		lcd.print(duration);
		lcd.setCursor(0, 1);
		lcd.print("Tempo:");
		lcd.print(metroTempo);
	}
	else{
		lcd.print("Pin = ");
		lcd.print(pin);
	}
	Serial.print("Count: ");
	Serial.println(count);*/
  
}
