#define setHigh(port, pin) ((port) |= (1 << (pin)))
void init() {
	ADMUX  = 0;//(1<<REFS0);							// external reference voltage
	ADCSRA  = (1<<ADEN);   							// enable ADC
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) |(1<<ADPS0);	// prescaler = highest division
//     ADCSRB |= (1<<BIN); //bipolar mode
}


// channel 8 can be used to measure the temperature of the chip
void connectChannel(uint8_t number) {
	ADMUX &= (11110000);
	ADMUX |= number;
	//DIDR0  =  (1<<number);
}

void startConversion() {
	setHigh(ADCSRA,ADSC);
}

bool isConversionFinished() {
	 return (ADCSRA & (1<<ADIF));
}

bool isConversionRunning() {
	return !(ADCSRA & (1<<ADIF));
}

uint16_t getConversionResult() {
	uint16_t result = ADCL;
	return result | (ADCH<<8);
}


