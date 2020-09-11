#include "xmodem.h"

static uint16_t crc_update(uint16_t crc_in, int incr)
{
	uint16_t xor1 = crc_in >> 15;
	uint16_t out = crc_in << 1;

	if (incr) out++;

	if (xor1) out ^= CRC_POLY;

	return out;
}

static uint16_t crc16(const uint8_t *data, uint16_t size)
{
	uint16_t crc, i;

	for (crc = 0; size > 0; size--, data++)
		for (i = 0x80; i; i >>= 1)
			crc = crc_update(crc, *data & i);

	for (i = 0; i < 16; i++)
		crc = crc_update(crc, 0);

	return crc;
}
static uint16_t swap16(uint16_t in)
{
	return (in >> 8) | ((in & 0xff) << 8);
}

//int xmodem_transmit(Serial *serial, const char *filename)
//Serial is Global;

//int xmodem_transmit()
int xmodem_transmit(HardwareSerial *serial, const char *filename,uint8_t resetPin)
{
     
	log_i("update FANET-Module %s %u",filename,resetPin);

	File file = SPIFFS.open(filename);
	if (!file){
		log_e("file not found %s",filename);
		return -1;
	}

	size_t len = file.size();
	log_i("len: %d", len);

    pinMode(resetPin, OUTPUT);
    digitalWrite(resetPin, LOW);
    delay(500);
    digitalWrite(resetPin, HIGH);
	delay(10);
	serial->flush();
	while(serial->available()) serial->read(); //empty receive-buffer
	//delay(500);
	//log_i("%s",serial->readString().c_str());

	//serial->write("#DGJ BLxld\n");
	//delay(100);
	//log_i("%s",serial->readString().c_str());
	//serial->flush();

	int ret;
	uint8_t answer;

	uint8_t eof = X_EOF;
	struct xmodem_chunk chunk;

	log_i("Sending Data ...");

	chunk.block = 1;
	chunk.start = X_STX;

	int sent_blocks=0;
	int failures = 0;

	serial->flush();
	//serial->readString();

    noInterrupts();

	bool read_file=true;

	while (len)
	{
		size_t z = 0;
		int next = 0;

		z = min(len, sizeof(chunk.payload));
		if (read_file)
		{
		  //log_i("read file len=%d",z);
		  file.read(chunk.payload, z);
		}

		memset(chunk.payload + z, 0xff, sizeof(chunk.payload) - z);

		chunk.crc = swap16(crc16(chunk.payload, sizeof(chunk.payload)));
		chunk.block_neg = 0xff - chunk.block;

		while(serial->available()) serial->read(); //empty receive-buffer
		ret = serial->write((uint8_t*) &chunk, sizeof(chunk));
		serial->flush();

		if (ret != sizeof(chunk))
		{
		    log_i("Invalid chunk size written");
			file.close();
			return -5;
		}


		//delay(10);
		size_t slen=0;
		while (!serial->available()){
			delay(1); //wait until something received
		}
		slen=serial->available();

		answer=0;

		if (slen!=0)
		{
		   answer=serial->read();
		}

		switch (answer)
		{

            case X_C:
               failures++;
               log_i("X_C");
               delay(100);
               read_file=false;
               break;
			case X_NAK:
				failures++;
				log_i("X_NAK");
				delay(100);
	            read_file=false;
				break;
			case X_ACK:
				next = 1;
				sent_blocks++;
				log_i("X_ACK");
				delay(100);
				read_file=true;
				break;
			default:
				failures++;
				log_i(">%d:%c<",answer,(char)answer);
				/*
				Serial.print(">");
			    Serial.print(answer);
				Serial.print(":");
				Serial.print((char)answer);
				Serial.print("<");
				*/
				delay(100);
				read_file=false;
				break;
		}

		if (next)
		{
			chunk.block++;
			len -= z;
		}

		if(failures > 200)
			break;

	}

	ret = serial->write(&eof, sizeof(eof));
    if (ret != sizeof(eof))
    {
        log_e(" Failure sending EOF ");
        return -7;
    }

    file.close();

    if (failures>200)
    {
        log_e( "Xmodem FAILED. Failure %d. Blocks %d", failures, sent_blocks);
    }
    else
    {
	    log_i( "Xmodem done. Failure %d. Blocks %d", failures, sent_blocks);
    }

	return failures>100?-2:0;
}
