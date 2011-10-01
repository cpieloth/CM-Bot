/**
 * \file	testRingBuffer.c
 *
 * \brief	Testprogramm für einen Ringbuffer.
 */

#define TEST_OFF
#ifdef TEST_ON

#include <stdio.h>
#include <stdlib.h>

#define DEBUG(output) printf("D: %x\n",output);
#define XM_RX_BUFFER_SIZE 255 // 256 not possible!
#define PACKET_SIZE 0x12

typedef unsigned char byte;

typedef struct {
	byte putIndex;
	byte getIndex;
	byte lastPacketLength;
	byte overflow_flag;
	byte buffer[XM_RX_BUFFER_SIZE];
} RXBuffer;

RXBuffer XM_RX_buffer_L;

byte getChecksum(byte* packet, byte l) {
	byte i, chksm = 0;
	for (i = 2; i < l - 1; i++)
		chksm += packet[i];
	return ~chksm;
}

// Not using MODULAR to prevent errors caused by overflows
byte receive(RXBuffer* rxBuffer, byte* dest) {
	// Error
	if ((rxBuffer->putIndex <= rxBuffer->getIndex) && (rxBuffer->overflow_flag
			== 0x00))
		return 0;
	else if ((rxBuffer->putIndex >= rxBuffer->getIndex)
			&& (rxBuffer->overflow_flag == 0x01))
		return 0;
	else if ((rxBuffer->buffer[rxBuffer->getIndex] != 0xFF)
			&& (rxBuffer->buffer[rxBuffer->getIndex + 1] != 0xFF))
		return 0;
	// Some data received. All data received if checksum is correct!
	else {
		// Calculate predicted length
		byte length;
		if ((rxBuffer->getIndex + 3) < XM_RX_BUFFER_SIZE)
			length = rxBuffer->buffer[rxBuffer->getIndex + 3];
		else {
			length = rxBuffer->buffer[3 + rxBuffer->getIndex
					- XM_RX_BUFFER_SIZE];
		}
		// Complete length = (FF + FF + ID + LENGTH) + length
		length += 4;
		// Copy packet from buffer in destination array
		byte i;
		for (i = 0; i < length; i++) {
			if ((rxBuffer->getIndex + i) < XM_RX_BUFFER_SIZE)
				dest[i] = rxBuffer->buffer[rxBuffer->getIndex + i];
			else
				dest[i] = rxBuffer->buffer[i + rxBuffer->getIndex
						- XM_RX_BUFFER_SIZE];
		}
		// If packet is complete, set new getIndex
		if (dest[length - 1] == getChecksum(dest, length)) {
			if ((rxBuffer->getIndex + length) < XM_RX_BUFFER_SIZE)
				rxBuffer->getIndex = rxBuffer->getIndex + length;
			else {
				rxBuffer->getIndex = length + rxBuffer->getIndex
						- XM_RX_BUFFER_SIZE;
				rxBuffer->overflow_flag = 0x00;
			}
			return length;
		} else
			return 0;
	}
}

void send(byte* data, byte l) {
	byte i;
	for (i = 0; i < l; i++) {
		XM_RX_buffer_L.buffer[XM_RX_buffer_L.putIndex++] = data[i];
		if (XM_RX_buffer_L.putIndex >= XM_RX_BUFFER_SIZE) {
			XM_RX_buffer_L.putIndex = 0;
			XM_RX_buffer_L.overflow_flag = 0x01;
		}
	}
}

void printArray(byte* array, byte length) {
	byte c;
	for (c = 0; c < length; c++) {
		printf("%x", array[c]);
	}
	printf("\n");
}

void generatePacket(byte *packet, byte length) {
	byte p;
	packet[0] = 0xFF;
	packet[1] = 0xFF;
	packet[2] = 0x01;
	packet[3] = length - 4;
	for (p = 4; p < length - 1; p++) {
		packet[p] = rand() % 256;
	}
	packet[length - 1] = getChecksum(packet, length);
}

int main(int argc, char **argv) {
	int r, count;
	byte packet[PACKET_SIZE];

	// TEST 1: send, receive, send, receive, ...
	printf("TEST 1:\n");
	XM_RX_buffer_L.getIndex = 0x00;
	XM_RX_buffer_L.putIndex = 0x00;
	XM_RX_buffer_L.overflow_flag = 0x00;
	count = XM_RX_BUFFER_SIZE + 50;

	for (r = 0; r < count; r++) {
		generatePacket(packet, PACKET_SIZE);
		printf("RUN: %d\n", r);
		printf("S: ");
		printArray(packet, PACKET_SIZE);
		send(packet, PACKET_SIZE);
		while (receive(&XM_RX_buffer_L, packet) < 1)
			;
		printf("R: ");
		printArray(packet, PACKET_SIZE);
		printf("------------------\n");
	}

	// TEST 2: send, send, ..., receive, receive, ... (n*PACKET_SIZE < BUFFER_SIZE)
	printf("\nTEST 2:\n");
	XM_RX_buffer_L.getIndex = 0x00;
	XM_RX_buffer_L.putIndex = 0x00;
	XM_RX_buffer_L.overflow_flag = 0x00;
	count = (XM_RX_BUFFER_SIZE / PACKET_SIZE) - 8;

	for (r = 0; r < count; r++) {
		generatePacket(packet, PACKET_SIZE);
		printf("RUN: %d\n", r);
		printf("S: ");
		printArray(packet, PACKET_SIZE);
		send(packet, PACKET_SIZE);
		printf("------------------\n");
	}

	for (r = 0; r < count; r++) {
		while (receive(&XM_RX_buffer_L, packet) < 1)
			;
		printf("RUN: %d\n", r);
		printf("R: ");
		printArray(packet, PACKET_SIZE);
		printf("------------------\n");
	}

	// TEST 3: send, send, ..., receive, receive, ... (n*PACKET_SIZE > BUFFER_SIZE) ... ERROR!
	printf("\nTEST 3: infinite loop\n");
	XM_RX_buffer_L.getIndex = 0x00;
	XM_RX_buffer_L.putIndex = 0x00;
	XM_RX_buffer_L.overflow_flag = 0x00;
	count = (XM_RX_BUFFER_SIZE / PACKET_SIZE) + 8;

	for (r = 0; r < count; r++) {
		generatePacket(packet, PACKET_SIZE);
		printf("RUN: %d\n", r);
		printf("S: ");
		printArray(packet, PACKET_SIZE);
		send(packet, PACKET_SIZE);
		printf("------------------\n");
	}

	printf("Too many puts without enough gets!\n");

	for (r = 0; r < count; r++) {
		while (receive(&XM_RX_buffer_L, packet) < 1)
			;
		printf("RUN: %d\n", r);
		printf("R: ");
		printArray(packet, PACKET_SIZE);
		printf("------------------\n");
	}

	return 0;
}

#endif
