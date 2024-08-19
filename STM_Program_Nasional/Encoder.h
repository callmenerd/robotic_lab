#ifndef Encoder_h_
#define Encoder_h_

#include <Arduino.h>

#define IO_REG_TYPE                     uint32_t
#define PIN_TO_BASEREG(pin)             portInputRegister(digitalPinToPort(pin))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)

typedef struct {
	volatile IO_REG_TYPE * pin1_register;
	volatile IO_REG_TYPE * pin2_register;
  volatile int           prevT;
	IO_REG_TYPE            pin1_bitmask;
	IO_REG_TYPE            pin2_bitmask;
	uint8_t                state;
	int32_t                position;
  float                  velocity;
} Encoder_internal_state_t;

class Encoder
{

	public:

		inline	static int		instance_count;
		static Encoder_internal_state_t * interruptArgs[12];

		Encoder(int pin1, int pin2) {

			pinMode(pin1, INPUT_PULLUP);
			pinMode(pin2, INPUT_PULLUP);

			encoder.pin1_register = PIN_TO_BASEREG(pin1);
			encoder.pin1_bitmask = PIN_TO_BITMASK(pin1);
			encoder.pin2_register = PIN_TO_BASEREG(pin2);
			encoder.pin2_bitmask = PIN_TO_BITMASK(pin2);
			encoder.position = 0;
			
			delayMicroseconds(2000);
			uint8_t s = 0;
			if (DIRECT_PIN_READ(encoder.pin1_register, encoder.pin1_bitmask)) s |= 1;
			if (DIRECT_PIN_READ(encoder.pin2_register, encoder.pin2_bitmask)) s |= 2;
			encoder.state = s;

			encodernumber = instance_count;
			instance_count++;

			pin_a = pin1;
			pin_b = pin2;

			interrupt_attached = 0;
			
		}

		void attach_all_interrupts(void) {
			
			attach_interrupt(pin_a, &encoder, (encodernumber*2));
			attach_interrupt(pin_b, &encoder, ((encodernumber*2)+1));

		}


		inline int32_t read() {

			if(!interrupt_attached) {
				attach_all_interrupts();
				interrupt_attached = 1; 
			}

			noInterrupts();
			int32_t ret = encoder.position;
			interrupts();
			return ret;

		}

    inline int32_t velocity() {

			if(!interrupt_attached) {
				attach_all_interrupts();
				interrupt_attached = 1; 
			}

			noInterrupts();
			int32_t ret = encoder.velocity;
			interrupts();
			return ret;

		}

		inline int32_t readAndReset() {

			if(!interrupt_attached) {
				attach_all_interrupts();
				interrupt_attached = 1; 
			}

			noInterrupts();
			int32_t ret = encoder.position;
			encoder.position = 0;
			interrupts();
			return ret;

		}

		inline void write(int32_t p) {

			if(!interrupt_attached) {
				attach_all_interrupts();
				interrupt_attached = 1; 
			}

			noInterrupts();
			encoder.position = p;
			interrupts();

		}

		static void update(Encoder_internal_state_t *arg) {

			uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
			uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);
			uint8_t state = arg->state & 3;
      volatile int inc = 0;
			if (p1val) state |= 4;
			if (p2val) state |= 8;
			arg->state = (state >> 2);
			switch (state) {
				case 1: case 7: case 8: case 14:
					inc = 1;
          break;
				case 2: case 4: case 11: case 13:
					inc = -1;
          break;
				case 3: case 12:
					inc = 2;
          break;
				case 6: case 9:
					inc = -2;
          break;
			}
      arg->position += inc;
      volatile long currT = micros();
      volatile float deltaT = ((float) (currT - (arg->prevT)))/1000000;
      arg->velocity = (inc/deltaT)*60/537.6;
      arg->prevT = currT;
		}


		static void isr0(void) { update(interruptArgs[0]); }

		static void isr1(void) { update(interruptArgs[1]); }

		static void isr2(void) { update(interruptArgs[2]); }

		static void isr3(void) { update(interruptArgs[3]); }

		static void isr4(void) { update(interruptArgs[4]); }

		static void isr5(void) { update(interruptArgs[5]); }

		static void isr6(void) { update(interruptArgs[6]); }

		static void isr7(void) { update(interruptArgs[7]); }

		static void isr8(void) { update(interruptArgs[8]); }

		static void isr9(void) { update(interruptArgs[9]); }

		static void isr10(void) { update(interruptArgs[10]); }

		static void isr11(void) { update(interruptArgs[11]); }

		
	private:

		Encoder_internal_state_t encoder;

		int pin_a;
		int	pin_b;

		int	encodernumber;
		int interrupt_attached;

		static void attach_interrupt(int pin, Encoder_internal_state_t *state, uint8_t nummer) {


			switch (nummer) {

				case 0:
					interruptArgs[0] = state;
					attachInterrupt(digitalPinToInterrupt(pin), isr0, CHANGE);
					break;

				case 1:
					interruptArgs[1] = state;
					attachInterrupt(digitalPinToInterrupt(pin), isr1, CHANGE);
					break;

				case 2:
					interruptArgs[2] = state;
					attachInterrupt(digitalPinToInterrupt(pin), isr2, CHANGE);
					break;

				case 3:
					interruptArgs[3] = state;
					attachInterrupt(digitalPinToInterrupt(pin), isr3, CHANGE);
					break;

				case 4:
					interruptArgs[4] = state;
					attachInterrupt(digitalPinToInterrupt(pin), isr4, CHANGE);
					break;

				case 5:
					interruptArgs[5] = state;
					attachInterrupt(digitalPinToInterrupt(pin), isr5, CHANGE);
					break;

				case 6:
					interruptArgs[6] = state;
					attachInterrupt(digitalPinToInterrupt(pin), isr6, CHANGE);
					break;

				case 7:
					interruptArgs[7] = state;
					attachInterrupt(digitalPinToInterrupt(pin), isr7, CHANGE);
					break;

				case 8:
					interruptArgs[8] = state;
					attachInterrupt(digitalPinToInterrupt(pin), isr8, CHANGE);
					break;

				case 9:
					interruptArgs[9] = state;
					attachInterrupt(digitalPinToInterrupt(pin), isr9, CHANGE);
					break;

				case 10:
					interruptArgs[10] = state;
					attachInterrupt(digitalPinToInterrupt(pin), isr10, CHANGE);
					break;

				case 11:
					interruptArgs[11] = state;
					attachInterrupt(digitalPinToInterrupt(pin), isr11, CHANGE);
					break;
			}

		}
};
#endif