#if !defined(_SERVOCONTROLLER_H)
#define _SERVOCONTROLLER_H

#include <iostream>
#include <libusb.h>
#include <cstdlib>
#include "protocol.h"

using namespace std;

namespace srcHals {
	
	class servoController{
		
		private:
						
			bool deviceMatchesVendorProduct(libusb_device *device, unsigned short idVendor, unsigned short idProduct);
		
			unsigned short vendorId;
			unsigned short productIDArray[4];
			libusb_device_handle *device_handle;
			libusb_context *ctx;
			
			// Represents the current status of a channel.
			struct ServoStatus
			{
			 // The position in units of quarter-microseconds.
			 unsigned short position;
			 // The target position in units of quarter-microseconds.
			 unsigned short target;
			 // The speed limit.
			 unsigned short speed;
			 // The acceleration limit.
			 unsigned char acceleration;
			};
			typedef struct ServoStatus ServoStatus;
			
			
			// Represents the non-channel-specific variables that can be read from
			// a Micro Maestro using REQUEST_GET_VARIABLES.
			struct MicroMaestroVariables
			{
			 // The number of values on the data stack (0-32). A value of 0 means the stack is empty.
			 unsigned char stackPointer;
			 // The number of return locations on the call stack (0-10). A value of 0 means the stack is empty.
			 unsigned char callStackPointer;
			 // The error register. Each bit stands for a different error (see uscError).
			 // If the bit is one, then it means that error occurred some time since the last
			 // GET_ERRORS serial command or CLEAR_ERRORS USB command.
			 unsigned short errors;
			 // The address (in bytes) of the next bytecode instruction that will be executed.
			 unsigned short programCounter;
			 // Meaningless bytes to protect the program from stack underflows.
			 unsigned short buffer[3];
			 // The data stack used by the script. The values in locations 0 through stackPointer-1
			 // are on the stack.
			 unsigned short stack[32];
			 // The call stack used by the script. The addresses in locations 0 through
			 // callStackPointer-1 are on the call stack. The next return will make the
			 // program counter go to callStack[callStackPointer-1].
			 unsigned short callStack[10];
			 // 0 = script is running.
			 // 1 = script is done.
			 // 2 = script will be done as soon as it executes one more instruction
			 // (used to implement step-through debugging features)
			 unsigned char scriptDone;
			 // Meaningless byte to protect the program from call stack overflows.
			 unsigned char buffer2;
			 // Container for channel information. The struct is 1 byte too large (should be 7, not 8)
			 ServoStatus ServoSettings[6];
			};
			typedef struct MicroMaestroVariables MicroMaestroVariables;
			
			
			
		//	void getVariablesMicroMaestro(MaestroVariables variables, short* stack, short* callStack, ServoStatus* servos);
			
		
		public:
			servoController();
			~ servoController();
			
			void setTarget(int position, int servo);
			void getServoPosition(short* positions, short numServos);
			
		
		
	};
	
};

#endif

