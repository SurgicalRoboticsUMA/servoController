#include "servoController.h"

namespace srcHals{
	
	servoController::servoController(){
		
		
		
		vendorId = 0x1ffb;
		productIDArray[0] = 0x0089;
		productIDArray[1] = 0x008a;
		productIDArray[2] = 0x008b;
		productIDArray[3] = 0x008c;
		
		
		libusb_device **device_list=0;
		libusb_init(&ctx);
		int count=libusb_get_device_list(ctx, &device_list);
		for(int i=0;i<count;i++){			
			libusb_device *device=device_list[i];
			{				
				for(int Id=0;Id<4;Id++){					
					if(deviceMatchesVendorProduct(device, vendorId, productIDArray[Id])){						
						libusb_open(device, &device_handle);
						break;
					}
				}
			}
		}
		libusb_free_device_list(device_list, 0);	
		
		 //~ Reset error	
		int ret = libusb_control_transfer(device_handle, 0x40, 0x93, 1, 0, (unsigned char*)0, 0, (short)5000);
		ret = libusb_control_transfer(device_handle, 0x40, 0x91, 0, 0, (unsigned char*)0, 0, (short)5000);
	}
	
	servoController::~servoController(){
		
		libusb_close(device_handle);
		libusb_exit(ctx);
		
	}
	
	
	bool servoController::deviceMatchesVendorProduct(libusb_device *device, unsigned short idVendor, unsigned short idProduct)
	{
		libusb_device_descriptor desc;
		libusb_get_device_descriptor(device, &desc);
		return idVendor == desc.idVendor && idProduct == desc.idProduct;
	}

	void servoController::setTarget(int position, int servo)
	{
		libusb_control_transfer(device_handle, 0x40, REQUEST_SET_TARGET, position*4, servo, 0, 0, (ushort)5000);
		
	}
		
	void servoController::getServoPosition(short* positions, short numServos)
	{
				
		unsigned char* readArray = (unsigned char*)calloc(sizeof(MicroMaestroVariables), sizeof(unsigned char));
		
		
		int ret;
		ret = libusb_control_transfer(device_handle, 0xC0, REQUEST_GET_VARIABLES, 0, 0, (unsigned char*) readArray, sizeof(MicroMaestroVariables), (short)5000);
 
					
		
		ServoStatus* servos;		
		servos = new ServoStatus[numServos];
		
		
		servos[0] = *((ServoStatus*)(readArray + sizeof(MicroMaestroVariables) - 6*sizeof(ServoStatus)));
		servos[1] = *((ServoStatus*)(readArray + sizeof(MicroMaestroVariables) - 5*sizeof(ServoStatus) -1));
		
		
		//~ for (int i = 0; i < 6; i++){
			//~ // the ServoStatus struct is rounded to 8 bytes by sizeof() instead of the 7 it takes up, must account for the error
			//~ servos[i] = *((ServoStatus*)(readArray + sizeof(MicroMaestroVariables) - (sizeof(ServoStatus) * (6 - i)) - i));
		//~ }
		
		for (int i = 0; i < numServos; i++){
			positions[i] = servos[i].position/4;
				
			//~ cout<<positions[i]<<endl;
		}
	
    }

	
}

//~ int main()
//~ {
	//~ 
	//~ srcHals::servoController controller;
	//~ 
    //~ while(1)
    //~ {
		//~ 
        //~ int position;
        //~ int servo=1;
        //~ cout << "Enter position: ";
        //~ cin >> position;
        //~ controller.setTarget(position, servo);
        //~ 
        //~ 
        //~ short positions[2];
        //~ 
        //~ controller.getServoPosition(positions, 2);
    //~ }
    //~ return 0;
//~ }
