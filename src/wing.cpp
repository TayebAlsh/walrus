#include "wing/wing.h"
#include "myAHRS_plus.h"
#include "ping1d.h"  // Include the Ping1D library
#include <QNEthernet.h>  
#include "eigen.h"      // Calls main Eigen matrix class library
#include <Eigen/LU>     // Calls inverse, determinant, LU decomp., etc.


Ping1D ping(Serial1);
// MAC address (unique for your Teensy)
uint8_t mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 

// Static IP address configuration
IPAddress ip(192, 168, 2, 3);      // Static IP for the Teensy
IPAddress subnet(255, 255, 255, 0); // Subnet mask 

namespace Cyberwing
{
    //Constructor
	Wing::Wing(void):
        status_(STATUS::INIT),
        udp1_(),
        udp2_(),
        servo1_(),
        servo2_(),
        servo3_(),
        servo4_(),
        servo5_(),
        // imu_(YOST_TTS_LX_MODE,
	    //  SPI,
	    //  YOST_TTS_LX_PIN_SS,
	    //  YOST_TTS_LX_PIN_ATT,
	    //  YOST_TTS_LX_RECEIVE_TIMEOUT),
        depth_(FLUID_DENSITY),
        inPacket_(),
        outPacket_(),
        state_{0.0F},
        input_{0.0F}, 
        packetBuffer_{PACKET_SIZE_OUT},
        ping(Serial1)
        
	{}
    

    void Wing::init(void) {
        /////////////////////////
        // Initialize Serial Port
        Serial.begin(115200);

        ///////////////////////////////////
        // Initialize Ethernet (using DHCP)
        Ethernet.begin(mac, ip, subnet);
        if (!Ethernet.waitForLocalIP(5000)) {
            status_ = STATUS::FAILURE;
            Serial.println(F("Failed to configure Ethernet"));
            if (!Ethernet.linkStatus())
                Serial.println(F("Ethernet cable is not connected."));
            // Instead of stopping execution, just set the status and continue
        } else {
            Serial.print(F("Connected! Teensy IP address:"));
            Serial.println(Ethernet.localIP());
            delay(1000);
            // Bind to port for receiving packets asynchronously
            udp1_.connect(Ethernet.localIP(), RECEIVE_PORT);
            if (udp1_.listen(RECEIVE_PORT)) {
                udp1_.onPacket([&](AsyncUDPPacket packet) {
                    parsePacket(packet);
                });
            }
            // Connect to IP and Port for sending packets
            udp2_.connect(IPAddress(IP1, IP2, IP3, IP4), SEND_PORT);
            status_ = STATUS::RUNNING;
            // Initialize Joystick inputs and servos
            input_[0] = 0.0;  // Joystick yaw input
            input_[1] = 0.0;  // Joystick pitch input
            input_[2] = 0.0;  // Joystick roll input
        }

        ///////////////////
        // Initialize Servos
        servo1_.attach(SERVO1_PWM_PIN);    // attaches the servo on pin
        servo2_.attach(SERVO2_PWM_PIN);    // attaches the servo on pin
        servo3_.attach(SERVO3_PWM_PIN);    // attaches the servo on pin
        servo4_.attach(SERVO4_PWM_PIN);    // attaches the servo on pin
        servo5_.attach(SERVO5_PWM_PIN);    // attaches the servo on pin

        ////////////////////
        // Init depth sensor
        Wire.begin();
        // Wire.setSCL(DEPTH_SCL);
        // Wire.setSDA(DEPTH_SDA);

        ///////////////////
        // Initialize Depth Sensor
        Serial.println(F("Initializing depth sensor..."));
        if (!depth_.init()) {
            Serial.println("Init failed!");
            Serial.println("Are SDA/SCL connected correctly?");
            Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
            Serial.println("\n\n\n");
            status_ = STATUS::FAILURE;
            delay(1000);
        } else {
            depth_.setFluidDensity(FLUID_DENSITY);
            depth_.setModel(MS5837::MS5837_30BA);
            //depth_.setModel(MS5837::MS5837_02BA);
            Serial.println("Depth Sensor Detected!");
            Serial.print(F("Fluid density set to: "));
            Serial.println(FLUID_DENSITY);
            status_ = STATUS::RUNNING;
        }

    
        ////////////////////
        // Initialize IMU
        Wire1.begin();

        
		// imu_.initSPI();
        // //Initialize IMU
		// int32_t rtCode = imu_.init(YOST_TTS_LX_RESET_SETTINGS);
		// if(rtCode!=1)
		// {
		// 	Serial.println("IMU init failed");
		// 	status_ = STATUS::FAILURE;
		// 	// errorMessage_ = 1;
		// 	return;
		// }

        ///////////////////////
        // Leak Sensor
        pinMode(LEAK_PIN, INPUT);


        Serial1.begin(9600);

    // Initialize the Ping1D sensor and retry if it fails
        if (!ping.initialize()) {
            Serial.println("Ping device failed to initialize!");
            Serial.println("Are the Ping RX/TX pins wired correctly?");
            Serial.println("Green wire (Ping RX) → TX1 (Teensy, Pin 1)");
            Serial.println("White wire (Ping TX) → RX1 (Teensy, Pin 0)");    
        }
        else {
            Serial.println("Ping device initialized");
        }    

        //ready
		return;
	}

    //Update internal state
	void Wing::update(void)
	{       

        switch(status_)
		{
			case STATUS::RUNNING: 
            {

                // update leak sensor
                leak_ = digitalRead(LEAK_PIN);
                // if (leak_ == 1) Serial.println("Leak Detected!");
            
                // update depth sensor
                depth_.read();

                // Update IMU
                // - Done in update functdion

                // receive input packet and update 
                // receive(); - done through asynchonous function on init()
                // forwardInputs();
                forwardInputs2();
                // updateJoystickInputs();

                // - Send vehicle state to host computer
                updateState();
                updatePingData();
                publishState();
                break;
            }

            case STATUS::FAILURE:
			{
                // Set Servos to 0 deg (must be tuned...)
                int del1_ms = map(0, -1.57, 1.57, 1000, 2000);
                int del2_ms = map(0, -1.57, 1.57, 1000, 2000);
                int del3_ms = map(0, -1.57, 1.57, 1000, 2000);
                int del4_ms = map(0, -1.57, 1.57, 1000, 2000);
                //int del5_ms = map(0, -1.57, 1.57, 1000, 2000);

                servo1_.writeMicroseconds(del1_ms);
                servo2_.writeMicroseconds(del2_ms);
                servo3_.writeMicroseconds(del3_ms);
                servo4_.writeMicroseconds(del4_ms);
                //servo5_.writeMicroseconds(del5_ms);

                Serial.println("failure");

				break;
			}

            default:
			{
				status_ = STATUS::FAILURE;
				break;
			}
        } // end switch

		return;
	}

    void Wing::parsePacket(AsyncUDPPacket packet) {
        if (packet.length()) {    
            memcpy(&inPacket_, packet.data(), packet.length());  
            input_[0] = inPacket_.axis1;
            input_[1] = inPacket_.axis2;
            input_[2] = inPacket_.axis3;
            input_[3] = inPacket_.axis4;
            input_[4] = inPacket_.axis5;
        }
    }


    void Wing::forwardInputs(void) {

        // TODO> THESE GUYS MUST BE TUNNED!
        int del1_ms = map(-input_[1], -1.57, 1.57, 1000, 2000);
        int del2_ms = map(-input_[1], -1.57, 1.57, 1000, 2000);
        int del3_ms = map(input_[1], -1.57, 1.57, 1000, 2000);
        int del4_ms = map(input_[1], -1.57, 1.57, 1000, 2000);

        
        // Write to the servos
        servo1_.writeMicroseconds(del1_ms);
        servo2_.writeMicroseconds(del2_ms);
        servo3_.writeMicroseconds(del3_ms);
        servo4_.writeMicroseconds(del4_ms);
    }

    void Wing::updatePingData() {
    // Check if new data is available from the Ping1D sensor
    if (ping.update()) {

        state_[13] = static_cast<float>(ping.distance());
    } else {
        Serial.println("No update received!");
        state_[13] = 0;
    }

}

    void Wing::forwardInputs2(void) {
       // Define joystick input as a 3x1 vector (yaw, pitch, roll)
        Eigen::Vector3f joystickInput;
        joystickInput << input_[0], input_[1], input_[2];

        // Define the 4x3 transformation matrix for yaw, pitch, and roll mapping
        Eigen::Matrix<float, 4, 3> A;
        A <<  1,  -1,  -1,   // Servo 1
             -1,  -1, -1,   // Servo 2
              1,  1, -1,   // Servo 3
             -1,  1,  -1;   // Servo 4

        // Perform matrix multiplication to get servo commands (4x1 vector)
        Eigen::Vector4f servoCommands = A * joystickInput;

       // Debugging output
        // Serial.println("Servo Commands before offset: ");
        // Serial.println(servoCommands[0]);
        // Serial.println(servoCommands[1]);
        // Serial.println(servoCommands[2]);
        // Serial.println(servoCommands[3]);

        // Map the servo commands to appropriate PWM values (1000–2000 µs typical range)
        float pwm_min = 1000.0;
        float pwm_max = 2000.0;
        float pwm_neutral = 1500.0;  // Neutral position

        for (int i = 0; i < 4; i++) {
            // Scale the commands to the PWM range
            servoCommands[i] = pwm_neutral + (servoCommands[i] * (pwm_max - pwm_min) / 2);
        }

        // Update the servos with calculated commands (scaled to PWM)
        servo1_.writeMicroseconds(servoCommands[0]);  // Send command to Servo 1
        servo2_.writeMicroseconds(servoCommands[1]);  // Send command to Servo 2
        servo3_.writeMicroseconds(servoCommands[2]);  // Send command to Servo 3
        servo4_.writeMicroseconds(servoCommands[3]);  // Send command to Servo 4

        // Debugging output
        // Serial.println("Servo Commands: ");
        // Serial.println(servoCommands[0]);
        // Serial.println(servoCommands[1]);
        // Serial.println(servoCommands[2]);
        // Serial.println(servoCommands[3]);

    }

    void Wing::updateJoystickInputs(void) {
        // Assume input_ is populated elsewhere from the joystick values
        // Raw joystick inputs (yaw, pitch, roll) are in input_[0], input_[1], input_[2]
        
        // Print the raw joystick inputs for inspection
        Serial.println("Raw Joystick Inputs: ");
        Serial.print("Yaw (input[0]): ");
        Serial.println(input_[0]);
        Serial.print("Pitch (input[1]): ");
        Serial.println(input_[1]);
        Serial.print("Roll (input[2]): ");
        Serial.println(input_[2]);
    }
    
    void Wing::updateState(void)
	{
        // read state
        uint8_t buf_comp_data[18];
        readSensor(I2C_SLAVE_REG_C_ACC_X_LOW, buf_comp_data, 18);
        int16_t acc_c_x = (buf_comp_data[1]<<8) | buf_comp_data[0];
        int16_t acc_c_y = (buf_comp_data[3]<<8) | buf_comp_data[2];
        int16_t acc_c_z = (buf_comp_data[5]<<8) | buf_comp_data[4];
        int16_t gyro_c_x = (buf_comp_data[7]<<8) | buf_comp_data[6];
        int16_t gyro_c_y = (buf_comp_data[9]<<8) | buf_comp_data[8];
        int16_t gyro_c_z = (buf_comp_data[11]<<8) | buf_comp_data[10];
        // compensate
        float comp_acc_x = (float)acc_c_x * 16.0 / 32767;
        float comp_acc_y = (float)acc_c_y * 16.0 / 32767;
        float comp_acc_z = (float)acc_c_z * 16.0 / 32767;
        float comp_gyro_x = (float)gyro_c_x * 2000 / 32767;
        float comp_gyro_y = (float)gyro_c_y * 2000 / 32767;
        float comp_gyro_z = (float)gyro_c_z * 2000 / 32767;

        // read quaternion
        uint8_t buf_quat[8];
        readSensor(I2C_SLAVE_REG_QUATERNIAN_X_LOW, buf_quat, 8);
        int16_t quat_x = (buf_quat[1]<<8) | buf_quat[0];
        int16_t quat_y = (buf_quat[3]<<8) | buf_quat[2];
        int16_t quat_z = (buf_quat[5]<<8) | buf_quat[4];
        int16_t quat_w = (buf_quat[7]<<8) | buf_quat[6];
        // compensate
        float quaternion_x = (float)quat_x / 32767;
        float quaternion_y = (float)quat_y / 32767;
        float quaternion_z = (float)quat_z / 32767;
        float quaternion_w = (float)quat_w / 32767;

        // New State                
        float stateNew[18];
        stateNew[0] = comp_acc_x;
        stateNew[1] = comp_acc_y;
        stateNew[2] = comp_acc_z;
        stateNew[3] = comp_gyro_x;
        stateNew[4] = comp_gyro_y;
        stateNew[5] = comp_gyro_z;
        stateNew[6] = quaternion_x;
        stateNew[7] = quaternion_y;
        stateNew[8] = quaternion_z;
        stateNew[9] = quaternion_w;
        stateNew[10] = depth_.depth();;
        stateNew[11] = depth_.temperature();
        stateNew[12] = leak_;

        // // Debug prints of dpeth sensor
        // float rawDepth = depth_.depth();
        // float rawTemperature = depth_.temperature();

        // // Print raw depth sensor data
        // Serial.print("Raw Depth: ");
        // Serial.println(rawDepth);

        // Serial.print("Raw Temperature: ");
        // Serial.println(rawTemperature);

        // // Existing code to print processed depth sensor data
        // Serial.print("Depth: ");
        // Serial.print(stateNew[10]);
        // Serial.println(" meters");

        // Serial.print("Temperature: ");
        // Serial.print(stateNew[11]);
        // Serial.println(" °C");

                // Debug prints for raw IMU data
        // Serial.print("raw_acc_x: "); Serial.println(acc_c_x);
        // Serial.print("raw_acc_y: "); Serial.println(acc_c_y);
        // Serial.print("raw_acc_z: "); Serial.println(acc_c_z);
        // Serial.print("raw_gyro_x: "); Serial.println(gyro_c_x);
        // Serial.print("raw_gyro_y: "); Serial.println(gyro_c_y);
        // Serial.print("raw_gyro_z: "); Serial.println(gyro_c_z);

        //     // Debug prints for raw quaternion data
        // Serial.print("raw_quat_x: "); Serial.println(quat_x);
        // Serial.print("raw_quat_y: "); Serial.println(quat_y);
        // Serial.print("raw_quat_z: "); Serial.println(quat_z);
        // Serial.print("raw_quat_w: "); Serial.println(quat_w);


        memcpy(state_,stateNew,sizeof(state_));	
	}


    float Wing::my_map(int x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }


    void Wing::publishState(void) {

        outPacket_.ax = state_[0];
        outPacket_.ay = state_[1];
        outPacket_.az = state_[2];
        outPacket_.wx = state_[3];
        outPacket_.wy = state_[4];
        outPacket_.wz = state_[5];
        outPacket_.q1 = state_[6];
        outPacket_.q2 = state_[7];
        outPacket_.q3 = state_[8];
        outPacket_.q4 = state_[9];
        outPacket_.depth = state_[10];
        outPacket_.temperature = state_[11];
        outPacket_.leak = state_[12];
        // outPacket_.d1 = state_[13];
        // outPacket_.d2 = state_[14];
        // outPacket_.d3 = state_[15];
        // outPacket_.d4 = state_[16];
        // outPacket_.d5 = state_[17];
        outPacket_.ping_distance = state_[13];

        memcpy(packetBuffer_, &outPacket_, sizeof(packetBuffer_));
        udp2_.write(packetBuffer_, sizeof(packetBuffer_));
    }

    void print_mtxf(const Eigen::MatrixXf& X)  
    {
        int i, j, nrow, ncol;
        nrow = X.rows();
        ncol = X.cols();
        Serial.print("nrow: "); Serial.println(nrow);
        Serial.print("ncol: "); Serial.println(ncol);       
        Serial.println();
        for (i=0; i<nrow; i++)
    {
            for (j=0; j<ncol; j++)
            {
                Serial.print(X(i,j), 6);   // print 6 decimal places
                Serial.print(", ");
            }
            Serial.println();
    }
    Serial.println();
}

}