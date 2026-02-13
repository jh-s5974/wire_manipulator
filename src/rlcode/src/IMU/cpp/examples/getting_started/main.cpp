//프로그래밍을 할 때, 주석을 전부 설명하려는 함수 위에다가 적네. 여기서 나도 그런식으로 적음.

#include <iostream>

// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"

// We need this file for our sleep function.
#include "vn/thread.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// Method declarations for future use.
void asciiAsyncMessageReceived(void* userData, Packet& p, size_t index);
void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);

int main(int argc, char *argv[])
{
	// This example walks through using the VectorNav C++ Library to connect to
	// and interact with a VectorNav sensor.

	// First determine which COM port your sensor is attached to and update the
	// constant below. Also, if you have changed your sensor from the factory
	// default baudrate of 115200, you will need to update the baudrate
	// constant below as well.
	// const string SensorPort = "COM3";                             // Windows format for physical and virtual (USB) serial port.
	// const string SensorPort = "/dev/ttyS1";                    // Linux format for physical serial port.
	const string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
	// const string SensorPort = "/dev/tty.usbserial-FTXXXXXX";   // Mac OS X format for virtual (USB) serial port.
	// const string SensorPort = "/dev/ttyS0";                    // CYGWIN format. Usually the Windows COM port number minus 1. This would co	const uint32_t SensorBaudrate = 115200;


	const uint32_t SensorBaudrate = 115200;
	// const uint32_t SensorBaudrate = 230400;
	// Now let's create a VnSensor object and use it to connect to our sensor.
	VnSensor vs;


	vs.connect(SensorPort, SensorBaudrate);

	vs.setResponseTimeoutMs(5000); // 최대응답 기다리는 시간을 5초로 늘린다.


	// vs.restoreFactorySettings(true); // 공장 초기화
	// Let's query the sensor's model number.
	string mn = vs.readModelNumber();
	cout << "Model Number: " << mn << endl;


	// vs.tare();


    // 참조 프레임 회전을 설정할 3x3 행렬 생성
    mat3f c = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    // 참조 프레임 회전 레지스터에 행렬 쓰기
    vs.writeReferenceFrameRotation(c, true);
    // 참조 프레임 회전 레지스터 읽기
    mat3f rotationMatrix = vs.readReferenceFrameRotation();

    // 읽은 참조 프레임 회전 값 출력
    cout << "Reference Frame Rotation Matrix:" << endl;
    cout << "[" << rotationMatrix.e00 << ", " << rotationMatrix.e01 << ", " << rotationMatrix.e02 << "]" << endl;
    cout << "[" << rotationMatrix.e10 << ", " << rotationMatrix.e11 << ", " << rotationMatrix.e12 << "]" << endl;
    cout << "[" << rotationMatrix.e20 << ", " << rotationMatrix.e21 << ", " << rotationMatrix.e22 << "]" << endl;




 //프레임을 바꾸고 적용시키려면 파워를 뺐다가 다시 넣어야함. reset으로는 안됨. 그래서  공장초기화도 하면 안됨. 다 날라가자나.
 // 근데 reset함수 적용안시키면 값이 이상하게 나올때가 있음.
	vs.reset(true);  

// 요를 0으로 세팅하는 함수.	
	vs.tare(true);


//설정 덮어쓰기,이걸 쓰면 값이 이상하게 나옴. 이거 안써도 vs.tare 잘 적용됨.
	// vs.writeSettings(true);

	// Get some orientation data from the sensor.
	vec3f ypr = vs.readYawPitchRoll();
	cout << "Current YPR: " << ypr << endl;
	vec4f quat = vs.readAttitudeQuaternion();
	cout << "Current quat: " << quat << endl;


	// Get some orientation and IMU data.
	YawPitchRollMagneticAccelerationAndAngularRatesRegister reg;
	reg = vs.readYawPitchRollMagneticAccelerationAndAngularRates();
	cout << "Current YPR: " << reg.yawPitchRoll << endl;
	cout << "Current Magnetic: " << reg.mag << endl;
	cout << "Current Acceleration: " << reg.accel << endl;
	cout << "Current Angular Rates: " << reg.gyro << endl;

	// Let's do some simple reconfiguration of the sensor. As it comes from the
	// factory, the sensor outputs asynchronous data at 40 Hz. We will change
	// this to 2 Hz for demonstration purposes.
	uint32_t oldHz = vs.readAsyncDataOutputFrequency();
	vs.writeAsyncDataOutputFrequency(50); //이 이상 높일려면 보드레이트도 같이 높여야 하는 듯.
	uint32_t newHz = vs.readAsyncDataOutputFrequency();
	cout << "Old Async Frequency: " << oldHz << " Hz" << endl;
	cout << "New Async Frequency: " << newHz << " Hz" << endl;





// 더 복잡한 설정 옵션이 있는 레지스터의 경우, 현재 기존 레지스터 설정을 읽고,
// 관심 있는 값만 변경한 다음, 해당 설정을 레지스터에 쓰는 것이 편리합니다.
// 이를 통해 레지스터의 다른 필드에 대한 현재 설정을 유지할 수 있습니다.
// 아래에서는 센서에서 사용하는 heading 모드를 변경합니다.
// For the registers that have more complex configuration options, it is
// convenient to read the current existing register configuration, change
// only the values of interest, and then write the configuration to the
// register. This allows preserving the current settings for the register's
// other fields. Below, we change the heading mode used by the sensor.





//Absolute 모드는 단기 교란만 제거하고 절대 방향을 추적하며, 
//Relative 모드는 장기 교란도 처리하지만 절대 방향은 유지하기 어렵고, 
//Indoor 모드는 실내 환경에 최적화되어 장기 교란을 처리하면서도 절대 방향을 유지할 수 있습니다.

	VpeBasicControlRegister vpeReg = vs.readVpeBasicControl();
	cout << "Old Heading Mode: " << vpeReg.headingMode << endl;
	vpeReg.headingMode = HEADINGMODE_ABSOLUTE;
	vs.writeVpeBasicControl(vpeReg);
	vpeReg = vs.readVpeBasicControl();
	cout << "New Heading Mode: " << vpeReg.headingMode << endl;





	// Up to now, we have shown some examples of how to configure the sensor
	// and query for the latest measurements. However, this querying is a
	// relatively slow method for getting measurements since the CPU has to
	// send out the command to the sensor and also wait for the command
	// response. An alternative way of receiving the sensor's latest
	// measurements without the waiting for a query response, you can configure
	// the library to alert you when new asynchronous data measurements are
	// received. We will illustrate hooking up to our current VnSensor to
	// receive these notifications of asynchronous messages.

	// First let's configure the sensor to output a known asynchronous data
	// message type.





//지금까지, 우리는 센서를 구성하고 최신 측정값을 쿼리하는 방법에 대한 몇 가지 예를 보여주었습니다. 
//그러나 이 쿼리 방식은 CPU가 센서에 명령을 보내고 명령 응답을 기다려야 하므로 측정값을 얻는 데 상대적으로 느린 방법입니다. 
//쿼리 응답을 기다리지 않고 센서의 최신 측정값을 받는 다른 방법은, 새로운 비동기 데이터 측정값이 수신될 때 라이브러리가 알려주도록 구성하는 것입니다. 
//우리는 현재 VnSensor에 연결하여 이러한 비동기 메시지 알림을 받는 방법을 보여줄 것입니다.

// 먼저 센서가 알려진 비동기 데이터 메시지 유형을 출력하도록 구성합시다.



////함수에 	VNMAG	= 10,		///< Asynchronous output type is Magnetic Measurements.
//VNQTN	= 2, 이건 방향을 쿼터니안으로 나오게 설정. 그런걸 설정해주는 함수임.
//레지스터에 VNYPR, VNQTN 설정을 입력하는 함수임.
  vs.writeAsyncDataOutputType(VNYPR);
  // To try a different sensor output type, comment out the line for VNYPR and 
  // uncomment one of the GPS output types
//다른 센서 출력 유형을 시도하려면, VNYPR 줄을 주석 처리하고 GPS 출력 유형 중 하나의 주석을 해제하세요.



  //vs.writeAsyncDataOutputType(VNGPS);
  //vs.writeAsyncDataOutputType(VNG2S);


  //레지스터에 VNYPR, VNQTN 같이 레지스터에 이미 등록되 있는 값을 가지고 오는 거임. 프린트 문으로 출력할려고
  AsciiAsync asyncType = vs.readAsyncDataOutputType();

  //이게 1이 나오면 현재 레지스터에 VNYPR로 써져 있다. 이거겠지. 확인을 여려번 하는거구나.
	cout << "ASCII Async Type: " << asyncType << endl;




	// You will need to define a method which has the appropriate
	// signature for receiving notifications. This is implemented with the
	// method asciiAsyncMessageReceived. Now we register the method with the
	// VnSensor object.
//알림을 수신하기 위해 적절한 시그니처를 가진 메서드를 정의해야 합니다. 
//이것은 asciiAsyncMessageReceived 메서드로 구현됩니다. 이제 이 메서드를 VnSensor 객체에 등록합니다.


//핸들러라는 의미를 약간 알 것 같다. 
//VS라는 클래스 어떤 함수 형식이 있는 거임. 
//이 registerAsyncPacketReceived라는 이름처럼 기능하는 함수가 있는데 그거랑 asciiAsyncMessageReceived 이 함수랑 연결시키는 거임.
	vs.registerAsyncPacketReceivedHandler(NULL, asciiAsyncMessageReceived);

	// Now sleep for 5 seconds so that our asynchronous callback method can
	// receive and display receive yaw, pitch, roll packets.
//이제 5초 동안 sleep 상태로 있어서 우리의 비동기 콜백 메서드가 yaw, pitch, roll 패킷을 수신하고 표시할 수 있도록 합니다.



//프로그램을 5초 동안 대기 상태로 만듭니다.
// 이 시간 동안 센서로부터 비동기 패킷이 도착하면, 등록된 처리기인 asciiAsyncMessageReceived 함수가 호출되어 해당 패킷을 처리합니다.
// 이 예제에서는 처리기 함수가 수신된 yaw, pitch, roll 패킷을 출력하도록 구현되어 있습니다.
	cout << "Starting sleep..." << endl;
	Thread::sleepSec(5);



	// Unregister our callback method.
//콜백 메서드의 등록을 해제합니다.
	vs.unregisterAsyncPacketReceivedHandler();

	// As an alternative to receiving notifications of new ASCII asynchronous
	// messages, the binary output configuration of the sensor is another
	// popular choice for receiving data since it is compact, fast to parse,
	// and can be output at faster rates over the same connection baudrate.
	// Here we will configure the binary output register and process packets
	// with a new callback method that can handle both ASCII and binary
	// packets.

	// First we create a structure for setting the configuration information
	// for the binary output register to send yaw, pitch, roll data out at
	// 4 Hz.



// 새로운 ASCII 비동기 메시지 알림을 수신하는 것의 대안으로, 센서의 바이너리 출력 구성은 데이터 수신을 위한 또 다른 인기 있는 선택입니다.
//  바이너리 출력은 컴팩트하고, 빠르게 파싱할 수 있으며, 동일한 연결 보드레이트로 더 빠른 속도로 출력할 수 있기 때문입니다. 
//  여기서는 바이너리 출력 레지스터를 구성하고 ASCII와 바이너리 패킷을 모두 처리할 수 있는 새로운 콜백 메서드로 패킷을 처리할 것입니다.

// 먼저 yaw, pitch, roll 데이터를 4Hz로 보내기 위해 바이너리 출력 레지스터의 구성 정보를 설정할 구조체를 생성합니다.
//원본-------------------------------------------------------------------------------------------------------------------------------
	// BinaryOutputRegister bor(
	// 	ASYNCMODE_PORT1,
	// 	200,
	// 	COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL,	// Note use of binary OR to configure flags.
	// 	TIMEGROUP_NONE,
	// 	IMUGROUP_NONE,
    // GPSGROUP_NONE,
	// 	ATTITUDEGROUP_NONE,
	// 	INSGROUP_NONE,
    // GPSGROUP_NONE);
//-------------------------------------------------------------------------------------------------------------------------------
	BinaryOutputRegister bor(
		ASYNCMODE_PORT1,
		16, //VN-100T data 주기가 800HZ 이걸 16으로 나누는 거임
		COMMONGROUP_TIMESTARTUP |COMMONGROUP_YAWPITCHROLL | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,	// Note use of binary OR to configure flags.
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
    GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE,
    GPSGROUP_NONE);
	vs.writeBinaryOutput1(bor);

	vs.registerAsyncPacketReceivedHandler(NULL, asciiOrBinaryAsyncMessageReceived);

	cout << "Starting sleep..." << endl;
	Thread::sleepSec(5);

	vs.unregisterAsyncPacketReceivedHandler();

	vs.disconnect();

	return 0;
}





// This is our basic callback handler for notifications of new asynchronous
// data packets received. The userData parameter is a pointer to the data we
// supplied when we called registerAsyncPacketReceivedHandler. In this case
// we didn't need any user data so we just set this to NULL. Alternatively you
// can provide a pointer to user data which you can use in the callback method.
// One use for this is help in calling back to a member method instead of just
// a global or static method. The Packet p parameter is an encapsulation of
// the data packet. At this state, it has already been validated and identified
// as an asynchronous data message. However, some processing is required on the
// user side to make sure it is the right type of asynchronous message type so
// we can parse it correctly. The index parameter is an advanced usage item and
// can be safely ignored for now.

// 이것은 새로운 비동기 데이터 패킷 수신에 대한 알림을 위한 기본 콜백 핸들러입니다.
// userData 매개변수는 registerAsyncPacketReceivedHandler를 호출할 때 제공한 데이터에 대한 포인터입니다.
// 이 경우에는 사용자 데이터가 필요하지 않으므로 이를 NULL로 설정했습니다.
// 또는 콜백 메서드에서 사용할 수 있는 사용자 데이터에 대한 포인터를 제공할 수 있습니다.
// 이것의 한 가지 용도는 전역 또는 정적 메서드 대신 멤버 메서드로 콜백하는 데 도움을 주는 것입니다.
// Packet p 매개변수는 데이터 패킷의 캡슐화입니다.
// 이 상태에서는 이미 검증되었고 비동기 데이터 메시지로 식별되었습니다.
// 그러나 올바른 유형의 비동기 메시지 유형인지 확인하여
// 올바르게 파싱할 수 있도록 사용자 측에서 일부 처리가 필요합니다.
// index 매개변수는 고급 사용 항목이며 지금은 안전하게 무시할 수 있습니다.
void asciiAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
  // Make sure we have an ASCII packet and not a binary packet.
// ASCII 패킷이 있는지 확인하고 바이너리 패킷이 아닌지 확인합니다.
  if(p.type() != Packet::TYPE_ASCII)
    return;

  // Make sure we have a VNYPR data packet.
// VNYPR 데이터 패킷이 있는지 확인합니다.
  if(p.determineAsciiAsyncType() == VNYPR) {

    // We now need to parse out the yaw, pitch, roll data.
// 이제 yaw, pitch, roll 데이터를 파싱해야 합니다.
    vec3f ypr;
    p.parseVNYPR(&ypr);

    // Now print out the yaw, pitch, roll measurements.
// 이제 yaw, pitch, roll 측정값을 출력합니다.
    cout << "ASCII Async YPR: " << ypr << endl; 
  }

  // If the VNGPS output type was selected, print out that data
   //VNGPS 출력 유형이 선택된 경우 해당 데이터를 출력합니다.
  else if(p.determineAsciiAsyncType() == VNGPS) {

    double time;
    uint16_t week;
    uint8_t gpsFix;
    uint8_t numSats;
    vec3d lla;
    vec3f nedVel;
    vec3f nedAcc;
    float speedAcc;
    float timeAcc;

    p.parseVNGPS(&time, &week, &gpsFix, &numSats, &lla, &nedVel, &nedAcc, &speedAcc, &timeAcc);
    cout << "ASCII Async GPS: " << lla << endl;
  }

  // If the VNG2S output type was selected, print out that data
  //// VNG2S 출력 유형이 선택된 경우 해당 데이터를 출력합니다.
  else if(p.determineAsciiAsyncType() == VNG2S) {
    double time;
    uint16_t week;
    uint8_t gpsFix;
    uint8_t numSats;
    vec3d lla;
    vec3f nedVel;
    vec3f nedAcc;
    float speedAcc;
    float timeAcc;

    p.parseVNGPS(&time, &week, &gpsFix, &numSats, &lla, &nedVel, &nedAcc, &speedAcc, &timeAcc);
    cout << "ASCII Async GPS2: " << lla << endl;
  }

  else {
    cout << "ASCII Async: Type(" << p.determineAsciiAsyncType() << ")" << endl;
  }
}
















//원본----------------------------------------------------------------------------------------------------------------------------------------------------------------
// void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
// {
// 	if (p.type() == Packet::TYPE_ASCII && p.determineAsciiAsyncType() == VNYPR)
// 	{
// 		vec3f ypr;
// 		p.parseVNYPR(&ypr);
// 		cout << "ASCII Async YPR: " << ypr << endl;
// 		return;
// 	}

// 	if (p.type() == Packet::TYPE_BINARY)
// 	{
// 		// First make sure we have a binary packet type we expect since there
// 		// are many types of binary output types that can be configured.
// 	// 구성할 수 있는 많은 유형의 바이너리 출력 유형이 있으므로 
// 	// 우리가 예상하는 바이너리 패킷 유형이 있는지 먼저 확인합니다.
// 		if (!p.isCompatible(
// 			COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL,
// 			TIMEGROUP_NONE,
// 			IMUGROUP_NONE,
//       GPSGROUP_NONE,
// 			ATTITUDEGROUP_NONE,
// 			INSGROUP_NONE,
//       GPSGROUP_NONE))
//       // Not the type of binary packet we are expecting.
//   // 우리가 기대하는 유형의 바이너리 패킷이 아닙니다.
// 			return;

// 		// Ok, we have our expected binary output packet. Since there are many
// 		// ways to configure the binary data output, the burden is on the user
// 		// to correctly parse the binary packet. However, we can make use of
// 		// the parsing convenience methods provided by the Packet structure.
// 		// When using these convenience methods, you have to extract them in
// 		// the order they are organized in the binary packet per the User Manual.
// 	// 좋습니다, 우리가 예상한 바이너리 출력 패킷이 있습니다. 
// 	// 바이너리 데이터 출력을 구성하는 여러 가지 방법이 있으므로, 
// 	// 바이너리 패킷을 올바르게 파싱하는 것은 사용자의 책임입니다. 
// 	// 그러나 Packet 구조체에서 제공하는 파싱 편의 메서드를 사용할 수 있습니다.
// 	// 이러한 편의 메서드를 사용할 때는 사용 설명서에 따라 
// 	// 바이너리 패킷에 구성된 순서대로 추출해야 합니다.
// 		uint64_t timeStartup = p.extractUint64();
// 		vec3f ypr = p.extractVec3f();
// 		cout << "Binary Async TimeStartup: " << timeStartup << endl;
// 		cout << "Binary Async YPR: " << ypr << endl;

// 	}
// }
//---------------------------------------------------------------------------------------------------------------------------------------------------------------


void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
	if (p.type() == Packet::TYPE_ASCII && p.determineAsciiAsyncType() == VNYPR)
	{
		vec3f ypr;
		p.parseVNYPR(&ypr);
		cout << "ASCII Async YPR: " << ypr << endl;
		return;
	}

	if (p.type() == Packet::TYPE_BINARY)
	{
		// First make sure we have a binary packet type we expect since there
		// are many types of binary output types that can be configured.
	// 구성할 수 있는 많은 유형의 바이너리 출력 유형이 있으므로 
	// 우리가 예상하는 바이너리 패킷 유형이 있는지 먼저 확인합니다.
		if (!p.isCompatible(
			COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,
			TIMEGROUP_NONE,
			IMUGROUP_NONE,
      GPSGROUP_NONE,
			ATTITUDEGROUP_NONE,
			INSGROUP_NONE,
      GPSGROUP_NONE))
      // Not the type of binary packet we are expecting.
  // 우리가 기대하는 유형의 바이너리 패킷이 아닙니다.
			return;

		// Ok, we have our expected binary output packet. Since there are many
		// ways to configure the binary data output, the burden is on the user
		// to correctly parse the binary packet. However, we can make use of
		// the parsing convenience methods provided by the Packet structure.
		// When using these convenience methods, you have to extract them in
		// the order they are organized in the binary packet per the User Manual.
	// 좋습니다, 우리가 예상한 바이너리 출력 패킷이 있습니다. 
	// 바이너리 데이터 출력을 구성하는 여러 가지 방법이 있으므로, 
	// 바이너리 패킷을 올바르게 파싱하는 것은 사용자의 책임입니다. 
	// 그러나 Packet 구조체에서 제공하는 파싱 편의 메서드를 사용할 수 있습니다.
	// 이러한 편의 메서드를 사용할 때는 사용 설명서에 따라 
	// 바이너리 패킷에 구성된 순서대로 추출해야 합니다.
        uint64_t timeStartup = p.extractUint64();
        vec3f yawPitchRoll = p.extractVec3f();
        vec4f quaternion = p.extractVec4f();
        vec3f angularRate = p.extractVec3f();
        vec3f acceleration = p.extractVec3f();


        cout << "Binary Async Time Startup: " << timeStartup << endl;
        cout << "Binary Async Yaw/Pitch/Roll: " << yawPitchRoll << endl;
        cout << "Binary Async Quaternion: " << quaternion << endl;
        cout << "Binary Async Angular Rate: " << angularRate << endl;
        cout << "Binary Async Acceleration: " << acceleration << endl;
        // cout << "Binary Async Positino: " << Position << endl;
	}
}