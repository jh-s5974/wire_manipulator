#include <iostream>

// Include this header file to get access to the EzAsyncData class.
#include "vn/ezasyncdata.h"

// We need this file for our sleep function.
#include "vn/thread.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

int main(int argc, char *argv[])
{
	// This example walks through using the EzAsyncData class to easily access
	// asynchronous data from a VectorNav sensor at a slight performance hit which is
	// acceptable for many applications, especially simple data logging.

	// First determine which COM port your sensor is attached to and update the
	// constant below. Also, if you have changed your sensor from the factory
	// default baudrate of 115200, you will need to update the baudrate
	// constant below as well.
	// const string SensorPort = "COM1";                             // Windows format for physical and virtual (USB) serial port.
	// const string SensorPort = "/dev/ttyS1";                    // Linux format for physical serial port.
	const string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
	// const string SensorPort = "/dev/tty.usbserial-FTXXXXXX";   // Mac OS X format for virtual (USB) serial port.
	// const string SensorPort = "/dev/ttyS0";                    // CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1.
	const uint32_t SensorBaudrate = 115200;


	// // 먼저 센서가 연결된 COM 포트를 확인하고 아래의 상수를 업데이트하십시오.
	// // 또한, 센서의 보드레이트를 공장 기본값인 115200에서 변경한 경우, 아래의 보드레이트 상수도 업데이트해야 합니다.
	// // const string SensorPort = "COM1";                             // 물리적 및 가상 (USB) 직렬 포트에 대한 Windows 형식.
	// // const string SensorPort = "/dev/ttyS1";                    // 물리적 직렬 포트에 대한 Linux 형식.
	// const string SensorPort = "/dev/ttyUSB0";                  // 가상 (USB) 직렬 포트에 대한 Linux 형식.
	// // const string SensorPort = "/dev/tty.usbserial-FTXXXXXX";   // 가상 (USB) 직렬 포트에 대한 Mac OS X 형식.
	// // const string SensorPort = "/dev/ttyS0";                    // CYGWIN 형식. 일반적으로 Windows COM 포트 번호에서 1을 뺀 값입니다. 이것은 COM1에 연결됩니다.
	// const uint32_t SensorBaudrate = 115200;


	// We create and connect to a sensor by the call below.
	// 아래의 호출로 센서를 생성하고 연결합니다.
	EzAsyncData* ez = EzAsyncData::connect(SensorPort, SensorBaudrate);

	// Now let's display the latest yaw, pitch, roll data at 5 Hz for 5 seconds.
	// 이제 5초 동안 5Hz로 최신 yaw, pitch, roll 데이터를 표시해 보겠습니다.
	for (int i = 0; i < 25; i++)
	{
		Thread::sleepMs(20);//단위가 ms임 0.001초 = 1ms, 아니 최대 속도가 50Hz네. 50Hz =0.02초 

		// This reads the latest data that has been processed by the EzAsyncData class.
		 // 이것은 EzAsyncData 클래스에 의해 처리된 최신 데이터를 읽습니다.
		CompositeData cd = ez->currentData();

		// Make sure that we have some yaw, pitch, roll data.
		// yaw, pitch, roll 데이터가 있는지 확인합니다.
		if (!cd.hasYawPitchRoll())
			cout << "YPR Unavailable." << endl;
		else
			cout << "Current YPR: " << cd.yawPitchRoll() << endl;
	}

	// Most of the asynchronous data handling is done by EzAsyncData but there are times
	// when we wish to configure the sensor directly while still having EzAsyncData do
	// most of the grunt work.This is easily accomplished and we show changing the ASCII
	// asynchronous data output type here.

	// 대부분의 비동기 데이터 처리는 EzAsyncData에 의해 수행되지만,
	// EzAsyncData가 대부분의 힘든 작업을 수행하면서도 센서를 직접 구성하고 싶을 때가 있습니다.
	// 이것은 쉽게 달성할 수 있으며, 여기서는 ASCII 비동기 데이터 출력 유형을 변경하는 것을 보여줍니다.
	try
	{
		ez->sensor()->writeAsyncDataOutputType(VNYPR);
	}
	catch (...)
	{
		cout << "Error setting async data output type." << endl;
		return -1;
	}

	cout << "[New ASCII Async Output]" << endl;

	// We can now display yaw, pitch, roll data from the new ASCII asynchronous data type.
	// 이제 새로운 ASCII 비동기 데이터 유형에서 yaw, pitch, roll 데이터를 표시할 수 있습니다.
	for (int i = 0; i < 25; i++)
	{
		Thread::sleepMs(20);

		CompositeData cd = ez->currentData();

		if (!cd.hasYawPitchRoll())
			cout << "YPR Unavailable." << endl;
		else
			cout << "Current YPR: " << cd.yawPitchRoll() << endl;
	}

	// The CompositeData structure contains some helper methods for getting data
	// into various formats. For example, although the sensor is configured to
	// output yaw, pitch, roll, our application might need it as a quaternion
	// value. However, if we query the quaternion field, we see that we don't
	// have any data.
// CompositeData 구조체에는 다양한 형식으로 데이터를 가져오는 헬퍼 메서드가 포함되어 있습니다.
// 예를 들어, 센서가 yaw, pitch, roll을 출력하도록 구성되어 있더라도,
// 우리 애플리케이션에서는 쿼터니언 값으로 필요할 수 있습니다.
// 그러나 쿼터니언 필드를 쿼리하면 데이터가 없다는 것을 알 수 있습니다.
	cout << "HasQuaternion: " << ez->currentData().hasQuaternion() << endl;

	// Uncommenting the line below will cause an exception to be thrown since
	// quaternion data is not available.

	// cout << "Current Quaternion: " << ez->currentData().quaternion() << endl;

	// However, the CompositeData structure provides the anyAttitude field
	// which will perform the necessary conversions automatically.

// 아래 줄의 주석을 해제하면 쿼터니언 데이터를 사용할 수 없기 때문에 예외가 발생합니다.

// cout << "Current Quaternion: " << ez->currentData().quaternion() << endl;

// 그러나 CompositeData 구조체는 anyAttitude 필드를 제공하므로
// 필요한 변환을 자동으로 수행합니다.
	cout << "[Quaternion from AnyAttitude]" << endl;

	for (int i = 0; i < 25; i++)
	{
		Thread::sleepMs(20);

		// This reads the latest data that has been processed by the EzAsyncData class.
		CompositeData cd = ez->currentData();

		// Make sure that we have some attitude data.
		if (!cd.hasAnyAttitude())
			cout << "Attitude Unavailable." << endl;
		else
			cout << "Current Quaternion: " << cd.anyAttitude().quat() << endl;
	}

	// Throughout this example, we have been using the ez->currentData() to get the most
	// up-to-date readings from the sensor that have been processed. When called, this
	// method returns immediately with the current values, thus the reason we have to
	// put the Thread::sleepMs(200) in the for loop. Otherwise, we would blaze through
	// the for loop and just print out the same values. The for loop below illustrates
	// this.
// 이 예제 전반에 걸쳐, 우리는 ez->currentData()를 사용하여 처리된 센서에서
// 가장 최신의 판독값을 얻었습니다. 호출될 때, 이 메서드는 현재 값과 함께 즉시 반환되므로,
// for 루프에서 Thread::sleepMs(200)를 사용해야 하는 이유입니다.
// 그렇지 않으면, for 루프를 빠르게 통과하고 같은 값만 출력할 것입니다.
// 아래의 for 루프는 이를 보여줍니다.
	cout << "[For Loop Without Sleep]" << endl;

	for (int i = 0; i < 25; i++)
	{
		CompositeData cd = ez->currentData();

		if (!cd.hasYawPitchRoll())
			cout << "YPR Unavailable." << endl;
		else
			cout << "Current YPR: " << cd.yawPitchRoll() << endl;
	}

	// Often, we would like to get and process each packet received from the sensor.
	// This is not realistic with ez->currentData() since it is non-blocking and we
	// would also have to compare each CompositeData struture for changes in the data.
	// However, EzAsyncData also provides the getNextData() method which blocks until
	// a new data packet is available. The for loop below shows how to output each
	// data packet received from the sensor using getNextData().
// 종종, 우리는 센서에서 수신한 각 패킷을 가져와서 처리하고 싶습니다.
// 이는 ez->currentData()로는 현실적이지 않습니다.
// 왜냐하면 논블로킹이고, 각 CompositeData 구조체의 데이터 변경 사항도 비교해야 하기 때문입니다.
// 그러나 EzAsyncData는 새로운 데이터 패킷이 사용 가능할 때까지 차단하는 getNextData() 메서드도 제공합니다.
// 아래의 for 루프는 getNextData()를 사용하여 센서에서 수신한 각 데이터 패킷을 출력하는 방법을 보여줍니다.

	cout << "[getNextData Method]" << endl;

	for (int i = 0; i < 25; i++)
	{
		CompositeData cd = ez->getNextData();

		if (!cd.hasYawPitchRoll())
			cout << "YPR Unavailable." << endl;
		else
			cout << "Current YPR: " << cd.yawPitchRoll() << endl;
	}

	ez->disconnect();

	delete ez;

	return 0;
}
