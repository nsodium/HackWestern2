/*
fist
waveIn
waveOut
fingersSpread
doubleTap
*/

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <chrono>
#include <thread>

#include <myo/myo.hpp> // The only file that needs to be included to use the Myo C++ SDK is myo.hpp.

const int MESSAGESIZE = 11;
const int EMOJISIZE = 11;

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
    : onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
    {
    }

    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        // We've lost a Myo.
        // Let's clean up some leftover state.
        roll_w = 0;
        pitch_w = 0;
        yaw_w = 0;
        onArm = false;
        isUnlocked = false;
    }

    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;

        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                        1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

        // Convert the floating point angles in radians to a scale from 0 to 18.
        roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
        pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
        yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
    }

    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        currentPose = pose;

        if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
            // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
            // Myo becoming locked.
            myo->unlock(myo::Myo::unlockHold);

            // Notify the Myo that the pose has resulted in an action, in this case changing
            // the text on the screen. The Myo will vibrate.
            myo->notifyUserAction();
        } else {
            // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
            // are being performed, but lock after inactivity.
            myo->unlock(myo::Myo::unlockTimed);
        }
    }

    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
                   myo::WarmupState warmupState)
    {
        onArm = true;
        whichArm = arm;
    }

    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
    }

    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = true;
    }

    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = false;
    }

    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.

    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
        // Clear the current line
        std::cout << '\r';

        // Print out the orientation. Orientation data is always available, even if no arm is currently recognized.

       std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
                  << '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
                  << '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';

        if (onArm) {
            // Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.

            // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
            // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
            // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
            std::string poseString = currentPose.toString();

            std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
                      << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
                      << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
        } else {
            // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
            std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
        }
		

        std::cout << std::flush;
    }

    // These values are set by onArmSync() and onArmUnsync() above.
    bool onArm;
    myo::Arm whichArm;

    // This is set by onUnlocked() and onLocked() above.
    bool isUnlocked;

    // These values are set by onOrientationData() and onPose() above.
    int roll_w, pitch_w, yaw_w;
    myo::Pose currentPose;
};

class Movement
{
public:
	int changePitch, changeYaw, changeRoll;

	Movement()
	{
		changePitch = 0;
		changeYaw = 0;
		changeRoll = 0;
	}

	bool recordMovement(DataCollector & collector, myo::Hub & hub) //BAD CODING PRACTICE, MAKE A CONSTRUCTOR
	{
		hub.run(1000 / 20); //Potentially erroneous 
		
		int initPitch = collector.pitch_w;
		int initYaw = collector.yaw_w; //take the distance one way and then the opposite way; use whichever is shortest (SINCE YAW STARTS ON THE EDGE)
		int initRoll = collector.roll_w;

		hub.run(1000/20);

		while (abs(initPitch - collector.pitch_w) < 3 && abs(initRoll - collector.pitch_w) < 3 && abs(initYaw - collector.yaw_w) < 3)
		{
			hub.run(1000 / 20);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(900));

		hub.run(1000 / 20);

		(*this).changePitch = collector.pitch_w - initPitch;
		(*this).changeRoll = collector.roll_w - initRoll;
		(*this).changeYaw = collector.yaw_w - initYaw;
		return true;

		//hub.run(1000 / 20);
		//initPitch = collector.pitch_w;
		//initYaw = collector.yaw_w;
		//initRoll = collector.roll_w;
	}
};

class Gesture
{
public:
	Movement movements[3];

	Gesture()
	{}

	bool recordGesture(DataCollector & collector, myo::Hub & hub)
	{
		for (int i = 0; i < 3; i++)
		{
			(*this).movements[i].recordMovement(collector, hub);
			std::this_thread::sleep_for(std::chrono::milliseconds(900));
		}
		return true;
	}
};

void emojiMenu(DataCollector & collector, std::string emojis[], int & currentPosition, bool & breakLoop)
{
	int scrollDelay = 4200 / (pow(abs(collector.pitch_w - 8) + 1, 1.5) + 5 );

	std::cout << scrollDelay;

	std::cout << "The current position(Emoji) is " << currentPosition << std::endl;

	if (collector.pitch_w > 10 && currentPosition < (EMOJISIZE-1))
	{
			currentPosition++;
		std::cout << "The current position(Emoji) is " << currentPosition << std::endl;
	}
	else if (collector.pitch_w < 7 && currentPosition > 0)
	{
		currentPosition--;
		std::cout << "The current position(Emoji) is " << currentPosition << std::endl;
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(scrollDelay));

	if (collector.currentPose.toString() == "fingersSpread")
	{
		breakLoop = true;
		std::cout << "Ready to cout: " << emojis[currentPosition] << std::endl;
	}
}

void messageMenu(DataCollector & collector, std::string messages[], int & currentPosition, bool & breakLoop)
{
	int scrollDelay = 4200 / (pow(abs(collector.pitch_w - 8) + 1, 1.5) + 5);

	std::cout << scrollDelay;

	std::cout << "The current position(Message) is " << currentPosition << std::endl;

	if (collector.pitch_w > 10 && currentPosition < (MESSAGESIZE-1))
	{
		currentPosition++;
		std::cout << "The current position(Message) is " << currentPosition << std::endl;
	}
	else if (collector.pitch_w < 7 && currentPosition > 0)
	{
		currentPosition--;
		std::cout << "The current position(Message) is " << currentPosition << std::endl;
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(scrollDelay));

	if (collector.currentPose.toString() == "fingersSpread")
	{
		breakLoop = true;
		std::cout << "Ready to cout: " << messages[currentPosition] << std::endl;
	}
}

void specialGestures(DataCollector & collector)
{
	int i = 0;
}

int main(int argc, char** argv)
{
	// We catch any exceptions that might occur below -- see the catch statement for more details.
	/*try {*/

	// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
	// publishing your application. The Hub provides access to one or more Myos.
	myo::Hub hub("com.example.hello-myo");

	std::cout << "Attempting to find a Myo..." << std::endl;

	// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
	// immediately.
	// waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
	// if that fails, the function will return a null pointer.
	myo::Myo* myo = hub.waitForMyo(10000);

	// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
	if (!myo) {
		throw std::runtime_error("Unable to find a Myo!");
	}

	// We've found a Myo.
	std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

	// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
	DataCollector collector;

	// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
	// Hub::run() to send events to all registered device listeners.
	hub.addListener(&collector);

	// Finally we enter our main loop.

	
	const int MAINMENU = 0;
	const int EMOJIMENU = 1;
	const int MESSAGEMENU = 2;
	const int MESSAGEREADY = 3;
	
	std::string emojis[MESSAGESIZE] = { "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11" };
	std::string messages[EMOJISIZE] = { "1a", "2a", "3a", "4a", "5a", "6a", "7a", "8a", "9a", "10a", "11a" };


	while (true) 
	{
		int currentPosition = (MESSAGESIZE+EMOJISIZE)/4;
		bool breakLoopMessage = false;
		bool breakLoopEmoji = false;
		int whichMenu = 0;
	
		//Movement move1;
		//bool recorded = false;

		Gesture g1;
		bool recorded = false;

		while (true)
		{

			// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
			// In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
			hub.run(1000 / 20);
			// After processing events, we call the print() member function we defined above to print out the values we've
			// obtained from any events that have occurred.
			collector.print();


			if (recorded == false)
			{
				recorded = g1.recordGesture(collector, hub);
			}
			
			for (int i = 0; i < 3; i++)
			{
				std::cout << g1.movements[i].changePitch << g1.movements[i].changeRoll << g1.movements[i].changeYaw << "   ";
			}
			


			/*if (recorded == false)
			{
				recorded = move1.recordMovement(collector, hub);
			}
			std::cout << move1.changePitch << move1.changeRoll << move1.changeYaw; */


			switch (whichMenu)
			{
				case MESSAGEMENU:
					messageMenu(collector, messages, currentPosition, breakLoopMessage);
					//if command to return to mainMenu, return to mainMenu
					if (breakLoopMessage == true)
					{	
						//Change whichMenu and remove "break;"
						break;
					}

				case EMOJIMENU:
					emojiMenu(collector, emojis, currentPosition, breakLoopEmoji);
					//if command to return to mainMenu, return to mainMenu
					if (breakLoopEmoji == true)
					{	
						//Change whichMenu and remove "break;"
						break;	
					}
					
				default:
					if (collector.currentPose.toString() == "waveIn")
					{
						std::cout << "TextMenu " << std::endl;
						whichMenu = MESSAGEMENU;
					}
					else if (collector.currentPose.toString() == "waveOut") //MAYBE REMOVE "ELSE"
					{
						std::cout << "EmojiMenu " << std::endl;
						whichMenu = EMOJIMENU;
					}
			}


			/* FOR ANGLE//////////////////////////////////
			if (collector.pitch_w > 9)
			{
				std::cout << "Hello!";
				std::this_thread::sleep_for(std::chrono::milliseconds(1500));
			
			if (collector.currentPose.toString() == "fist")
			{
				std::cout << "Fist!";
				std::this_thread::sleep_for(std::chrono::milliseconds(1500));
			}
			else if (collector.currentPose.toString() == "waveIn")
			{
				std::cout << "Hello!";
				std::this_thread::sleep_for(std::chrono::milliseconds(1500));
			}
			else if (collector.currentPose.toString() == "waveOut")
			{
				std::cout << "Bye!";
				std::this_thread::sleep_for(std::chrono::milliseconds(1500));
			}
			else if (collector.currentPose.toString() == "fingersSpread")
			{
				std::cout << "What's up!";
				std::this_thread::sleep_for(std::chrono::milliseconds(1500));
			}*/

			
			
			

			//int currentPosition = 1;

			//if (collector.currentPose.toString() == "waveIn")
			//{
			//	std::cout << "WqveIn" << std::endl;
			//	if (currentPosition > 0)
			//	{
			//		std::cout << currentPosition;
			//		currentPosition--;
			//		std::cout << currentPosition;
			//	}
			//	if (currentPosition<0)
			//	{
			//		//vibrate
			//		std::cout << "Vibrate, too far" << std::endl;
			//		continue;
			//	}
			//	std::this_thread::sleep_for(std::chrono::milliseconds(500));
			//}

			//if (collector.currentPose.toString() == "waveOut")
			//{
			//	if (currentPosition < 2)
			//	{
			//		std::cout << currentPosition;
			//		currentPosition++;
			//		std::cout << currentPosition;
			//	}
			//	else
			//	{
			//		//vibrate
			//		std::cout << "Vibrate, too far";
			//		continue;
			//	}
			//	std::this_thread::sleep_for(std::chrono::milliseconds(500));
			//}

			//if (currentPosition == 0 )
			//{
			//	if (collector.currentPose.toString() == "fingersSpread")
			//	{
			//		//emoji menu
			//		std::cout << "Emoji Menu";
			//	}
			//}
			//else if (currentPosition == 2 && collector.currentPose.toString() == "fingersSpread")
			//{
			//	if (collector.currentPose.toString() == "fingersSpread")
			//	{
			//		//quick message menu
			//		std::cout << "Quick Message Menu";
			//	}
			//}
	  //  }

			// If a standard exception occurred, we print out its message and exit.
			/*} catch (const std::exception& e) {
				std::cerr << "Error: " << e.what() << std::endl;
				std::cerr << "Press enter to continue.";
				std::cin.ignore();
				return 1;
			}*/
	
		}
	while (true)
	{}

	}

}
