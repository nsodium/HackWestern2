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

#include <myo/myo.hpp> 

const int MESSAGESIZE = 11;
const int EMOJISIZE = 11;

class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
    : onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
    {
    }

    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        roll_w = 0;
        pitch_w = 0;
        yaw_w = 0;
        onArm = false;
        isUnlocked = false;
    }

    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;

        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                        1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

        roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
        pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
        yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
    }

    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        currentPose = pose;

        if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
            myo->unlock(myo::Myo::unlockHold);
            myo->notifyUserAction();
        } else {       
            myo->unlock(myo::Myo::unlockTimed);
        }
    }

    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
                   myo::WarmupState warmupState)
    {
        onArm = true;
        whichArm = arm;
    }

    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
    }

    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = true;
    }

    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = false;
    }

    void print()
    {
        std::cout << '\r';

        if (onArm) {     
            std::string poseString = currentPose.toString();
            std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
                      << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
                      << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
        } else {          
            std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
        }
        std::cout << std::flush;
    }

    bool onArm;
    myo::Arm whichArm;

    bool isUnlocked;

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

	bool recordMovement(DataCollector & collector, myo::Hub & hub) 
	{
		hub.run(1000 / 20);
		
		int initPitch = collector.pitch_w;
		int initYaw = collector.yaw_w;
		int initRoll = collector.roll_w;

		hub.run(1000/20);

		while (abs(initPitch - collector.pitch_w) < 3 && abs(initRoll - collector.pitch_w) < 3 && abs(initYaw - collector.yaw_w) < 3)
		{
			hub.run(1000 / 20);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(300));

		hub.run(1000 / 20);

		(*this).changePitch = collector.pitch_w - initPitch;
		(*this).changeRoll = collector.roll_w - initRoll;
		(*this).changeYaw = collector.yaw_w - initYaw;

		return true;
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
			std::this_thread::sleep_for(std::chrono::milliseconds(300));
		}
		return true;
	}

	std::string recognize()
	{
		if (((*this).movements[1].changePitch + (*this).movements[1].changeRoll) < abs((*this).movements[1].changeYaw))
		{
			return "WAVE";
		}
		else
		{
			return "SLICE";
		}
	}
};

void emojiMenu(DataCollector & collector, std::string emojis[], int & currentPosition, bool & breakLoop)
{
	int scrollDelay = 4200 / (pow(abs(collector.pitch_w - 8) + 1, 1.5) + 5 );
	std::cout << "Emoji Menu " << std::endl;
	std::cout << "The selected Emoji is: " << emojis[currentPosition] << std::endl;

	if (collector.pitch_w > 10 && currentPosition < (EMOJISIZE-1))
	{
			currentPosition++;
		std::cout << "The selected Emoji is: " << emojis[currentPosition] << std::endl;
	}
	else if (collector.pitch_w < 7 && currentPosition > 0)
	{
		currentPosition--;
		std::cout << "The selected Emoji is: " << emojis[currentPosition] << std::endl;
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(scrollDelay));

	if (collector.currentPose.toString() == "fingersSpread")
	{
		breakLoop = true;
		std::cout << std::endl << "     " << emojis[currentPosition] << "     " << std::endl;
	}
}

void messageMenu(DataCollector & collector, std::string messages[], int & currentPosition, bool & breakLoop)
{
	std::cout << "Message Menu " << std::endl;
	int scrollDelay = 4200 / (pow(abs(collector.pitch_w - 8) + 1, 1.5) + 5);

	std::cout << scrollDelay;

	std::cout << "The selected QuickMessage is: " << messages[currentPosition] << std::endl;

	if (collector.pitch_w > 10 && currentPosition < (MESSAGESIZE-1))
	{
		currentPosition++;
		std::cout << "The selected QuickMessage is: " << messages[currentPosition] << std::endl;
	}
	else if (collector.pitch_w < 7 && currentPosition > 0)
	{
		currentPosition--;
		std::cout << "The selected QuickMessage is: " << messages[currentPosition] << std::endl;
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(scrollDelay));

	if (collector.currentPose.toString() == "fingersSpread")
	{
		breakLoop = true;
		std::cout << std::endl << "     " << messages[currentPosition] << "     " << std::endl;
	}
}

void specialGestures(DataCollector & collector)
{
	int i = 0;
}

int main(int argc, char** argv)
{
	myo::Hub hub("com.example.hello-myo");
	std::cout << "Attempting to find a Myo..." << std::endl;
	myo::Myo* myo = hub.waitForMyo(10000);
	
	if (!myo) {
		throw std::runtime_error("Unable to find a Myo!");
	}

	std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

	DataCollector collector;
	hub.addListener(&collector);

	const int MAINMENU = 0;
	const int EMOJIMENU = 1;
	const int MESSAGEMENU = 2;
	const int WAVERECOGNITION = 3;
	const int MESSAGEREADY = 4;
	
	std::string emojis[MESSAGESIZE] = { "emj1", "emj2", "emj3", "emj4", "emj5", "emj6", "emj7", "emj8", "emj9", "emj10", "emj11" };
	std::string messages[EMOJISIZE] = { "Hi!", "What's up?", "How are you?", "I'm driving!", "I'm in a meeting!", "On my way!", "Be right back!", "Talk to you later!", "Got to go!", "Bye!", "Talk to you soon!" };

	int currentPosition = (MESSAGESIZE+EMOJISIZE)/4;
	bool breakLoopMessage = false;
	bool breakLoopEmoji = false;
	int whichMenu = 0;

	while (true)
	{
		hub.run(1000 / 20);
		collector.print();

		switch (whichMenu)
		{
			case MESSAGEMENU:
				messageMenu(collector, messages, currentPosition, breakLoopMessage);
				
				if (collector.currentPose.toString() == "fist")
				{
					std::cout << "Main Menu " << std::endl;
					whichMenu = MAINMENU;
				}

			case EMOJIMENU:
				emojiMenu(collector, emojis, currentPosition, breakLoopEmoji);
				
				if (collector.currentPose.toString() == "fist")
				{
					std::cout << "Main Menu " << std::endl;
					whichMenu = MAINMENU;
				}
	
			default:
				if (collector.currentPose.toString() == "waveIn")
				{
				std::cout << "Message Menu " << std::endl;
					whichMenu = MESSAGEMENU;
				}
				else if (collector.currentPose.toString() == "waveOut")
				{
					std::cout << "Emoji Menu " << std::endl;
					whichMenu = EMOJIMENU;
				}
		}
	}

	std::cout << "EXIT HYO APP?";
	std::string endprogram;
	std::cin >> endprogram;

	return 0;
}
