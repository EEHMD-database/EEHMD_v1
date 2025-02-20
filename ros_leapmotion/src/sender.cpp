#include "std_msgs/String.h"
#include <map>
#include <iterator>
#include <iostream>
#include <string.h>
#include "Leap.h"
#include "ros/ros.h"
#include <sstream>
#include "leapmsg.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

using namespace Leap;
using namespace std;

typedef map<pair<string, string>, geometry_msgs::Point> bones_map;

class SampleListener : public Listener {
  public:
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
    virtual void onDeviceChange(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);

    ros::NodeHandle n;
    ros::Publisher leap_pub = n.advertise<leapmotion::leapmsg>("Leapmotion_raw", 1000);
  
  private:
};

geometry_msgs::Point LeapVectorToPoint(Leap::Vector vector){
	geometry_msgs::Point point;
	point.x=vector.x;
	point.y=vector.y;
	point.z=vector.z;
	return point;
}

geometry_msgs::Vector3 LeapVectorToVector3(Leap::Vector lvector){
	geometry_msgs::Vector3 vector3;
	vector3.x=lvector.x;
	vector3.y=lvector.y;
	vector3.z=lvector.z;
	return vector3;
}

void fill_leapmsg_bones(bones_map fingers_bones, leapmotion::leapmsg* msg_ptr){
	bones_map::iterator itr;
	for (itr = fingers_bones.begin(); itr != fingers_bones.end(); itr++)
	{
		if (itr->first.first == "Thumb")
		{
			if (itr->first.second == "Metacarpal"){
				msg_ptr->thumb_metacarpal=itr->second;	
			}
			else if (itr->first.second == "Proximal"){
				msg_ptr->thumb_proximal=itr->second;	
			}
			else if (itr->first.second == "Middle"){
				msg_ptr->thumb_intermediate=itr->second;	
			}
			else if (itr->first.second == "Distal"){
				msg_ptr->thumb_distal=itr->second;	
			}
			else if (itr->first.second == "Tip"){
				msg_ptr->thumb_tip=itr->second;	
			}
		}
		if (itr->first.first == "Index")
		{
			if (itr->first.second == "Metacarpal"){
				msg_ptr->index_metacarpal=itr->second;	
			}
			else if (itr->first.second == "Proximal"){
				msg_ptr->index_proximal=itr->second;	
			}
			else if (itr->first.second == "Middle"){
				msg_ptr->index_intermediate=itr->second;	
			}
			else if (itr->first.second == "Distal"){
				msg_ptr->index_distal=itr->second;	
			}
			else if (itr->first.second == "Tip"){
				msg_ptr->index_tip=itr->second;	
			}
		}
		if (itr->first.first == "Middle")
		{
			if (itr->first.second == "Metacarpal"){
				msg_ptr->middle_metacarpal=itr->second;	
			}
			else if (itr->first.second == "Proximal"){
				msg_ptr->middle_proximal=itr->second;	
			}
			else if (itr->first.second == "Middle"){
				msg_ptr->middle_intermediate=itr->second;	
			}
			else if (itr->first.second == "Distal"){
				msg_ptr->middle_distal=itr->second;	
			}
			else if (itr->first.second == "Tip"){
				msg_ptr->middle_tip=itr->second;	
			}
		}
		if (itr->first.first == "Ring")
		{
			if (itr->first.second == "Metacarpal"){
				msg_ptr->ring_metacarpal=itr->second;	
			}
			else if (itr->first.second == "Proximal"){
				msg_ptr->ring_proximal=itr->second;	
			}
			else if (itr->first.second == "Middle"){
				msg_ptr->ring_intermediate=itr->second;	
			}
			else if (itr->first.second == "Distal"){
				msg_ptr->ring_distal=itr->second;	
			}
			else if (itr->first.second == "Tip"){
				msg_ptr->ring_tip=itr->second;	
			}
		}
		if (itr->first.first == "Pinky")
		{
			if (itr->first.second == "Metacarpal"){
				msg_ptr->pinky_metacarpal=itr->second;	
			}
			else if (itr->first.second == "Proximal"){
				msg_ptr->pinky_proximal=itr->second;	
			}
			else if (itr->first.second == "Middle"){
				msg_ptr->pinky_intermediate=itr->second;	
			}
			else if (itr->first.second == "Distal"){
				msg_ptr->pinky_distal=itr->second;	
			}
			else if (itr->first.second == "Tip"){
				msg_ptr->pinky_tip=itr->second;	
			}
		}
	}

}

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};
const std::string stateNames[] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};

void SampleListener::onInit(const Controller& controller) {
  std::cout << "Initialized" << std::endl;
}

void SampleListener::onConnect(const Controller& controller) {
  std::cout << "Connected" << std::endl;
  controller.enableGesture(Gesture::TYPE_CIRCLE);
  controller.enableGesture(Gesture::TYPE_KEY_TAP);
  controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
  controller.enableGesture(Gesture::TYPE_SWIPE);
}

void SampleListener::onDisconnect(const Controller& controller) {
  // Note: not dispatched when running in a debugger.
  std::cout << "Disconnected" << std::endl;
}

void SampleListener::onExit(const Controller& controller) {
  std::cout << "Exited" << std::endl;
}

void SampleListener::onFrame(const Controller& controller) {
  // Get the most recent frame and report some basic information
  const Frame frame = controller.frame();
  /*std::cout << "Frame id: " << frame.id()
            << ", timestamp: " << frame.timestamp()
            << ", hands: " << frame.hands().count()
            << ", extended fingers: " << frame.fingers().extended().count()
            << ", tools: " << frame.tools().count()
            << ", gestures: " << frame.gestures().count() << std::endl;
  */
  leapmotion::leapmsg leapmsg;
  

  

  HandList hands = frame.hands();
  for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
    // Get the first hand
    const Hand hand = *hl;
    if (!hand.isLeft()){
	
	leapmsg.header.frame_id = "leapmsg";
	leapmsg.header.stamp = ros::Time::now();
	bones_map fingers_bones;
  	leapmsg.hands_count=frame.hands().count();
    	
	const Vector normal = hand.palmNormal();
        const Vector direction = hand.direction();
	Arm arm = hand.arm();
     	leapmsg.arm_direction=LeapVectorToVector3(arm.direction());
     	leapmsg.wrist=LeapVectorToPoint(arm.wristPosition());
     	leapmsg.elbow=LeapVectorToPoint(arm.elbowPosition());
    
	leapmsg.hand_direction=LeapVectorToVector3(hand.direction());
	leapmsg.palm_normal=LeapVectorToVector3(hand.palmNormal());

	leapmsg.palmpos=LeapVectorToPoint(hand.palmPosition());
	leapmsg.ypr.x=direction.pitch();
	leapmsg.ypr.y=normal.roll();
	leapmsg.ypr.z=direction.yaw();



    // Get fingers
    const FingerList fingers = hand.fingers();
    leapmsg.fingers_count=fingers.count();
    for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
      const Finger finger = *fl;
     /* std::cout << std::string(4, ' ') <<  fingerNames[finger.type()]
                << " finger, id: " << finger.id()
                << ", length: " << finger.length()
                << "mm, width: " << finger.width() << std::endl;
	*/
      // Get finger bones
      for (int b = 0; b < 4; ++b) {
        Bone::Type boneType = static_cast<Bone::Type>(b);
        Bone bone = finger.bone(boneType);
	fingers_bones.insert({{fingerNames[finger.type()],boneNames[boneType]},LeapVectorToPoint(bone.prevJoint())});
	if (boneNames[boneType] == "Distal"){
		fingers_bones.insert({{fingerNames[finger.type()],"Tip"},LeapVectorToPoint(bone.nextJoint())});
	}
        /*std::cout << std::string(6, ' ') <<  boneNames[boneType]
                  << " bone, start: " << bone.prevJoint()
                  << ", end: " << bone.nextJoint()
                  << ", direction: " << bone.direction() << std::endl;*/
      }
    }
 	
    //load fingers_bones in the leapmsg
    fill_leapmsg_bones(fingers_bones, &leapmsg);
    }
    
  }
  leap_pub.publish(leapmsg);
  /*
  // Get tools
  const ToolList tools = frame.tools();
  for (ToolList::const_iterator tl = tools.begin(); tl != tools.end(); ++tl) {
    const Tool tool = *tl;
    std::cout << std::string(2, ' ') <<  "Tool, id: " << tool.id()
              << ", position: " << tool.tipPosition()
              << ", direction: " << tool.direction() << std::endl;
  }
  
  // Get gestures
  const GestureList gestures = frame.gestures();
  for (int g = 0; g < gestures.count(); ++g) {
    Gesture gesture = gestures[g];

    switch (gesture.type()) {
      case Gesture::TYPE_CIRCLE:
      {
        CircleGesture circle = gesture;
        std::string clockwiseness;

        if (circle.pointable().direction().angleTo(circle.normal()) <= PI/2) {
          clockwiseness = "clockwise";
        } else {
          clockwiseness = "counterclockwise";
        }

        // Calculate angle swept since last frame
        float sweptAngle = 0;
        if (circle.state() != Gesture::STATE_START) {
          CircleGesture previousUpdate = CircleGesture(controller.frame(1).gesture(circle.id()));
          sweptAngle = (circle.progress() - previousUpdate.progress()) * 2 * PI;
        }
        std::cout << std::string(2, ' ')
                  << "Circle id: " << gesture.id()
                  << ", state: " << stateNames[gesture.state()]
                  << ", progress: " << circle.progress()
                  << ", radius: " << circle.radius()
                  << ", angle " << sweptAngle * RAD_TO_DEG
                  <<  ", " << clockwiseness << std::endl;
        break;
      }
      case Gesture::TYPE_SWIPE:
      {
        SwipeGesture swipe = gesture;
        std::cout << std::string(2, ' ')
          << "Swipe id: " << gesture.id()
          << ", state: " << stateNames[gesture.state()]
          << ", direction: " << swipe.direction()
          << ", speed: " << swipe.speed() << std::endl;
        break;
      }
      case Gesture::TYPE_KEY_TAP:
      {
        KeyTapGesture tap = gesture;
        std::cout << std::string(2, ' ')
          << "Key Tap id: " << gesture.id()
          << ", state: " << stateNames[gesture.state()]
          << ", position: " << tap.position()
          << ", direction: " << tap.direction()<< std::endl;
        break;
      }
      case Gesture::TYPE_SCREEN_TAP:
      {
        ScreenTapGesture screentap = gesture;
        std::cout << std::string(2, ' ')
          << "Screen Tap id: " << gesture.id()
          << ", state: " << stateNames[gesture.state()]
          << ", position: " << screentap.position()
          << ", direction: " << screentap.direction()<< std::endl;
        break;
      }
      default:
        std::cout << std::string(2, ' ')  << "Unknown gesture type." << std::endl;
        break;
    }
  }

  if (!frame.hands().isEmpty() || !gestures.isEmpty()) {
    std::cout << std::endl;
  }
  */
}

void SampleListener::onFocusGained(const Controller& controller) {
  std::cout << "Focus Gained" << std::endl;
}

void SampleListener::onFocusLost(const Controller& controller) {
  std::cout << "Focus Lost" << std::endl;
}

void SampleListener::onDeviceChange(const Controller& controller) {
  std::cout << "Device Changed" << std::endl;
  const DeviceList devices = controller.devices();

  for (int i = 0; i < devices.count(); ++i) {
    std::cout << "id: " << devices[i].toString() << std::endl;
    std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
  }
}

void SampleListener::onServiceConnect(const Controller& controller) {
  std::cout << "Service Connected" << std::endl;
}

void SampleListener::onServiceDisconnect(const Controller& controller) {
  std::cout << "Service Disconnected" << std::endl;
}

int main(int argc, char** argv) {
  // Create a ros node and the associated ros publisher
  ros::init(argc, argv, "leapmotion");

  // Create a sample listener and controller
  SampleListener listener;
  Controller controller;

  // Have the sample listener receive events from the controller
  controller.addListener(listener);

  if (argc > 1 && strcmp(argv[1], "--bg") == 0)
    controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);

  // Keep this process running until Enter is pressed
  std::cout << "Press Enter to quit..." << std::endl;
  std::cin.get();

  // Remove the sample listener when done
  controller.removeListener(listener);

  return 0;
}
