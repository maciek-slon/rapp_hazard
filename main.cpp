
#include <rapp/objects/QRCode3D/QRCode3D.hpp>
#include <rapp/dynamic/navigation/localization.hpp>
#include <rapp-robots-api/navigation/navigation.hpp>
#include <rapp-robots-api/vision/vision.hpp>
#include <rapp-robots-api/communication/communication.hpp>



#include <sstream>
#include <cstdio>
#include <vector>

rapp::dynamic::localization rapp_localization;
rapp::robot::vision rapp_vision;

rapp::robot::navigation rapp_navigation;
rapp::robot::communication rapp_communication(0, NULL);

bool mute = true;

int globalLocalization(const char* QRmap_path){
	int status;
	rapp::object::Pose new_pose;
	rapp::object::QRcodeMap map;
	rapp::object::QRCode3D detected_qrcodes;


	rapp::object::picture::Ptr picture_ptr  = rapp_vision.capture_image (0, 3 , "png");
	picture_ptr->save("/home/nao/pictureTest.png");

	std::vector<std::vector<float>> robotToCameraMatrix = rapp_navigation.getTransform("CameraTop",0);



	double camera_top_matrix[3][3];
	camera_top_matrix[0][0] = 182.0992346 / 0.16;
	camera_top_matrix[0][2] = 658.7582;
	camera_top_matrix[1][1] = 185.0952141 / 0.16;
	camera_top_matrix[1][2] = 484.2186;
	camera_top_matrix[2][2] = 1.0;
	camera_top_matrix[0][1] = 0.0;
	camera_top_matrix[1][0] = 0.0;
	camera_top_matrix[2][0] = 0.0;
	camera_top_matrix[2][1] = 0.0;

	float lendmarkSize = 0.16f;

	detected_qrcodes = rapp_vision.qr_code_detection(picture_ptr, robotToCameraMatrix, camera_top_matrix, lendmarkSize);


	if(!mute)
		rapp_communication.textToSpeech("I found");
	int t = detected_qrcodes.numberOfQRcodes;
	std::stringstream ss;
	ss << t;
	if(!mute)
		rapp_communication.textToSpeech(ss.str());
	if(!mute)
		rapp_communication.textToSpeech("QRcodes");

	std::cout<<"Number of detected QRcodes: "<< std::endl;
	std::cout<< detected_qrcodes.numberOfQRcodes<<std::endl;

	if (detected_qrcodes.numberOfQRcodes > 0){
		map = rapp_localization.loadQRcodeMap(QRmap_path);
		std::string label = map.labels.at(0);
		new_pose = rapp_localization.qrCodeLocalization(detected_qrcodes,robotToCameraMatrix, map);
		bool setPose_status = rapp_navigation.setGlobalPose(new_pose);
		std::cout << setPose_status<<std::endl;
		if (setPose_status){
			status = 0;
		}else{
			if(!mute)
				rapp_communication.textToSpeech("Setting pose status FAILED");

			std::cout <<"Setting pose status FAILED"<<std::endl;

			status = 1;
		}
	}else{
		if(!mute)
			rapp_communication.textToSpeech("NO QRcodes found!");
		if(!mute)
			rapp_communication.textToSpeech("Changing my head position");

		std::cout << "NO QRcodes found!"<<std::endl;
		status = 2;
	}

	return status;
}


int main (int argc, char ** argv ) {
	int mute_trigger;
	std::istringstream ss_mute_trigger(argv[10]);
	ss_mute_trigger >> mute_trigger;
	if(mute_trigger==1){
		mute = true;
	}else{
		mute=false;
	}
	int localization_status = 2;
	bool camera_joint_range = false;
	std::vector<std::string> move_joints_names;

	move_joints_names.clear();
	move_joints_names.push_back("HeadYaw");
	std::vector<float> new_joint_angle;
	rapp_navigation.takePredefinedPosture("Stand", 0.5);
	// global localization with Nao head motion
	std::cout<<"setting head start position"<< std::endl;
	new_joint_angle.clear();
	new_joint_angle.push_back((-119*(3.1415/180)));
	while ((localization_status != 0 || camera_joint_range)){
		if(!mute)
			rapp_communication.textToSpeech("Calling global localization");

		std::cout<<"Calling global localization"<< std::endl;

		localization_status = globalLocalization("testMap.xml");
		if (localization_status==1)
		{	
			if(!mute)
				rapp_communication.textToSpeech("Application FAILED because of core agent crash");

			std::cout<<"RApp FAILED because of core agent crash"<< std::endl;
			return 1;	
		}else if (localization_status==0){
			std::cout<<"localization_status = "<< localization_status <<std::endl;
		} else if(localization_status == 2){
			bool moveJoint_status = rapp_navigation.moveJoint(move_joints_names, new_joint_angle, 0.5);
			if (moveJoint_status)
				localization_status = 1;

			new_joint_angle.at(0) = new_joint_angle.at(0)+float(30*(3.14/180));
			std::cout<<"New head position request: "<<new_joint_angle.at(0) <<std::endl;

			if (new_joint_angle.at(0)>119*(3.1415/180)){
				camera_joint_range = true;
				if(!mute)
					rapp_communication.textToSpeech("I can't rotate my head further, give me QRcodes!");
				std::cout<<"Robot head checked whole camera spectrum, Hazard detection ends"<< std::endl;
				return 1;	
			}
		}



	}


	// -----------

	// get robot position
	rapp::object::PoseStamped current_pose = rapp_navigation.getRobotPose();
	// ----

	//compose hazard pose and hazard point 
	rapp::object::Pose goal_pose_rapp;
	std::vector<float> goal_pose_input;
	float x;
	goal_pose_input.clear();
	for (int i=0; i<=6 ; i++){
		std::istringstream ss1(argv[i]);
		ss1 >> x;
		std::cout<<"pushing X : "<< x <<std::endl;

		goal_pose_input.push_back(x);
	}
	for (int i=6; i<=9 ; i++){
		std::istringstream ss2(argv[i]);
		ss2 >> x;
		std::cout<<"pushing X_2 : "<< x <<std::endl;

		goal_pose_input.push_back(x);
	}
	// hazard pose
	goal_pose_rapp.position.x = goal_pose_input.at(0);
	goal_pose_rapp.position.y = goal_pose_input.at(1);
	goal_pose_rapp.position.z = goal_pose_input.at(2);
	goal_pose_rapp.orientation.x = goal_pose_input.at(3);
	goal_pose_rapp.orientation.y = goal_pose_input.at(4);
	goal_pose_rapp.orientation.z = goal_pose_input.at(5);
	goal_pose_rapp.orientation.w = goal_pose_input.at(6);
	// hazard point
	float hazard_point_x = goal_pose_input.at(7);
	float hazard_point_y = goal_pose_input.at(8);
	float hazard_point_z = goal_pose_input.at(9);

	// ----

	// plann path
	rapp::object::PlannedPath planner_response;
	// planner_response = rapp_dynamic_navigation.plannPath2d(........ , current_pose.pose,goal_pose_rapp); 
	// 
	//

	// handle failed planning

	// ----


	// move along path
	//bool move_along_status = rapp_navigation.moveAlongPath(planner_response.path);

	// handle failed moveAlongPath

	// ----
	goal_pose_rapp.position.x = goal_pose_input.at(0);
	goal_pose_rapp.position.y = goal_pose_input.at(1);
	goal_pose_rapp.position.z = goal_pose_input.at(2);
	//rapp_navigation.rest("Crouch");

	bool look_at_point_status = rapp_navigation.lookAtPoint(hazard_point_x, hazard_point_y, hazard_point_z);

	// handle failed lookAtPoint

	// ----

	if(!mute)
		rapp_communication.textToSpeech("I'm checking the door status now.");
	sleep(5);

	// call detect hazard method
	//
	//
	//
	//    bool door_status = checkDoor();
	//
	//

	//
	// say sth


	if(!mute)
		rapp_communication.textToSpeech("The front door is opened");
	if(!mute)
		rapp_communication.textToSpeech("I need to rest now");
	//  
	//
	rapp_navigation.rest("Crouch");
	return 0;
}
