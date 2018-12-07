//#define LIBBOT_PRESENT 0

#include <iostream>
#include <ctime>
#include <sys/stat.h>
#include <fstream>
#include <cstdlib>
#include <list>
#include <string>
#include <ros/ros.h>
#include <vector>
#include <rrtstar_msgs_point/rrtStarSRV.h>
#include <rrtstar_msgs_volume/rrtStarSRV.h>
#include <rrtstar_msgs_point/Region.h>
#include <geometry_msgs/Vector3.h>
#include <time.h>
#include <signal.h>


using namespace std;
void sig_handler(int);

void randObstacleCenterGenerator (double *obsSize, double *obsCenter,const double *rootcart,const double *goalcenter,const double *goalsize);

int main (int argc, char** argv) {

	ros::init(argc, argv, "test_path_planner");
	ros::NodeHandle nh;

	if (signal(SIGINT, sig_handler) == SIG_ERR)
	{
		printf("\ncan't catch SIGINT\n");
	}

	//	ros::ServiceServer service = nh.advertiseService("rrtStar_volume_Service",generatePath);

	//	const char* home=getenv("HOME");
	//	string log_path(home);
	//	log_path=+"/Datalog/PathPlanner/Test1";
	//
	//	const char* DataLogPath	(log_path.c_str());
	//	mkdir(DataLogPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	//	const char* DataLogPathTime	="/home/nasa/Datalog/Planner/timeEvaluation";
	//	string DataLogPath2Time		="/home/nasa/Datalog/Planner/timeEvaluation";
	//	mkdir(DataLogPathTime, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	//
	//	ofstream file1_trajectory, file2_regions;
	//	char stringIter[10];


	//	sprintf(stringIter, "%d", iter);
	//	Myfile1.open ((DataLogPath2+"/"+stringIter+"Regions.txt").c_str(),ios::trunc);

	ros::ServiceClient rrtStar_point_SRV_client  = nh.serviceClient<rrtstar_msgs_point::rrtStarSRV>("rrtStar_point_Service");
	ros::ServiceClient rrtStar_volume_SRV_client = nh.serviceClient<rrtstar_msgs_volume::rrtStarSRV>("rrtStar_volume_Service");

	srand(time(0));

	const char* LogPath	="/home/nasa/Datalog/PathPlanner";
	int counter=1;
	mkdir(LogPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	double timeBeforeCall, timeAfterCall;
	// Define the operating region
	//    system.regionOperating.setNumDimensions(3);




	//    rrtstar_msgs_point::Region obstacle;
	//
	//    obstacle.center_x= 0.0;
	//    obstacle.center_x= 0.0;
	//    obstacle.center_x= 0.0;
	//    obstacle.size_x= 2.0;
	//    obstacle.size_y= 2.0;
	//    obstacle.size_z= 2.0;
	//
	//    serviceMSG_point.request.Obstacles.push_back(obstacle);
	//
	//
	//    if( rrtStar_point_SRV_client.call(serviceMSG_point))
	//    {
	//    	cout<<"Waypoint:"<<endl;
	//    	for(int i=0;i<serviceMSG_point.response.path.size();i++)
	//    	{
	//    		double wayPoint[3];
	//    		wayPoint[0]=serviceMSG_point.response.path[i].x;
	//    		wayPoint[1]=serviceMSG_point.response.path[i].y;
	//    		wayPoint[2]=serviceMSG_point.response.path[i].z;
	//    		cout<< wayPoint[0]<<" "<<wayPoint[1]<<" "<<wayPoint[2]<<endl;
	//    	}
	//
	//    }

	for (int robotSize=0; robotSize<5;robotSize++)
	{
		//! 1- Define the Msg types
		rrtstar_msgs_point::rrtStarSRV  serviceMSG_rrt_point;
		rrtstar_msgs_volume::rrtStarSRV serviceMSG_rrt_volume;


		//! 2- Define WS region, goal Region , and initial Pose

		//! 2-A: WS regions
		serviceMSG_rrt_point.request.WS.center_x= 0.0;
		serviceMSG_rrt_point.request.WS.center_y = 0.0;
		serviceMSG_rrt_point.request.WS.center_z = 0.0;
		serviceMSG_rrt_point.request.WS.size_x= 40.0-robotSize;
		serviceMSG_rrt_point.request.WS.size_y= 40.0-robotSize;
		serviceMSG_rrt_point.request.WS.size_z= 40.0-robotSize;


		serviceMSG_rrt_volume.request.WS.center_x= 0.0;
		serviceMSG_rrt_volume.request.WS.center_y = 0.0;
		serviceMSG_rrt_volume.request.WS.center_z = 0.0;
		serviceMSG_rrt_volume.request.WS.size_x= 40.0;
		serviceMSG_rrt_volume.request.WS.size_y= 40.0;
		serviceMSG_rrt_volume.request.WS.size_z= 40.0;


		//! 2-B: goal regions
		serviceMSG_rrt_point.request.Goal.center_x = 15.0;
		serviceMSG_rrt_point.request.Goal.center_y= 15.0;
		serviceMSG_rrt_point.request.Goal.center_z = 15.0;
		serviceMSG_rrt_point.request.Goal.size_x= 2.00;
		serviceMSG_rrt_point.request.Goal.size_y= 2.00;
		serviceMSG_rrt_point.request.Goal.size_z= 2.00;

		serviceMSG_rrt_volume.request.Goal.center_x = 15.0;
		serviceMSG_rrt_volume.request.Goal.center_y= 15.0;
		serviceMSG_rrt_volume.request.Goal.center_z = 15.0;
		serviceMSG_rrt_volume.request.Goal.size_x= 2.00;
		serviceMSG_rrt_volume.request.Goal.size_y= 2.00;
		serviceMSG_rrt_volume.request.Goal.size_z= 2.00;


		//! 2-C: initial position
		serviceMSG_rrt_point.request.Init.x=-15.0;
		serviceMSG_rrt_point.request.Init.y=-15.0;
		serviceMSG_rrt_point.request.Init.z=-15.0;

		serviceMSG_rrt_volume.request.Object.center_x=-15.0;
		serviceMSG_rrt_volume.request.Object.center_y=-15.0;
		serviceMSG_rrt_volume.request.Object.center_z=-15.0;
		//! 3- Define the robot size
		serviceMSG_rrt_volume.request.Object.size_x=robotSize;
		serviceMSG_rrt_volume.request.Object.size_y=robotSize;
		serviceMSG_rrt_volume.request.Object.size_z=robotSize;


		//! 4- Define No of obstacles
		//		for (int NoObs=1; NoObs<7;NoObs++)
		for (int NoObs=6; NoObs<7;NoObs++)
		{

//			for(int obsSize=1;obsSize<6;obsSize++)
			for(int obsSize=5;obsSize<6;obsSize++)
			{


				const char* LogPath2=((string)LogPath+"/"+to_string(counter)+"_robotSize_"+to_string(robotSize)+"_noObs_"+
						to_string(NoObs)+"_obsSize_"+ to_string(obsSize)).c_str();

				mkdir(LogPath2, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);


				//! here in an iteration loop, obstacle poses are random
				//! 6- randomly assign the obstacle positions
				//					for(int iter=1;iter<11;iter++)
				for(int iter=1;iter<2;iter++)
				{

					serviceMSG_rrt_point.request.Obstacles.clear();
					serviceMSG_rrt_volume.request.Obstacles.clear();
					if(serviceMSG_rrt_point.request.Obstacles.empty() && serviceMSG_rrt_volume.request.Obstacles.empty() )
					{

						double obsSizeVec[]={(double)obsSize,(double)obsSize,(double)obsSize};
						double obsCenter[3];
						double rootcart[]={-15.0,-15.0,-15.0};
						double goalcenter[]={15.0,15.0,15.0};
						double goalsize[]={2.0,2.0,2.0};
						double goalsizefindObstacles[]={2.0+robotSize,2.0+robotSize,2.0+robotSize}; // in case the robot has a size >0, the obstacles should not go the bigger region!

						ofstream ws_Info_File_point,ws_Info_File_volume;
						ofstream traj_Info_File_point,traj_Info_File_volume;



						ws_Info_File_point.open (((string)LogPath+"/"+to_string(counter)+"_robotSize_"+to_string(robotSize)+"_noObs_"+
								to_string(NoObs)+"_obsSize_"+ to_string(obsSize)+"/"+to_string(iter)+"_ws_INFO_point.txt").c_str(),ios::app);

						ws_Info_File_volume.open (((string)LogPath+"/"+to_string(counter)+"_robotSize_"+to_string(robotSize)+"_noObs_"+
								to_string(NoObs)+"_obsSize_"+ to_string(obsSize)+"/"+to_string(iter)+"_ws_INFO_volume.txt").c_str(),ios::app);

						//
						traj_Info_File_point.open (((string)LogPath+"/"+to_string(counter)+"_robotSize_"+to_string(robotSize)+"_noObs_"+
								to_string(NoObs)+"_obsSize_"+ to_string(obsSize)+"/"+to_string(iter)+"_traj_INFO_point.txt").c_str(),ios::app);

						traj_Info_File_volume.open (((string)LogPath+"/"+to_string(counter)+"_robotSize_"+to_string(robotSize)+"_noObs_"+
								to_string(NoObs)+"_obsSize_"+ to_string(obsSize)+"/"+to_string(iter)+"_traj_INFO_volume.txt").c_str(),ios::app);
						//

						// ws info
						ws_Info_File_point <<0.0<<" "<<0.0<<" "<<0.0<<" "<<40.0<<" "<<40.0<<" "<<40.0<<"\n";
						ws_Info_File_volume <<0.0<<" "<<0.0<<" "<<0.0<<" "<<40.0<<" "<<40.0<<" "<<40.0<<"\n";

						//goal info
						ws_Info_File_point  <<goalcenter[0]<<" "<<goalcenter[1]<<" "<<goalcenter[2]<<" "<<goalsize[0]<<" "<<goalsize[1]<<" "<<goalsize[2]<<"\n";
						ws_Info_File_volume <<goalcenter[0]<<" "<<goalcenter[1]<<" "<<goalcenter[2]<<" "<<goalsize[0]<<" "<<goalsize[1]<<" "<<goalsize[2]<<"\n";

						// initial pose info
						ws_Info_File_point  <<rootcart[0]<<" "<<rootcart[1]<<" "<<rootcart[2]<<"\n";
						ws_Info_File_volume <<rootcart[0]<<" "<<rootcart[1]<<" "<<rootcart[2]<<" "<<robotSize<<" "<<robotSize<<" "<<robotSize<<"\n";


						for(int i=0;i<NoObs;i++)
						{
							rrtstar_msgs_point::Region  obstacle_point;
							rrtstar_msgs_volume::Region obstacle_volume;

							//! 5- Define the obstacle sizes, save the ws information in a file inside a folder
							obstacle_volume.size_x= obsSize;
							obstacle_volume.size_y= obsSize;
							obstacle_volume.size_z= obsSize;

							obstacle_point.size_x= obsSize+robotSize;
							obstacle_point.size_y= obsSize+robotSize;
							obstacle_point.size_z= obsSize+robotSize;

							randObstacleCenterGenerator (obsSizeVec, obsCenter,rootcart,goalcenter,goalsizefindObstacles);

							obstacle_volume.center_x= obsCenter[0];
							obstacle_volume.center_y= obsCenter[1];
							obstacle_volume.center_z= obsCenter[2];

							obstacle_point.center_x= obsCenter[0];
							obstacle_point.center_y= obsCenter[1];
							obstacle_point.center_z= obsCenter[2];

							serviceMSG_rrt_point.request.Obstacles.push_back(obstacle_point) ;
							serviceMSG_rrt_volume.request.Obstacles.push_back(obstacle_volume);

							// Add obstacles info
							ws_Info_File_point  <<obstacle_point.center_x<<" "<<obstacle_point.center_y<<" "<<obstacle_point.center_z<<" "<<obstacle_point.size_x<<" "<<obstacle_point.size_y<<" "<<obstacle_point.size_z;
							ws_Info_File_volume <<obstacle_point.center_x<<" "<<obstacle_point.center_y<<" "<<obstacle_point.center_z<<" "<<obstacle_point.size_x<<" "<<obstacle_point.size_y<<" "<<obstacle_point.size_z;
							if(i<(NoObs-1))
							{
								ws_Info_File_point  <<"\n";
								ws_Info_File_volume <<"\n";
							}
						}
						ws_Info_File_point.close();
						ws_Info_File_volume.close();

						//! 7- Send Command the rrt* method, measure the time, save: trajectory data, success/failure, time of the experiment
						timeBeforeCall=ros::Time::now().toSec();

						if( rrtStar_point_SRV_client.call(serviceMSG_rrt_point))
						{

							timeAfterCall=ros::Time::now().toSec();
							traj_Info_File_point<<to_string(timeAfterCall-timeBeforeCall);

							cout<<"**************** rrtStar-point  ****************"<<endl;
							cout<<"time:"<<to_string(timeAfterCall-timeBeforeCall)<<", way points:"<<endl;
							for(int i=0;i<serviceMSG_rrt_point.response.path.size();i++)
							{
								double wayPoint[3];
								wayPoint[0]=serviceMSG_rrt_point.response.path[i].x;
								wayPoint[1]=serviceMSG_rrt_point.response.path[i].y;
								wayPoint[2]=serviceMSG_rrt_point.response.path[i].z;
								cout<< wayPoint[0]<<" "<<wayPoint[1]<<" "<<wayPoint[2]<<endl;
								traj_Info_File_point<<"\n"<<wayPoint[0]<<" "<<wayPoint[1]<<" "<<wayPoint[2];
							}
						}
						traj_Info_File_point.close();

						//! 8- Send Command the extended-rrt* method, measure the time, save: trajectory data, success/failure, time of the experiment

						timeBeforeCall=ros::Time::now().toSec();

						if( rrtStar_volume_SRV_client.call(serviceMSG_rrt_volume))
						{

							timeAfterCall=ros::Time::now().toSec();
							traj_Info_File_volume<<to_string(timeAfterCall-timeBeforeCall);
							cout<<"**************** rrtStar-volume  ****************"<<endl;
							cout<<"time:"<<to_string(timeAfterCall-timeBeforeCall)<<", way points:"<<endl;
							for(int i=0;i<serviceMSG_rrt_volume.response.path.size();i++)
							{
								double wayPoint[3];
								wayPoint[0]=serviceMSG_rrt_volume.response.path[i].x;
								wayPoint[1]=serviceMSG_rrt_volume.response.path[i].y;
								wayPoint[2]=serviceMSG_rrt_volume.response.path[i].z;
								cout<< wayPoint[0]<<" "<<wayPoint[1]<<" "<<wayPoint[2]<<endl;
								traj_Info_File_volume<<"\n"<<wayPoint[0]<<" "<<wayPoint[1]<<" "<<wayPoint[2];
							}
						}
						traj_Info_File_volume.close();
					}

				}
				counter++;
				cout<<counter<<endl;
			}

		}

	}


	return 1;
}

void randObstacleCenterGenerator (double *obsSize, double *obsCenter,const double *rootcart,const double *goalcenter,const double *goalsize){

	// 3: working space dimension
	// 10.0: is the working space size in positive direction
	double ws_size=40.0;
	double MAX_centerLim[3]={ws_size/2.0-obsSize[0]/2.0,ws_size/2.0-obsSize[1]/2.0,ws_size/2.0-obsSize[2]/2.0};
	bool obsVerified[3]={false,false,false};
	// Generate random center for obstacle
	// 10.0: is the working space size in positive direction

	bool collisionFound= true;
	do{
		for (int i=0;i<3;i++){ // for each space dimension

			obsCenter[i]=((double)rand()/(RAND_MAX/2.0)-1.0)*MAX_centerLim[i];
		}

		for(int i=0;i<3;i++)
		{
			if (fabs(obsCenter[i]- rootcart[i]) > obsSize[i]/2.0 ) {
				collisionFound= false;
				break;
			}
		}
		if(collisionFound==false)// if with the root does not have collision, check with goal region
		{
			collisionFound=true;
			for(int i=0;i<3;i++)
			{
				if (fabs(obsCenter[i]- goalcenter[i]) > (obsSize[i]/2.0 + goalsize[i]/2.0) )
				{
					collisionFound= false;
					break;
				}
			}
		}

	}while (collisionFound==true);


	return;
};

void sig_handler(int sig) {
	printf("killing process %d\n",getpid());
	exit(0);
};
