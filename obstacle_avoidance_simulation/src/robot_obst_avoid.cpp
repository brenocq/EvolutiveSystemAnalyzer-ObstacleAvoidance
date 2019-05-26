#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "gazebo_msgs/SetModelState.h"
#include "obstacle_avoidance_simulation/RobotInfo.h"
#include "obstacle_avoidance_simulation/ControlSimulation.h"
#include "obstacle_avoidance_simulation/EvolutiveSystemConfiguration.h"
#include <sstream>
#include <string>
using namespace std;

#define PI 3.14159265
#define NumRobots 6
#define PopulationTestTime 60 //set the time in seconds that each population will run
#define InitialFitness 0
#define WalkingPoints 100 // Maximum number of points that a robots can receive in a second (walking with 1m/s)
#define CollisionPoints -50
//----- On/Off Debug (output) -----//
#define ChromosomeDebug 1
#define FitnessEndCycle 1
#define FitnessFollowDebug 0
#define ColisionMovementState 1
#define InitialPositionDebug 1
#define LaserDebug 0
//----- MATLAB parameters configuration -----//
// ControlSimulation
int SimulationStatus = 0;
bool updateStatus = true;
// Fixed Genes
float FixedSensorActivation = -1;
float FixedLinearVelocity = -1;
float FixedAngularVelocity = -1;
float FixedRotationTime = -1;
float FixedSensorAngle = -1;
// Types of Pressure
bool ControlBackMutationPrevention = 1;
int ControlPredation = 10;
// Types of Diversity
float ControlMutation = 10;
float ControlNeutralization = 50;
bool ControlCrossing = 1;

//----- Create Publishers and Subscribers -----//
ros::Publisher velocity_publisher[NumRobots+1];//{0, robot1, robot2}
ros::Publisher robotInfo_publisher;
ros::Publisher evolutive_config_publisher;
ros::Publisher control_simulation_publisher;
ros::Subscriber laser_subscriber[NumRobots+1];//{0, robot1, robot2}
ros::Subscriber evolutive_config_subscriber;
ros::Subscriber control_simulation_subscriber;
ros::ServiceClient change_robot_position;
ros::ServiceClient resetSimulation;
ros::ServiceClient runSimulation;
ros::ServiceClient pauseSimulation;

//----- Robot variables -----//
float laserData[NumRobots+1][3] = {0};//three laser sensors per robot
bool laser1BitData[NumRobots+1][3] = {0};//three 2bit laser sensor per robot
double stopRotation[NumRobots+1] = {0};//it stores the time to stop the rotation (thread)
bool dirRotation[NumRobots+1] = {0};//it stores the rotation direction (clockwise or counterclockwise)
bool BMP[NumRobots+1][5];

//----- Evolutive System variables -----//
int populationNumber=1;
double chromosome[NumRobots+1][5];// {sensorActivation, vel_linear, vel_angular, time_rotation, sensor_angle}
int fitness[NumRobots+1][5];
//--- Used to ajust the fitness ---//
bool inCollision[NumRobots+1];
bool walking[NumRobots+1];
//--- Auxiliary ---//
double populationCurrentTime;
double populationInitTime=0;
int lastUpdateFitness=0;//time in seconds


//----- Robot movement and reading -----//
void move(double speed, int robotNumber);
void rotate(double angular_speed, bool clockwise, int robotNumber);
double degree2radians(double degree);
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
void evolutiveConfigCallBack(const obstacle_avoidance_simulation::EvolutiveSystemConfiguration::ConstPtr &msg);
void controlSimulationCallBack(const obstacle_avoidance_simulation::ControlSimulation::ConstPtr &msg);
void controlLoop();

//----- Evolutive System -----//
void initiatePopulation();
void resetFitness();
void updateFitness();
bool endPopulationTime();
void generateNewPopulation();
void publishInfo();
void randomPositions();

//----- MAIN -----//
int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_obst_avoid");
    ros::NodeHandle n;

    //Create robot topics and subscribe to the LIDAR
    for (int i = 1; i <= NumRobots; i++) {
      string robotNameSpace;
      if(i<10){
        robotNameSpace = "/robotX";
        robotNameSpace[6]='0'+i;
      }
      else{
        robotNameSpace = "/robotXX";
        robotNameSpace[6]='0'+ int(i/10);
        robotNameSpace[7]='0'+(i%10);
      }


      velocity_publisher[i] = n.advertise<geometry_msgs::Twist>(robotNameSpace+"/cmd_vel",10);
      laser_subscriber[i] = n.subscribe(robotNameSpace+"/scan", 10, laserCallBack);
    }

    //Topics to communicate with MATLAB
    evolutive_config_publisher = n.advertise<obstacle_avoidance_simulation::EvolutiveSystemConfiguration>("/evolutive_config",10);
    evolutive_config_subscriber = n.subscribe("evolutive_config", 10, evolutiveConfigCallBack);
    control_simulation_publisher = n.advertise<obstacle_avoidance_simulation::ControlSimulation>("/control_simulation",10);
    control_simulation_subscriber = n.subscribe("control_simulation", 10, controlSimulationCallBack);

    robotInfo_publisher = n.advertise<obstacle_avoidance_simulation::RobotInfo>("/simulation_info",10);

    ros::ServiceClient resetSimulation = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
    ros::ServiceClient change_robot_position = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient runSimulation = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    ros::ServiceClient pauseSimulation = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    ros::Rate loop_rate(100);

    std_srvs::Empty srv;
    resetSimulation.call(srv);
    initiatePopulation();
    //randomPositions();
    while (ros::ok())
    {
      if(updateStatus){
        updateStatus=false;
        std_srvs::Empty srv;
        switch(SimulationStatus){
          case 0:
            runSimulation.call(srv);
            resetSimulation.call(srv);
            pauseSimulation.call(srv);
          break;
          case 1:
            runSimulation.call(srv);
          break;
          case 2:
            pauseSimulation.call(srv);
          break;
        }
      }

      //randomPositions();
      /*geometry_msgs::Pose start_pose;
      start_pose.position.x = 2;
      start_pose.position.y = 2;
      start_pose.position.z = 2;
      start_pose.orientation.x = 0.0;
      start_pose.orientation.y = 0.0;
      start_pose.orientation.z = 0.0;
      start_pose.orientation.w = 0.0;

      geometry_msgs::Twist start_twist;
      start_twist.linear.x = 0.0;
      start_twist.linear.y = 0.0;
      start_twist.linear.z = 0.0;
      start_twist.angular.x = 0.0;
      start_twist.angular.y = 0.0;
      start_twist.angular.z = 1;

      gazebo_msgs::ModelState modelstate;
      string robotName = "Robot3";

      modelstate.model_name = robotName;
      modelstate.reference_frame = "";
      //modelstate.reference_frame = (std::string) "world";
      modelstate.pose = start_pose;
      modelstate.twist = start_twist;

      //ros::ServiceClient client = n.serviceClient<gazebo::SetModelState>("/gazebo/set_model_state");
      gazebo_msgs::SetModelState setmodelstate;
      setmodelstate.request.model_state = modelstate;
      change_robot_position.call(setmodelstate);*/

      if(SimulationStatus==1){
        loop_rate.sleep();
        controlLoop();

        if(int(ros::Time::now().toSec())>=(lastUpdateFitness+1)){//Update each 1 second
          cout<<"UpdateFitness"<<endl;
          lastUpdateFitness=int(ros::Time::now().toSec());
          updateFitness();
        }

        if(endPopulationTime()){
          publishInfo();

          generateNewPopulation();
          resetFitness();

          std_srvs::Empty srv;
          resetSimulation.call(srv);
          populationNumber++;
          for (int i = 1; i <=NumRobots; i++) {
                stopRotation[i]=0;
          }
          {
            float initialPositions[NumRobots+1][2];
            float initialAngle[NumRobots+1];
            int spawnPoints=4;
            float possiblePositionsXY[spawnPoints]={-1.5,-0.5,0.5,1.5};//Possible locations to spawn a robot

            if(InitialPositionDebug)
            cout<<endl<<"Initial Positions:"<<endl;

            for(int i = 1; i <= NumRobots; i++){//Generate random positions and angles

                newPosition:
                initialPositions[i][0]=possiblePositionsXY[rand()%spawnPoints];
                initialPositions[i][1]=possiblePositionsXY[rand()%spawnPoints];
                for(int j = 1; j <= NumRobots; j++){
                  if(j!=i){
                    if(initialPositions[j][0]==initialPositions[i][0] && initialPositions[j][1]==initialPositions[i][1])
                      goto newPosition;
                  }
                }
                initialAngle[i]=float(rand()%360)/180*PI;

            }


            for (int i = 1; i <= NumRobots; i++) {
              geometry_msgs::Pose start_pose;
              start_pose.position.x = initialPositions[i][0];
              start_pose.position.y = initialPositions[i][1];
              start_pose.position.z = 0.05;
              start_pose.orientation.x = 0.0;
              start_pose.orientation.y = 0.0;
              start_pose.orientation.z = 0.0;
              start_pose.orientation.w = 0.0;

              geometry_msgs::Twist start_twist;
              start_twist.linear.x = 0.0;
              start_twist.linear.y = 0.0;
              start_twist.linear.z = 0.0;
              start_twist.angular.x = 0.0;
              start_twist.angular.y = 0.0;
              start_twist.angular.z = initialAngle[i];

              gazebo_msgs::ModelState modelstate;
              string robotName;
              if(i<10){
                robotName = "RobotX";
                robotName[5]='0'+i;
              }
              else{
                robotName = "RobotXX";
                robotName[5]='0'+ int(i/10);
                robotName[6]='0'+(i%10);
              }
              modelstate.model_name = robotName;
              modelstate.reference_frame = "";
              //modelstate.reference_frame = (std::string) "world";
              modelstate.pose = start_pose;
              modelstate.twist = start_twist;

              //ros::ServiceClient client = n.serviceClient<gazebo::SetModelState>("/gazebo/set_model_state");
              gazebo_msgs::SetModelState setmodelstate;
              setmodelstate.request.model_state = modelstate;
              change_robot_position.call(setmodelstate);

            }
          }

          lastUpdateFitness=0;
          populationInitTime=0;
        }

      }
      ros::spinOnce();
    }
    return 0;
  }

//----- Robot movement and reading -----//
void controlLoop(){

  for (int i = 1; i <= NumRobots; i++){
    //printf("Robot %d: time: %lf rotation: %lf\n",i,ros::Time::now().toSec(),stopRotation[i]);
    if(ros::Time::now().toSec()>stopRotation[i]){
      if(laser1BitData[i][0]){
        stopRotation[i]=(double)ros::Time::now().toSec()+chromosome[i][3];
        dirRotation[i]=false;
        //cout<<"Robot"<<i<<" stopRotation: "<<stopRotation[i]<<" Time now: "<< (double)ros::Time::now().toSec() <<endl;
        rotate(degree2radians(chromosome[i][2]), dirRotation[i], i);
      }
      else if(laser1BitData[i][1]){
          stopRotation[i]=(double)ros::Time::now().toSec()+chromosome[i][3];
          dirRotation[i]=true;
          //cout<<"Robot"<<i<<" stopRotation: "<<stopRotation[i]<<" Time now: "<< ros::Time::now().toSec() <<endl;
          rotate(degree2radians(chromosome[i][2]), dirRotation[i], i);
      }else if(laser1BitData[i][2]){
          stopRotation[i]=(double)ros::Time::now().toSec()+chromosome[i][3];
          dirRotation[i]=true;
          //cout<<"Robot"<<i<<" stopRotation: "<<stopRotation[i]<<" Time now: "<< (double)ros::Time::now().toSec() <<endl;
          rotate(degree2radians(chromosome[i][2]), dirRotation[i], i);
      }else{
        move(chromosome[i][1],i);
      }

    }
  }
}
void move(double speed, int robotNumber){
    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x = speed;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    double now =  ros::Time::now().toSec();
    if(now>stopRotation[robotNumber]){
      velocity_publisher[robotNumber].publish(vel_msg);
    }
    walking[robotNumber]=true;
  }
void rotate(double angular_speed, bool clockwise, int robotNumber){
  geometry_msgs::Twist vel_msg;

  vel_msg.linear.x = 0;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;

  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;

  if(clockwise)
    vel_msg.angular.z = -abs(angular_speed);
  else
    vel_msg.angular.z = abs(angular_speed);

  ros::Rate loop_rate(100);

  double now =  ros::Time::now().toSec();

  if(now<=stopRotation[robotNumber]){
    velocity_publisher[robotNumber].publish(vel_msg);
    walking[robotNumber]=false;
  }
}
double degree2radians(double degree){
  return degree/180*PI;
}
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr &msg){
  const int delta = 5; //Number of degrees to each sensor
  float process[delta]; //Data from each degree
  int robotNumber = msg->header.frame_id[5]-'0';// msg->header.frame_id == "robotX_tf/base_scan"
  uint16_t laserAngle[3] = {chromosome[robotNumber][4], 0, 360-chromosome[robotNumber][4]};

  if(LaserDebug){
  cout<<"---Laser Data ("<<robotNumber<<")---"<<endl;
  cout<<"Laser activation distance: "<< chromosome[robotNumber][0] <<"m"<<endl;
  }

  for (int num = 0; num < 3; num++)
  {
    if(LaserDebug){
    cout<<"Laser "<< laserAngle[num] <<" degrees: "<<endl;
    }

    for(int i=laserAngle[num]-(delta/2); i<=laserAngle[num]+(delta/2);i++){
      int degree=i;
      degree>=360?degree-=360:degree;
      degree<0?degree+=360:degree;

      if(std::isinf(msg->ranges.at(degree)))
      {
        process[i-(laserAngle[num]-(delta/2))] = msg->range_max;
      }
      else
      {
        process[i-(laserAngle[num]-(delta/2))] = msg->ranges.at(degree);
      }
      if(LaserDebug){
      cout<<"\t"<<degree<<" degrees: "<<process[i-(laserAngle[num]-(delta/2))]<<"m"<<endl;
      }
    }
    laserData[robotNumber][num]=0;
    for (int i = 0; i < delta; i++) {
      laserData[robotNumber][num]+=process[i];
    }
    laserData[robotNumber][num]/=delta;

    if(laserData[robotNumber][num]<=chromosome[robotNumber][0])
      laser1BitData[robotNumber][num]=true;
    else
      laser1BitData[robotNumber][num]=false;

    if(LaserDebug){
      cout<<"\tR(media): "<<laserData[robotNumber][num]<<"m"<<endl;
      cout<<"\t2Bit: "<<laser1BitData[robotNumber][num]<<endl;
    }
  }
  //----- Check Collision -----//
  const int degreePrecision = 5;
  const float collisionDistance = 0.15;//radius
  inCollision[robotNumber]=false;
  for (int i = 0; i < 360; i+=degreePrecision) {

    if(!std::isinf(msg->ranges.at(i)))
    {
      if(msg->ranges.at(i)<=collisionDistance){
          //cout<<"Robot("<<robotNumber<<"): COLLISION " << msg->ranges.at(i)<<"m"<<endl;
          inCollision[robotNumber]=true;
      }
    }
  }
}
void evolutiveConfigCallBack(const obstacle_avoidance_simulation::EvolutiveSystemConfiguration::ConstPtr &msg){
  FixedSensorActivation = msg->SensorActivation;
  FixedLinearVelocity = msg->LinearVelocity;
  FixedAngularVelocity = msg->AngularVelocity;
  FixedRotationTime = msg->RotationTime;
  FixedSensorAngle = msg->SensorAngle;
  // Types of Pressure
  ControlBackMutationPrevention = msg->BackMutationPrevention;
  ControlPredation = msg->Predation;
  // Types of Diversity
  ControlMutation = msg->Mutation;;
  ControlNeutralization = msg->Neutralization;
  ControlCrossing = msg->Crossing;
}
void controlSimulationCallBack(const obstacle_avoidance_simulation::ControlSimulation::ConstPtr &msg){
  SimulationStatus=msg->SimulationState;
  updateStatus=true;
}
//----- Evolutive System -----//
void initiatePopulation(){
  srand (time(NULL));

  cout<<"----------- POPULATION "<<populationNumber<<" -----------"<<endl;


  for (int i = 1; i <= NumRobots; i++) {
    chromosome[i][0] = double(rand() % 35)/10;//sensorActivation    [0 - 3.5]
    chromosome[i][1] = double(rand() % 100)/100;//vel_linear        [0.00 - 1.00]
    chromosome[i][2] = (rand() % 85)+5;//vel_angular                [5 - 90]
    chromosome[i][3] = double(rand() % 50)/10;//time_rotation       [0 - 5]
    chromosome[i][4] = double(rand() % 90);//sensor_angle           [0 - 90]
    resetFitness();

    if(ChromosomeDebug){
      cout<<"Robot "<<i<<":"<<endl;
      cout<<"\t"<<"Sensor Activation: "<<chromosome[i][0]<<"m"<<endl;
      cout<<"\t"<<"Vel Linear: "<<chromosome[i][1]<<"m/s"<<endl;
      cout<<"\t"<<"Vel Angular: "<<chromosome[i][2]<<"degree/s"<<endl;
      cout<<"\t"<<"Time Rotation: "<<chromosome[i][3]<<"s"<<endl;
      cout<<"\t"<<"Sensor Angle: ["<<chromosome[i][4]<<",0,"<<360-chromosome[i][4]<<"] (degree)"<<endl;
    }
  }
}

void resetFitness(){
  for (int i = 1; i <= NumRobots; i++)
    fitness[i][4]=InitialFitness;
}

void updateFitness(){
  if(FitnessFollowDebug)
    cout<<"Population "<<populationNumber+1<<":"<<endl;

  if(ColisionMovementState){
    cout<<"collisions=[";
    for (int i = 1; i <= NumRobots; i++) {
      cout<<inCollision[i];
      if(i<NumRobots)
        cout<<",";
    }
    cout<<"]";

    cout<<" movement=[";
    for (int i = 1; i <= NumRobots; i++) {
      cout<<walking[i];
      if(i<NumRobots)
        cout<<",";
    }
    cout<<"]"<<endl;
  }

  for (int i = 1; i <= NumRobots; i++) {
    if(walking[i]==true && inCollision[i]==false){//o robô andava mesmo colidindo e estava ganhando muitos pontos com isso
      fitness[i][4]+=chromosome[i][1]*WalkingPoints; //points = vel_linear*WalkingPoints
    }
    if(inCollision[i]==true){
      fitness[i][4]+=CollisionPoints; //points = CollisionPoints
    }
    if(FitnessFollowDebug){
      cout<<"\t"<<"Robot"<<i<<": ";
      for (size_t j = 0; j < 5; j++) {
          cout<<fitness[i][j]<<" ";
      }
      cout<<endl;
    }

  }
}

bool endPopulationTime(){
  if(populationInitTime==0){
    populationInitTime=ros::Time::now().toSec();
  }
  populationCurrentTime=ros::Time::now().toSec()-populationInitTime;

  if(populationCurrentTime>=PopulationTestTime){
    return true;
  }else{
    return false;
  }
}

void generateNewPopulation(){
  int bestRobot=0, bestRobotFitness=0, totalFitness=0;
  int averageFitness[NumRobots+1];

  char showMutation[NumRobots+1][18];
  for(int i = 1; i <= NumRobots; i++){
    for (size_t j = 0; j < 4; j++) {
      showMutation[i][j]=' ';
    }
  }

  for (size_t i = 1; i <= NumRobots; i++) {

    averageFitness[i]=0;
    for (size_t j = 0; j < 5; j++) {
      averageFitness[i]+=fitness[i][j];
    }
    if(populationNumber>=5){
      averageFitness[i]/=5;
    }else{
      averageFitness[i]/=populationNumber;
    }
  }

  for (int i = 1; i <= NumRobots; i++){
    if(averageFitness[i]>bestRobotFitness){
      bestRobot=i;
      bestRobotFitness=averageFitness[i];
    }
    totalFitness+=fitness[i][4];
  }

  if(FitnessEndCycle){
    cout<<endl<<"Fitness Population End Cycle:"<<endl;
    cout<<"\tMaximum Fitness: "<<InitialFitness+(PopulationTestTime)*WalkingPoints*0.5<<endl;
    cout<<"\tMinimum Fitness: "<<InitialFitness+(PopulationTestTime)*CollisionPoints<<endl;
    cout<<"\tSum Fitness: "<<totalFitness<<endl;
    cout<<"\tAverage Fitness: "<<totalFitness/NumRobots<<endl;
    cout<<"\tRobot "<<bestRobot<<" was the best robot with fitness equal to "<<bestRobotFitness<<endl<<endl;
    cout<<"\tRobots Fitness:"<<endl;
    for (int i = 1; i <= NumRobots; i++){
      cout<<"\t\tRobot "<<i<<": ";
      for (size_t j = 0; j < 5; j++) {
          cout<<fitness[i][j]<<" ";
      }
      cout<<"avr("<<averageFitness[i]<<")"<<endl;
    }
  }

  for (int i = 1; i <= NumRobots; i++){
    if(fitness[i][4]<=fitness[bestRobot][4]){
      if(ControlCrossing==1){
        for (int j = 0; j < 5; j++){
          chromosome[i][j] = (chromosome[i][j]+chromosome[bestRobot][j])/2;
        }
      }

      if(ControlBackMutationPrevention==1){
          for (int j = 0; j < 5; j++){
            BMP[i][j]=0;
          }
      }
      //----- Mutation -----//
      if(ControlMutation!=-1){
        for (int j = 0; j < 5; j++) {
          bool mutation = (rand()%100)<=ControlMutation;

          if(mutation && BMP[i][j]==0){
            BMP[i][j]=1;

            int checkBMP=0;
            for (int k = 0; k < 5; k++) {
              BMP[i][k]==1?checkBMP++:checkBMP;
            }
            if(checkBMP==5){
              for (int k = 0; k < 5; k++) {
                BMP[i][k]=0;
              }
            }

            double neut=0;
            if(ControlNeutralization!=-1){
              neut = double(ControlNeutralization)/100;
            }else{
              neut = 1;
            }

            showMutation[i][j]='*';
            switch(j){
              case 0:
                chromosome[i][0] = ( (1-neut)*chromosome[i][0] )+ ( (neut)*(double(rand() % 35)/10) );
              break;
              case 1:
                chromosome[i][1] = ( (1-neut)*chromosome[i][1]) + ( (neut)*(double(rand() % 100)/100) );
              break;
              case 2:
                chromosome[i][2] = ( (1-neut)*chromosome[i][2]) + ( (neut)*((rand() % 85)+5) );
              break;
              case 3:
                chromosome[i][3] = ( (1-neut)*chromosome[i][3]) + ( (neut)*(double(rand() % 50)/10) );
              break;
              case 4:
                chromosome[i][4] = ( (1-neut)*chromosome[i][4]) + ( (neut)*(double(rand() % 90)) );
              break;
              default:
              break;
            }
          }
        }
      }
    }
  }
  //----- Predator -----//
  int newRobot=0;
  if(ControlPredation!=-1){
    if((populationNumber+1)%ControlPredation==0){
      int worstRobot=1, worstRobotFitness=0;
      worstRobotFitness =  averageFitness[1];
      for (int i = 1; i <= NumRobots; i++){
        if(averageFitness[i] <= worstRobotFitness){
          worstRobot=i;
          worstRobotFitness=averageFitness[i];
        }
      }
      if(FitnessEndCycle){
        cout<<"\tROBO "<<worstRobot<<" FOI MORTO PELO PREDADOR!"<<endl;
      }

      chromosome[worstRobot][0] = double(rand() % 35)/10;//sensorActivation    [0 - 3.5]
      chromosome[worstRobot][1] = double(rand() % 48)/100 + 0.02;//vel_linear  [0.02 - 0.50]
      chromosome[worstRobot][2] = (rand() % 85)+5;//vel_angular                [5 - 90]
      chromosome[worstRobot][3] = double(rand() % 50)/10;//time_rotation       [0 - 5]
      chromosome[worstRobot][4] = double(rand() % 90);//sensor_angle       [0 - 5]

      newRobot=worstRobot;
    }
  }
  //----- Update average vector fitness -----//
  for (size_t i = 1; i <= NumRobots; i++) {
    for (size_t j = 1; j < 5; j++) {
      fitness[i][j-1]=fitness[i][j];
    }
  }

  for (int i = 0; i <= NumRobots; i++) {
    if(FixedSensorActivation!=-1){
      chromosome[i][0]=FixedSensorActivation;
      showMutation[i][0]='-';
    }
    if(FixedLinearVelocity!=-1){
      chromosome[i][1]=FixedLinearVelocity;
      showMutation[i][1]='-';
    }
    if(FixedAngularVelocity!=-1){
      chromosome[i][2]=FixedAngularVelocity;
      showMutation[i][2]='-';
    }
    if(FixedRotationTime!=-1){
      chromosome[i][3]=FixedRotationTime;
      showMutation[i][3]='-';
    }
    if(FixedSensorAngle!=-1){
      chromosome[i][4]=FixedSensorAngle;
      showMutation[i][4]='-';
    }
  }


  if(ChromosomeDebug){
    cout<<endl<<"----------- POPULATION "<<populationNumber+1<<" -----------"<<endl;
    for (int i = 1; i <= NumRobots; i++){
      cout<<"Robot"<<i;
      if(i==newRobot)
        cout<<"(new)";
      cout<<" :"<<endl;
      cout<<"\t"<<"Sensor Activation: "<<chromosome[i][0]<<"m"<<  showMutation[i][0]<<endl;
      cout<<"\t"<<"Vel Linear: "<<chromosome[i][1]<<"m/s"<<       showMutation[i][1]<<endl;
      cout<<"\t"<<"Vel Angular: "<<chromosome[i][2]<<"degree/s"<< showMutation[i][2]<<endl;
      cout<<"\t"<<"Time Rotation: "<<chromosome[i][3]<<"s"<<      showMutation[i][3]<<endl;
      cout<<"\t"<<"Sensor Angle: ["<<chromosome[i][4]<<",0,"<<360-chromosome[i][4]<<"] (degree)"<<endl;
    }
  }
}

void publishInfo(){
  ros::Rate loop_rate(10);
  obstacle_avoidance_simulation::RobotInfo infoPub;
  for (size_t i = 1; i <= NumRobots; i++) {

    infoPub.RobotNumber.push_back(i);
    infoPub.Fitness.push_back(fitness[i][4]);
    infoPub.Generation = populationNumber;
    infoPub.SensorActivation.push_back(chromosome[i][0]);
    infoPub.LinearVelocity.push_back(chromosome[i][1]);
    infoPub.AngularVelocity.push_back(chromosome[i][2]);
    infoPub.RotationTime.push_back(chromosome[i][3]);
    infoPub.SensorAngle.push_back(chromosome[i][4]);
    infoPub.Mutation.push_back(1);
  }
    robotInfo_publisher.publish(infoPub);

}

void randomPositions(){

  /*float initialPositions[NumRobots+1][2];
  float initialAngle[NumRobots+1];
  int spawnPoints=6;
  float possiblePositionsXY[spawnPoints]={-2.5,-1.5,-0.5,0.5,1.5,2.5};//Possible locations to spawn a robot

  if(InitialPositionDebug)
  cout<<endl<<"Initial Positions:"<<endl;

  for(int i = 1; i <= NumRobots; i++){//Generate random positions and angles

      newPosition:
      initialPositions[i][0]=possiblePositionsXY[rand()%spawnPoints];
      initialPositions[i][1]=possiblePositionsXY[rand()%spawnPoints];
      for(int j = 1; j <= NumRobots; j++){
        if(j!=i){
          if(initialPositions[j][0]==initialPositions[i][0] && initialPositions[j][1]==initialPositions[i][1])
            goto newPosition;
        }
      }
      initialAngle[i]=float(rand()%360)/180*PI;

  }


  for (int i = 1; i <= NumRobots; i++) {
    geometry_msgs::Pose start_pose;
    start_pose.position.x = initialPositions[i][0];
    start_pose.position.y = initialPositions[i][1];
    start_pose.position.z = 0.05;
    start_pose.orientation.x = 0.0;
    start_pose.orientation.y = 0.0;
    start_pose.orientation.z = 0.0;
    start_pose.orientation.w = 0.0;

    geometry_msgs::Twist start_twist;
    start_twist.linear.x = 0.0;
    start_twist.linear.y = 0.0;
    start_twist.linear.z = 0.0;
    start_twist.angular.x = 0.0;
    start_twist.angular.y = 0.0;
    start_twist.angular.z = initialAngle[i];

    gazebo_msgs::ModelState modelstate;
    string robotName;
    if(i<10){
      robotName = "RobotX";
      robotName[5]='0'+i;
    }
    else{
      robotName = "RobotXX";
      robotName[5]='0'+ int(i/10);
      robotName[6]='0'+(i%10);
    }
    modelstate.model_name = robotName;
    modelstate.reference_frame = "";
    //modelstate.reference_frame = (std::string) "world";
    modelstate.pose = start_pose;
    modelstate.twist = start_twist;

    //ros::ServiceClient client = n.serviceClient<gazebo::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState setmodelstate;
    setmodelstate.request.model_state = modelstate;
    change_robot_position.call(setmodelstate);


    if(InitialPositionDebug){
      //cout<<"\t"<<robotName<<": Pos("<<initialPositions[i][0]<<","<<initialPositions[i][1]<<")\tdegree("<< initialAngle[i]*180/PI<<")"<<endl;
      cout<<modelstate<<endl;
    }
  }*/
}
