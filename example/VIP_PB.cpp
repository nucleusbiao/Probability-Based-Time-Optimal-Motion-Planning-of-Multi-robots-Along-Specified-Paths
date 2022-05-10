/**
 * TRUNK
 *
 * EXAMPLE SCENARIO CREATION AND EVALUATION
 *
 * This file demonstrates setting up a simple lane-crossing scenario, and
 * evaluating that scenario to construct path-time obstacles that can be
 * fed into the planner.
 *
 * LIMITATIONS: The API currently only supports scenarios in which
 * the user-controlled vehicle follows a straight-line path, and obstacle
 * vehicles follow straight-line paths that cross the user-controlled vehicle's
 * path perpendicularly; so, scenarios that have the driver driving straight
 * across lanes of traffic. I am working on extending the scenario to handle
 * elliptical as well as linear paths, and arbitrary angles of intersection.
 *
 * SETUP: A scenario consists of a user-controlled vehicle, and a (possibly
 * empty) set of obstacles (other vehicles). These two things are described by
 * the ScenarioUser and ScenarioObstacle classes, respectively.
 *
 * The ScenarioUser class consists of a Vehicle object and a Path object, which
 * respectively describe the characteristics of the user's vehicle (such as
 * length, width, velocity, position, etc.), and the path that the vehicle
 * follows. Because this is the user-controlled vehicle, it also contains
 * a representation of the scenario (the other vehicles and their paths), and
 * and a SCIMP controller which attempts to prevent the user from executing
 * unsafe controls.
 *
 * The ScenarioObstacle class also contains a Vehicle and a Path. Instead of
 * a SCIMP controller, it contains an actual controller that defines how the
 * vehicle is controlled over time. (In this context, control is limited to
 * longitudinal control: acceleration or deceleration.)
 */
#include <fstream>
#include <iostream>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
using boost::math::normal; // typedef provides default type is double.
#include <iomanip>
using std::setw; using std::setprecision;
#include <limits>
using std::numeric_limits;

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
#include <math.h>
#include <yaml-cpp/yaml.h>

#include "timer.hpp"
#include <iostream>
#include <PVTP/Planner.hpp>
#include <PVTP/ScenarioUser.hpp>
#include <PVTP/Utilities.hpp>
// the planner is contained in the PVTP namespace
using namespace PVTP;

// the scenario API, including I/O is contained in the SCIMP_Scenario namespace
using namespace SCIMP_Scenario;

struct Location {
    Location(double x, double y) : x(x), y(y) {}
    double x;
    double y;

    bool operator<(const Location& other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }

    bool operator==(const Location& other) const {
        return std::tie(x, y) == std::tie(other.x, other.y);
    }

    friend std::ostream& operator<<(std::ostream& os, const Location& c) {
        return os << "(" << c.x << "," << c.y << ")";
    }
};

struct State {
    State(double time, double x, double y,double v,double width,double length) :time(time), x(x), y(y),v(v),width(width),length(length){}

    bool operator==(const State& s) const {
        return std::abs(time-s.time)<=0.0001 && std::abs(x-s.x)<=0.0001 && std::abs(y-s.y)<=0.0001;
    }

    bool equal(const State& s) const { return std::abs(x-s.x)<=0.0001 && std::abs(y-s.y)<=0.0001; }

    friend std::ostream& operator<<(std::ostream& os, const State& s) {
        return os << s.time << ": (" << s.x << "," << s.y << ")";
        // return os << "(" << s.x << "," << s.y <<","<< s.StepNum<<")";
    }

    double time;
    double x;
    double y;
    double v;
    double width;
    double length;
};

struct Result {
    std::vector<State> states;
};


void generalEquation(double x1,
                     double y1,
                     double x2,
                     double y2,
                     double &A,
                     double &B,
                     double &C){
    A=y2-y1;
    B=x1-x2;
    C=x2*y1-x1*y2;
}

bool getIntersectPointofLines(double x1,
                              double y1,
                              double x2,
                              double y2,
                              double x3,
                              double y3,
                              double x4,
                              double y4,
                              Location& itersectPoint){
    double A1,B1,C1,A2,B2,C2;
    generalEquation(x1,y1,x2,y2,A1,B1,C1);
    generalEquation(x3,y3,x4,y4,A2,B2,C2);
    double m=A1*B2-A2*B1;
    if(std::abs(m)<=0.0001){
        return false;
    }else{
        Location temp((C2*B1-C1*B2)/m,(C1*A2-C2*A1)/m);
        itersectPoint=temp;
        return true;
    }
}
struct Conflicts {
    double time1;
    double time2;
    size_t agent1;
    size_t agent2;
    double position1;
    double position2;
    double x;
    double y;
    double prob1;
    double prob2;
    friend std::ostream& operator<<(std::ostream& os, const Conflicts& c) {

        return os << c.time1<< ": Vertex(" << c.x << "," << c.y << ")";
    }
};

double getPositionByTime(double time,
                         double timeStep,
                         ScenarioUser robot){
    ScenarioUser temp(robot);
    double user_control = temp.vehicle.getMaximumAcceleration();
    for(double i=0;i<=time&&!temp.pathTraversed();i+=timeStep){
        temp.applyFilteredControl(user_control, timeStep);
    }
    return temp.vehicle.getPosition();
}

double getPositionByTime(double time,
                         double timeStep,
                         std::vector<double>& seq){
    size_t t=static_cast<size_t>(time/timeStep+0.5);
    return seq[t];
}

double getTimeByPosition(double position,
                         double timeStep,
                         ScenarioUser robot){
    ScenarioUser temp(robot);
    double time;
    double user_control = temp.getVehicle().getMaximumAcceleration();
    for(time=0;temp.vehicle.getPosition()<=position&&!temp.pathTraversed();time+=timeStep){
        if(!temp.applyFilteredControl(user_control, timeStep)){
            return -1;
        }
    }
    return time;
}

double getTimeByPosition(double position,
                         double timeStep,
                         std::vector<double>& seq){
    size_t time;
    for(time=0;time<=seq.size()-1;time++){
        if(seq[time]>=position){
            return time*timeStep;
        }
    }
    return seq.back();
}

double calculateConflictProb(double time,
                             double timeStep,
                             double sigma,
                             ScenarioUser robot,
                             double position,
                             double threshold){
    double prob=0;
    double tempPosition=getPositionByTime(time,timeStep,robot);
    normal s(tempPosition,sigma*sqrt(time));
    double i=(position-threshold)>0?position-threshold:0;
    double j=position+threshold;
    prob=cdf(s,j)-cdf(s,i);
    return prob;
}
double calculateConflictProb(double time,
                             double timeStep,
                             double sigma,
                             std::vector<double>& seq,
                             double position,
                             double threshold){
    double prob=0;
    double tempPosition=getPositionByTime(time,timeStep,seq);
    normal s(tempPosition,sigma*sqrt(time));
    double i=(position-threshold)>0?position-threshold:0;
    double j=position+threshold;
    prob=cdf(s,j)-cdf(s,i);
    return prob;
}

double calculateConflictProb(double time,
                             double sigma,
                             double pos,
                             double position,
                             double threshold){
    double prob=0;
    normal s(pos,sigma*sqrt(time));
    double i=(position-threshold)>0?position-threshold:0;
    double j=position+threshold;
    prob=cdf(s,j)-cdf(s,i);
    return prob;
}

bool getFirstConflict(std::vector<State>& startStates,
                      std::vector<Location>& goals,
                      std::vector<ScenarioUser>& robots,
                      double timeStep,
                      Conflicts& result,
                      std::vector<std::vector<double>>& positionSeq,
                      double sigma,
                      double p_) {
    positionSeq.clear();
    for(size_t i=0;i<=startStates.size()-1;i++){
        ScenarioUser temp(robots[i]);
        std::vector<double> seq;
        double user_control = temp.getVehicle().getMaximumAcceleration();
        seq.emplace_back(0.);
        while(!temp.pathTraversed()){
            temp.applyFilteredControl(user_control, timeStep);
            seq.emplace_back(temp.vehicle.getPosition());
        }
        positionSeq.emplace_back(seq);
    }

    // check drive-drive vertex collisions
    for(size_t i=0;i<=startStates.size()-1;i++){
        for(size_t j=i;j<=startStates.size()-1;j++){
            Location itersectPoint(0,0);
            if(getIntersectPointofLines(startStates[i].x,startStates[i].y,goals[i].x,goals[i].y, startStates[j].x,startStates[j].y,goals[j].x,goals[j].y,itersectPoint))
            {
                ScenarioUser robot1(robots[i]);
                ScenarioUser robot2(robots[j]);
                double maxProb1,maxProb2,prob1,prob2;
                //double pathStep=0.1;
                double position1=0.5+sqrt(pow(startStates[i].x-itersectPoint.x,2)+pow(startStates[i].y-itersectPoint.y,2));
                double position2=0.5+sqrt(pow(startStates[j].x-itersectPoint.x,2)+pow(startStates[j].y-itersectPoint.y,2));
                double threshold1=robot1.getVehicle().getLength()+robot2.getVehicle().getWidth()/2;
                double threshold2=robot2.getVehicle().getLength()+robot1.getVehicle().getWidth()/2;
                double maxProb=0;
                double maxTime1=getTimeByPosition(position1,timeStep,positionSeq[i]);
                double maxTime2=getTimeByPosition(position2,timeStep,positionSeq[j]);
                double enterTime1=getTimeByPosition(position1-threshold1,timeStep,positionSeq[i]);
                double enterTime2=getTimeByPosition(position2-threshold2,timeStep,positionSeq[j]);
                double outTime1=getTimeByPosition(position1+threshold1,timeStep,positionSeq[i]);
                double outTime2=getTimeByPosition(position2+threshold2,timeStep,positionSeq[j]);
                if(std::abs(maxTime1-maxTime2)>4) continue;
                for(double time=enterTime1;time<outTime1;time+=timeStep){
                    prob1=calculateConflictProb(time,timeStep,sigma,positionSeq[i],position1,threshold1);
                    prob2=calculateConflictProb(time,timeStep,sigma,positionSeq[j],position2,threshold2);
                    maxProb=maxProb>prob1*prob2?maxProb:prob1*prob2;
                    if(maxProb>p_){
                        std::cout<<1<<std::endl;
                        break;
                    }

                }

                if(maxProb>p_){
                    maxProb1=calculateConflictProb(maxTime1,timeStep,sigma,positionSeq[i],position1,threshold1);
                    maxProb2=calculateConflictProb(maxTime2,timeStep,sigma,positionSeq[j],position2,threshold2);
                    std::cout<<"prob"<<maxProb<<" time:"<<enterTime1<<"  found conflict"<<"("<<i<<","<<j<<") in"<<"("<<itersectPoint.x<<","<<itersectPoint.y<<")"<<std::endl;
                    result.time1=maxTime1;
                    result.time2=maxTime2;
                    result.x=itersectPoint.x;
                    result.y=itersectPoint.y;
                    result.prob1=maxProb1;
                    result.prob2=maxProb2;
                    result.position1=position1;
                    result.position2=position2;
                    result.agent1=i;
                    result.agent2=j;
                    return true;
                }
            }
        }

    }

    return false;
}



bool PCBS(Constraints& constraints,
          std::vector<State>& startStates,
          std::vector<Location>& goals,
          std::vector<ScenarioUser>& robots,
          double timeStep,
          std::vector<std::vector<double>>& positionSeq,
          double sigma,
          double p_){
    Conflicts result;
    int loop=0;

    while(getFirstConflict(startStates,goals,robots,timeStep,result,positionSeq,sigma,p_)){
        ScenarioUser robot1(robots[result.agent1]);
        ScenarioUser robot2(robots[result.agent2]);
        double pathStep=0.01;
        double threshold1=robot1.getVehicle().getLength()+robot2.getVehicle().getWidth()/2;
        double threshold2=robot2.getVehicle().getLength()+robot1.getVehicle().getWidth()/2;

        double pl=0,pu=0,tl=0,tu=0;
        for(double time=result.time1;time<positionSeq[result.agent1].size()*timeStep;time+=timeStep){
            double prob=calculateConflictProb(time,timeStep,sigma,positionSeq[result.agent1],result.position1,threshold1);
            if(prob<p_){
                tu=time;
                break;
            }
        }
        for(double time=result.time1;time>0;time-=timeStep){
            double prob=calculateConflictProb(time,timeStep,sigma,positionSeq[result.agent1],result.position1,threshold1);
            if(prob<p_){
                tl=time;
                break;
            }
        }
        double minPj=p_/result.prob1;
        for(double pos=result.position2;pos<positionSeq[result.agent2].back();pos+=pathStep){
            double prob=calculateConflictProb(result.time1,sigma,pos,result.position2,threshold2);
            if(prob<minPj){
                pu=pos;
                break;
            }
        }
        for(double pos=result.position2;pos>0;pos-=pathStep){
            double prob=calculateConflictProb(result.time1,sigma,pos,result.position2,threshold2);
            if(prob<minPj){
                pl=pos;
                break;
            }
        }
        double ld=pu-pl;
        double z=0;
        for(double time=tl;time<tu;time+=timeStep){
            for(double tempZ=0;tempZ<15;tempZ+=timeStep){
                normal s(0,sigma*sqrt(time));
                double pdfL=-tempZ-ld;
                double pdfh=-tempZ;
                double prob=cdf(s,pdfh)-cdf(s,pdfL);
                if((prob)<p_){
                    z=z>tempZ?z:tempZ;
                    break;
                }
            }
        }
        pl=pl-z;
        pu=pu+z;
        robots[result.agent2].scimp_controller.addPVTObstacleSet(tl,tu,pl,pu);
        //robots[result.agent2].generatePositionSeq(timeStep);
        std::cout << robots[result.agent2].scimp_controller.obstacle_set<< std::endl;
        std::cout << std::endl;

        loop=loop+1;
    }
    if(loop>=1000)
        return false;
    else
        return true;

}
double caculateCost(std::vector<ScenarioUser>& robots,
                    double timeStep){
    double cost=0;
    for(size_t i=0;i<=robots.size()-1;i++){
        ScenarioUser tempRobot(robots[i]);
        double tempTime=getTimeByPosition(tempRobot.constraints.getXLimit(),timeStep,tempRobot);
        cost=cost>tempTime?cost:tempTime;
    }
}

double caculateCost2(std::vector<ScenarioUser>& robots,
                     double timeStep){
    double cost=0;
    for(size_t i=0;i<=robots.size()-1;i++){
        ScenarioUser tempRobot(robots[i]);
        double tempTime=getTimeByPosition(tempRobot.constraints.getXLimit(),timeStep,tempRobot);
        cost=cost+tempTime;
    }
}

int main () {

    // I use this for reading out values with enough precision to do debugging;
    // sometimes it's handy to print something out to cerr to see what's
    // going on
    std::cerr.precision(Constants::DEBUGGING_OUTPUT_PRECISION);

    /**
     * First, we'll define the system constraints. These are encapsulated in
     * a Constraints object and handed off to the system.
     *
     * Units are in meters and seconds, where appropriate.
     */


    double pathLength = 70.;
    double timeLimit =100.;
    double minimumVelocity = 0.;
    double maximumVelocity = 3.;
    double minimumAcceleration = -3.;
    double maximumAcceleration =2;
    double timeStep = 0.1;
    const size_t rowNum=6;
    const size_t colNum=6;
    double sigma=0.1;
    double p_=0.01;


    // if constraint construction fails because of inconsistent constraints, fail out
    try {
        Constraints constraints( pathLength,
                                 timeLimit,
                                 minimumVelocity,
                                 maximumVelocity,
                                 minimumAcceleration,
                                 maximumAcceleration );

        std::vector<Location> goals;
        std::vector<State> startStates;
        std::vector<ScenarioUser> robots;
        std::vector<Result> solution;

        for(size_t i=0;i<rowNum;i++){
            goals.emplace_back(Location(pathLength,pathLength/(rowNum+1)*(i+1)));
            startStates.emplace_back(State(0,0,pathLength/(rowNum+1)*(i+1),static_cast<double>(i)/static_cast<double>(rowNum)*maximumVelocity,1,2));
        }

        for(size_t i=0;i<colNum;i++){
            goals.emplace_back(Location(pathLength/(colNum+1)*(i+1),0));
            startStates.emplace_back(State(0,pathLength/(colNum+1)*(i+1),pathLength,static_cast<double>(i)/static_cast<double>(colNum)*maximumVelocity,1,2));
        }

        double minimumFinalVelocity = 0.;
        double maximumFinalVelocity = 3.;
        double alpha = 0.999;
        std::vector<std::vector<double>> positionSeq;

        for(size_t i=0;i<rowNum+colNum;i++){
            XY_Point robotBegin(0.,0.);
            XY_Point robotEnd(0,pathLength);
            Path robotPath(robotBegin,robotEnd);
            Vehicle robotVehicle( startStates[i].width,
                                  startStates[i].length,
                                  constraints.getVMin(),
                                  constraints.getVMax(),
                                  constraints.getAMin(),
                                  constraints.getAMax() );
            double robotInitialVelocity = startStates[i].v;
            robotVehicle.setVelocity( robotInitialVelocity);
            ;
            robots.emplace_back(ScenarioUser(robotVehicle,robotPath,minimumFinalVelocity,maximumFinalVelocity,timeStep,alpha,constraints));
            robots[i].initializeScenario();
        }

        Timer timer;
        if(PCBS(constraints,startStates,goals,robots,timeStep,positionSeq,sigma,p_))
        {
            timer.stop();
            std::cout<<"Time:  "<<timer.elapsedSeconds() << std::endl;
            for(size_t i=0;i<=robots.size()-1;i++){
                char fileName[256];
                sprintf(fileName,"robot%d.txt",i);
                std::ofstream outFile(fileName);
                ScenarioUser robot(robots[i]);
                double user_control = robot.getVehicle().getMaximumAcceleration();

                // Set up a loop that steps through time up to the pre-defined limit.
                double current_time = 0.;
                while ( !robot.pathTraversed() && !robot.timeLimitReached() && (current_time <= timeLimit) ) {

                    outFile<<robot.vehicle.getPosition()<<"    "<<current_time<<"  "<<robot.vehicle.getVelocity()<<std::endl;

                    // Attempt to apply user_control over this time step
                    robot.applyFilteredControl( user_control, timeStep );


                    // Step forward in time.
                    current_time += timeStep;
                }
            }
            std::cout<<"cost: "<<caculateCost(robots,timeStep)<<std::endl;
            std::cout<<"success"<<std::endl;
        }else{
            std::cout<<"failed to plan"<<std::endl;
        }

    } catch ( int e ) {
        Constraints::exceptionMessage( e );
        return -1;
    }

    return 0;
}
