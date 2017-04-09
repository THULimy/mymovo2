#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include <iostream>
#include <ctype.h>
#include <algorithm> 
#include <iterator> 
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

Point3d getGroundTruthP(int frame_id)  {
  string line;
  int i = 0;
  ifstream infile ("/home/limy/mono-vo2/build/GTresults00.txt");
  double x =0, y=0, z = 0;
  double x_prev, y_prev, z_prev;
  Point3d loc;
  if (infile.is_open())
  {
    while (( getline (infile,line) ) && (i<=frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;
      std::istringstream in(line);
      //cout << line << '\n';
      for (int j=0; j<6; j++)  {
        in >> z ;
        if (j==4) y=z;
        if (j==3)  x=z;
      }
      
      i++;
    }
    infile.close();
    loc.x=x;
    loc.y=y;
    loc.z=z;    

  }
  else {
    cout << "Unable to open file";
    return loc;
  }
  return loc;
}

Point3d getGroundTruthV(int frame_id)  {
  string line;
  int i = 0;
  ifstream infile ("/home/limy/mono-vo2/build/GTresults00.txt");
  double x =0, y=0, z = 0,temp=0;
  double x_prev, y_prev, z_prev;
  Point3d vel;
  if (infile.is_open())
  {
    while (( getline (infile,line) ) && (i<=frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;
      std::istringstream in(line);
      //cout << line << '\n';
      for (int j=0; j<6; j++)  {
       	in >> temp ;
       	if (j==0) x=temp;
        if (j==1) y=temp;
        if (j==2) z=temp;
      }
      
      i++;
    }
    infile.close();
    vel.x=x;
    vel.y=y;
    vel.z=z;    

  }
  else {
    cout << "Unable to open file";
    return vel;
  }
  return vel;
}

int main()
{
  int numFrame;
  Point3d loc,vel;
  ofstream outfileGT1,outfileGT2;
  outfileGT1.open("allNOISEresults00.txt");
  outfileGT2.open("someNOISEresults00.txt");
  double nplevel_L=2.0,nplevel_M=1.0,nplevel_S=0.5;
  double nvlevel_L=0.6,nvlevel_M=0.4,nvlevel_S=0.2;

  for(numFrame=0;numFrame<1000;numFrame++){

  loc=getGroundTruthP(numFrame);
  vel=getGroundTruthV(numFrame);
  outfileGT1 << vel.x-nvlevel_M+2*nvlevel_M*rand()/double(RAND_MAX) << " " << vel.y-nvlevel_M+2*nvlevel_M*rand()/double(RAND_MAX) << " " << vel.z-nvlevel_M+2*nvlevel_M*rand()/double(RAND_MAX) <<" ";
  outfileGT1 << loc.x-nplevel_L+2*nplevel_L*rand()/double(RAND_MAX) << " " << loc.y-nplevel_L+2*nplevel_L*rand()/double(RAND_MAX) << " " << loc.z-nplevel_L+2*nplevel_L*rand()/double(RAND_MAX) << endl;

  if(numFrame>=250 && numFrame<400){
  	outfileGT2 << vel.x-nvlevel_M+2*nvlevel_M*rand()/double(RAND_MAX) << " " << vel.y-nvlevel_M+2*nvlevel_M*rand()/double(RAND_MAX) << " " << vel.z-nvlevel_M+2*nvlevel_M*rand()/double(RAND_MAX) <<" ";
  	outfileGT2 << loc.x-nplevel_M+2*nplevel_M*rand()/double(RAND_MAX) << " " << loc.y-nplevel_M+2*nplevel_M*rand()/double(RAND_MAX) << " " << loc.z-nplevel_M+2*nplevel_M*rand()/double(RAND_MAX) << endl;
  }
  else if(numFrame>=600 && numFrame<800){
  	outfileGT2 << vel.x-nvlevel_L+2*nvlevel_L*rand()/double(RAND_MAX) << " " << vel.y-nvlevel_L+2*nvlevel_L*rand()/double(RAND_MAX) << " " << vel.z-nvlevel_L+2*nvlevel_L*rand()/double(RAND_MAX) <<" ";
  	outfileGT2 << loc.x-nplevel_L+2*nplevel_L*rand()/double(RAND_MAX) << " " << loc.y-nplevel_L+2*nplevel_L*rand()/double(RAND_MAX) << " " << loc.z-nplevel_L+2*nplevel_L*rand()/double(RAND_MAX) << endl;
  }
  else {
  	outfileGT2 << vel.x << " " << vel.y << " " << vel.z <<" ";
  	outfileGT2 << loc.x << " " << loc.y << " " << loc.z << endl;
  }
}
  outfileGT1.close();
  outfileGT2.close();
  return 0;

}