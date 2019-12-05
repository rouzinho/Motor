/*======================================================================================================================

    Copyright 2011, 2012, 2013, 2014, 2015 Institut fuer Neuroinformatik, Ruhr-Universitaet Bochum, Germany

    This file is part of cedar.

    cedar is free software: you can redistribute it and/or modify it under
    the terms of the GNU Lesser General Public License as published by the
    Free Software Foundation, either version 3 of the License, or (at your
    option) any later version.

    cedar is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
    License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with cedar. If not, see <http://www.gnu.org/licenses/>.

========================================================================================================================

    Institute:   Ruhr-Universitaet Bochum
                 Institut fuer Neuroinformatik

    File:        EarSubscriber.h

    Maintainer:  Tutorial Writer Person
    Email:       cedar@ini.rub.de
    Date:        2011 12 09

    Description:

    Credits:

======================================================================================================================*/

// CEDAR INCLUDES
#include "Motor.h"
#include <cedar/processing/ExternalData.h> // getInputSlot() returns ExternalData
#include <cedar/auxiliaries/MatData.h> // this is the class MatData, used internally in this step
#include "cedar/auxiliaries/math/functions.h"
#include <cmath>
#include "std_msgs/Float64.h"



// SYSTEM INCLUDES

//----------------------------------------------------------------------------------------------------------------------
// constructors and destructor
//----------------------------------------------------------------------------------------------------------------------
Motor::Motor()
:
cedar::proc::Step(true),
mOutput(new cedar::aux::MatData(cv::Mat::zeros(1, 1, CV_32F))),
mInput(new cedar::aux::MatData(cv::Mat::zeros(1, 100, CV_32F))),
mTopic(new cedar::aux::StringParameter(this, "Topic Name", "")),
mLimb(new cedar::aux::StringParameter(this, "Limb Name", "")),
mSize(new cedar::aux::IntParameter(this, "Size",100)),
mEffort(new cedar::aux::DoubleParameter(this,"effort",0.5)),
mLower(new cedar::aux::DoubleParameter(this,"lower",-1.0)),
mUpper(new cedar::aux::DoubleParameter(this,"upper",1.0))
{
this->declareInput("motor", true);
this->declareOutput("output",mOutput);
motorPos.data = 0;
motorCommand.name.resize(1);
motorCommand.position.resize(1);
motorCommand.effort.resize(1);
motorCommand.velocity.resize(1);
motorCommand.position[0] = 0;
motorCommand.effort[0] = 0.5;
motorCommand.velocity[0] = 0;
size = 100;
upper = 1;
lower = -1;
begin_peak = 0;
end_peak = 0;
is_peak = false;
field_pos = 0;
//this->connect(this->mCenter.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mTopic.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
this->connect(this->mLimb.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
this->connect(this->mEffort.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mSize.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mLower.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mUpper.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));

}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
//generic publisher. If limbName is there, a JointCommand will be published. Otherwise it's publishing a Float64 on the topic name.
void Motor::compute(const cedar::proc::Arguments&)
{
   //ros::Rate loop_rate(60);
   cv::Mat& sum = mInput->getData();
   cedar::aux::ConstDataPtr op1 = this->getInputSlot("motor")->getData();
   sum = op1->getData<cv::Mat>();



   for(int i = 0;i < size;i++)
   {
     if(sum.at<float>(i) > 0.95)
     {
       if(is_peak == false)
       {
         is_peak = true;
         begin_peak = i;
         end_peak = i;
       }
       else
       {
         end_peak = i;
       }
     }
   }

   if(is_peak)
   {
    pos = static_cast<double> ((begin_peak + end_peak)/2);
    //field_pos = pos;
    pos = setPosition(pos);
    //if(std::abs(std::abs(old_pos) -  std::abs(pos))> 0.01)
    {
       if(choice == 0)
       {
          motorPos.data = pos;
          pub.publish(motorPos);
          //loop_rate.sleep();
          //ros::spinOnce();
       }
       else
       {
          motorCommand.position.resize(1);
          motorCommand.position[0] = pos;
          pub.publish(motorCommand);
          //std::cout << motorCommand << '\n';
          //loop_rate.sleep();
          //ros::spinOnce();
       }
    }
    old_pos = pos;
  }
  is_peak = false;
  field_pos = getPosition(old_pos);
  this->mOutput->getData().at<float>(0,0) = field_pos;
}
//simple function to transform coordinates
double Motor::setPosition(double position)
{
  double p = position / size;
  double a = (upper-lower);
  double res = a*p + lower;

  return res;
}
//specificc to this size
double Motor::getPosition(double position)
{
   double a = size / (upper - lower);
   double res = position * a + 50;

   return res;
}

void Motor::reCompute()
{
   const std::string tname = topicName;
   const std::string tnamelimb = limbName;
   if(!tnamelimb.empty())
   {
      choice = 1;
      motorCommand.name.resize(1);
      motorCommand.name[0] = tnamelimb;
      motorCommand.effort[0] = effort;
      pub = n.advertise<sensor_msgs::JointState>(topicName, 1000);
   }
   else
   {
      choice = 0;
      pub = n.advertise<std_msgs::Float64>(tname, 1000);
   }
}

void Motor::reName()
{
   topicName = this->mTopic->getValue();
   limbName = this->mLimb->getValue();
   effort = this->mEffort->getValue();
   size = this->mSize->getValue();
   lower = this->mLower->getValue();
   upper = this->mUpper->getValue();
}

void Motor::reset()
{

	//ros::shutdown();

}
