/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#include <ow_com/com_estimator.h>

namespace ow_com
{

COMEstimator::COMEstimator() :
  Base("COMEstimator"),
  Xf_com_w_(ow::CartesianState::Zero()),
  DCMr_w_(ow::LinearState::Zero())
{
}

COMEstimator::~COMEstimator()
{
}

bool COMEstimator::init(const ow::Parameter& parameter, ros::NodeHandle& nh)
{
  // get global ow parameter
  ow::Scalar frequency = parameter.get<ow::Scalar>("loop_rate");

  parameter_.add<ow::Scalar>("f_cutoff", 5.0);

  // load module parameter
  if(!parameter_.load(nh, "com_estimator"))
  {
    ROS_ERROR("%s::initialize: Config loading failed.", Base::name().c_str());
    return false;
  }

  // init members
  differentiator_.reset(new Differentiator(
    ow::ScalarFiniteDifference::FirstOrderAccurarcyThree(frequency)));

  com_filter_.reset(new Filter(ow::ScalarButterWorthFilter(
    frequency, parameter_.get<ow::Scalar>("f_cutoff"))));

  return true;
}

void COMEstimator::update(
  const ow::CartesianState& X_com_w, const ow::ImuSensor& imu)
{
  // no model based filter at the moment just forward the fk

  Xf_com_w_ = X_com_w;
  Xf_com_w_.vel().angular().setZero();                               
  
  // low pass filter on velocity to smooth dcm
  Xf_com_w_.vel() = com_filter_->update(Xf_com_w_.vel());

  // compute the capture point
  DCMr_w_.pos() = compute_capture_point();

  // compute its derivatives
  DCMr_w_ = differentiator_->update(DCMr_w_);
}

const ow::CartesianState& COMEstimator::Xf_com_w() const
{
  return Xf_com_w_;
}

const ow::LinearState& COMEstimator::DCMr_w() const
{
  return DCMr_w_;
}

ow::LinearPosition COMEstimator::compute_capture_point()
{
  // lipm time constant
  ow::Scalar omega = std::sqrt(Xf_com_w_.pos().linear().z()/OW_GRAVITY);

  // capture point
  ow::LinearPosition dcm;
  dcm.x() = Xf_com_w_.pos().linear().x() + omega*Xf_com_w_.vel().linear().x();
  dcm.y() = Xf_com_w_.pos().linear().y() + omega*Xf_com_w_.vel().linear().y();
  dcm.z() = Xf_com_w_.pos().linear().z();

  return dcm;
}

}
