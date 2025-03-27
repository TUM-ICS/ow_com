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

#ifndef OPEN_WALKER_COM_ESTIMATOR
#define OPEN_WALKER_COM_ESTIMATOR

#include <ow_core/interfaces/i_com_estimator.h>
#include <ow_core/common/smart_ptr.h>
#include <ow_core/algorithms.h>

/*!
 * \brief Open Walker center of mass estimator module namespace. These classes
 * implement the components to estimate the Center of mass state.
 */
namespace ow_com
{

/*!
 * \brief The COMEstimator class
 *
 * This class implements the COMEstimator module of the
 * openwalker framework.
 * 
 */
class COMEstimator : 
  public ow::ICOMEstimator
{
public:
  typedef ow::ICOMEstimator Base;
  typedef ow::StateDifferentiator<ow::LinearState> Differentiator;
  typedef ow_core::MatrixAlgorithm<ow::CartesianVelocity> Filter;

protected:
  ow::Parameter parameter_;           //!< configuration of this module
  ow::CartesianState Xf_com_w_;       //!< the filtered pose of the real CoM.
  ow::LinearState DCMr_w_;               //!< DCMState of the real robot

  std::unique_ptr<Differentiator> differentiator_;

  std::unique_ptr<Filter> com_filter_;

public:
  /*!
  * \brief COMEstimator Default constructor.
  * 
  */
  COMEstimator();

  // destructor
  virtual ~COMEstimator();

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState of the left foot reference wrt world frame.
  */
  virtual const ow::CartesianState& Xf_com_w() const;

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState of the right foot reference wrt world frame.
  */
  virtual const ow::LinearState& DCMr_w() const;


  /*!
   * \brief update fuction.
   *    Update the module.
   *
   * \param X_com_w
   *    CartesianState of the center of mass wrt. world.
   * \param imu
   */
  void update(const ow::CartesianState& X_com_w, const ow::ImuSensor& imu);

protected:
  /*!
   * \brief Initialization of COMEstimator module
   */
  virtual bool init(const ow::Parameter& parameter, ros::NodeHandle& nh);

  /*!
   * \brief Calcualte the capture point
   */
  ow::LinearPosition compute_capture_point();

};

}

#endif
