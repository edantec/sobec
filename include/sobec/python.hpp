#ifndef __sobec_python__
#define __sobec_python__

#include <boost/python.hpp>
#include <pinocchio/fwd.hpp>

namespace sobec {
namespace python {

void exposeStdContainers();
void exposeResidualCoMVelocity();
void exposeResidualVelCollision();
void exposeResidualCenterOfPressure();
void exposeResidualFeetCollision();
void exposeResidualFlyHigh();
void exposeActivationQuadRef();
void exposeDesigner();
void exposeHorizonManager();
void exposeModelFactory();
void exposeWBC();
void exposeFootTrajectory();
void exposeFlex();
void exposeOCPWalk();
void exposeMPCWalk();

}  // namespace python
}  // namespace sobec

#endif  // #ifndef __sobec_python__
