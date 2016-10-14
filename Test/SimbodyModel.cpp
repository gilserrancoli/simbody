#include "Simbody.h"
#include "SimTKcommon.h"
#include <adolc.h>
#include <adolc_sparse.h>
#include "SimTKmath.h"


using namespace SimTK;
void main() {
	// Define the system.
	MultibodySystem system;
	SimbodyMatterSubsystem matter(system);
	GeneralForceSubsystem forces(system);
	Force::Gravity gravity(forces, matter, -YAxis, 9.8);

	// Describe mass and visualization properties for a generic body.
	Body::Rigid bodyInfo(MassProperties(1.0, Vec3(0), UnitInertia(1)));
	bodyInfo.addDecoration(Transform(), DecorativeSphere(0.1));

	// Create the moving (mobilized) bodies of the pendulum.
	MobilizedBody::Pin pendulum1(matter.Ground(), Transform(Vec3(0)),
		bodyInfo, Transform(Vec3(0, 1, 0)));
	MobilizedBody::Pin pendulum2(pendulum1, Transform(Vec3(0)),
		bodyInfo, Transform(Vec3(0, 1, 0)));

	// Set up visualization.
	/*Visualizer viz(system);
	system.addEventReporter(new Visualizer::Reporter(viz, 0.01));*/

	// Initialize the system and state.
	State state = system.realizeTopology();
	pendulum2.setRate(state, 5.0);

	// Our implementation


	int nb = matter.getNumBodies();

	MobilizedBodyIndex indxpd1 = pendulum1.getMobilizedBodyIndex();
	MobilizedBodyIndex indxpd2 = pendulum2.getMobilizedBodyIndex();

	SpatialVec pdl1_CorAcc = matter.getTotalCoriolisAcceleration(state, indxpd1);
	SpatialVec pdl2_CorAcc = matter.getTotalCoriolisAcceleration(state, indxpd2);

	SpatialVec pdl1_GyrFor = matter.getGyroscopicForce(state, indxpd1);
	SpatialVec pdl2_GyrFor = matter.getGyroscopicForce(state, indxpd2);

	SpatialInertia I1 = pendulum1.getBodySpatialInertiaInGround(state);
	SpatialInertia I2 = pendulum2.getBodySpatialInertiaInGround(state);

	Vector_<SpatialVec> TotGyrCorIn1;
	Vector_<SpatialVec> TotGyrCorIn2;

	TotGyrCorIn1 = I1*pdl1_CorAcc + pdl1_GyrFor;
	TotGyrCorIn2 = I2*pdl2_CorAcc + pdl2_GyrFor;

	Vector a = state.getQ();
	adouble aa = a.get(0);

	//// Simulate for 20 seconds.
	//RungeKuttaMersonIntegrator integ(system);
	//TimeStepper ts(system, integ);
	//ts.initialize(state);
	//ts.stepTo(20.0);
	//return ;0
}