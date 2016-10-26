#include "Simbody.h"
#include "SimTKcommon.h"
#include <adolc.h>
#include <adolc_sparse.h>
#include "SimTKmath.h"

#include <iostream>
#include <iterator>
#include <random>
#include <cassert>
#include <algorithm>


using namespace SimTK;
int main() {
	// Define the system.
	MultibodySystem system;
	SimbodyMatterSubsystem matter(system);
	GeneralForceSubsystem forces(system);
	Force::UniformGravity gravity(forces, matter, Vec3(10, Real(-9.8), 3));
	
	// Describe mass and visualization properties for a generic body.
	Body::Rigid bodyInfo(MassProperties(1.0, Vec3(0), UnitInertia(1)));
	//bodyInfo.addDecoration(Transform(), DecorativeSphere(0.1));

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
	pendulum1.setRate(state, 6.0);
	pendulum1.setAngle(state, 1);
	pendulum2.setRate(state, 5.0);
	pendulum2.setAngle(state, 0.5);

	// Our implementation
    //system.realize(state, SimTK::Stage::Dynamics);

	int nb = matter.getNumBodies();
	adouble x1, x2, x3, x4; // independent variables.
	adouble y1, y2, y3; // dependent variables.

	trace_on(1);

	system.realize(state, Stage::Velocity);

	MobilizedBodyIndex indxpd1 = pendulum1.getMobilizedBodyIndex();
	MobilizedBodyIndex indxpd2 = pendulum2.getMobilizedBodyIndex();

	SpatialVec pdl1_CorAcc = matter.getTotalCoriolisAcceleration(state, indxpd1);
	SpatialVec pdl2_CorAcc = matter.getTotalCoriolisAcceleration(state, indxpd2);

	SpatialVec pdl1_GyrFor = matter.getGyroscopicForce(state, indxpd1);
	SpatialVec pdl2_GyrFor = matter.getGyroscopicForce(state, indxpd2);

	SpatialInertia I1 = pendulum1.getBodySpatialInertiaInGround(state);
	SpatialInertia I2 = pendulum2.getBodySpatialInertiaInGround(state);

	SpatialVec TotGyrCorIn1 = I1*pdl1_CorAcc + pdl1_GyrFor;
	SpatialVec TotGyrCorIn2 = I2*pdl2_CorAcc + pdl2_GyrFor;

	Vector Q = state.getQ();
	Vector U = state.getY();

	// Try to differentiate something

	// Indicate independent variables.
	x1 <<= Q.get(0).getValue();
	x2 <<= Q.get(1).getValue();
	x3 <<= U.get(0).getValue();
	x4 <<= U.get(1).getValue();

	Vec3 TotGyrCorIn1_b1_F = TotGyrCorIn1[0];
	double TotGyrCorIn1_b1_F_f1 = TotGyrCorIn1_b1_F[0].getValue();
	double TotGyrCorIn1_b1_F_f2 = TotGyrCorIn1_b1_F[1].getValue();
	double TotGyrCorIn1_b1_F_f3 = TotGyrCorIn1_b1_F[2].getValue();

	// Indicate dependent variables.
	y1 >>= TotGyrCorIn1_b1_F_f1;
	y2 >>= TotGyrCorIn1_b1_F_f2;
	y3 >>= TotGyrCorIn1_b1_F_f3;

	trace_off();

	double xp[4];
	xp[0] = Q.get(0).getValue(); xp[1] = U.get(0).getValue();
	xp[2] = Q.get(1).getValue(); xp[3] = U.get(1).getValue();

	double** J;
	J = myalloc(3, 4);
	jacobian(1, 3, 4, xp, J);
	double *v1, *z1;
	v1 = myalloc(4); z1 = myalloc(3);
	jac_vec(1, 3, 4, xp, v1, z1);

	printf("Jacobian \n %f %f %f %f \n  %f %f %f %f \n  %f %f %f %f \n", J[0], J[1], J[2], J[3], J[4], J[5], J[6], J[7], J[8], J[9], J[10], J[11]);
	printf("Jacobian vector %f %f %f %f \n ", z1[0], z1[1], z1[2], z1[3]);

}



