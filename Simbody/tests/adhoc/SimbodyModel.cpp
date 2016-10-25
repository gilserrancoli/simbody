#include "Simbody.h"
#include "SimTKcommon.h"
#include <C:\ADOL-C-sparse\include\adolc\adolc.h>
#include <C:\ADOL-C-sparse\include\adolc\adolc_sparse.h>
#include "SimTKmath.h"


using namespace SimTK;
void main() {
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
	pendulum2.setRate(state, 5.0);

	// Our implementation



	int nb = matter.getNumBodies();
	system.realize(state, Stage::Velocity);

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

	printf("Number of Qs %d", state.getNQ());
	printf("Number of Us %d", state.getNU());

	//

	///// This differentiates a function R^n -> R^m at px using ADOL-C.
	//void auto_jacobian(int n, int m, const double* px, double** J) {

	//	short int tag = 0;

	//	// Start recording information for computing derivatives.
	//	trace_on(tag);

	//	SimTK::vector<adouble> x(n);
	//	SimTK::vector<adouble> y(m);
	//	SimTK::vector<double> py(m); // p for "passive variable;" ADOL-C terminology.

	//	// Indicate independent variables.
	//	for (int i = 0; i < n; ++i) x[i] <<= px[i];

	//	// Evaluate function.
	//	constraint_function_dense(n, m, x.data(), y.data());



	//	// Indicate dependent variables.
	//	for (int j = 0; j < m; ++j) y[j] >>= py[j];

	//	// Stop recording.
	//	trace_off();

	//	// Use the recorded tape to compute the jacobian.
	//	int success = jacobian(tag, m, n, px, J);
	//	assert(success == 3);
	//}



}