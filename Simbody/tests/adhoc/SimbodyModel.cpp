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

	// Create the moving (mobilized) bodies of the pendulum.
	MobilizedBody::Pin pendulum1(matter.Ground(), Transform(Vec3(0)),
		bodyInfo, Transform(Vec3(0, 1, 0)));
	MobilizedBody::Pin pendulum2(pendulum1, Transform(Vec3(0)),
		bodyInfo, Transform(Vec3(0, 1, 0)));

	// Initialize the system and state.
	State state = system.realizeTopology();
	pendulum1.setRate(state, 5.0);
	pendulum1.setAngle(state, 1);
	pendulum2.setRate(state, 5.0);
	pendulum2.setAngle(state, 0.5);

	//double xp[4];
	//xp[0] = 1; /*0.5;*/
	//xp[1] = 1; /*0.6;*/
	//xp[2] = 1; /*0.1;*/
	//xp[3] = 1; /*0.2;*/

	//trace_on(1);

	//Vector& Q = state.updQ();
	//Vector& U = state.updU();
	//Q[0] <<= xp[0];
	//Q[1] <<= xp[1];
	//U[0] <<= xp[2];
	//U[1] <<= xp[3];

	//adouble y1 = 2.0 * Q[0];
	//adouble y2 = 3.0 * Q[1];
	//adouble y3 = 4.5 * U[1];

	//double yp[3];
	//y1 >>= yp[0];
	//y2 >>= yp[1];
	//y3 >>= yp[2];

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

	std::cout << pdl1_CorAcc << std::endl;
	std::cout << pdl2_CorAcc << std::endl;

	std::cout << pdl1_GyrFor << std::endl;
	std::cout << pdl2_GyrFor << std::endl;

	std::cout << TotGyrCorIn1 << std::endl;
	std::cout << TotGyrCorIn2 << std::endl;

	////// Try to differentiate something

	////// Indicate independent variables.
	/////*adouble x1 = Q.get(0);
	////adouble x2 = Q.get(1);
	////adouble x3 = U.get(0);
	////adouble x4 = U.get(1);

	////x1 <<= 0;
	////x2 <<= 0;
	////x3 <<= 0;
	////x4 <<= 0;*/

	////// Indicate dependent variables.
	////Vec3 TotGyrCorIn1_b1_F = TotGyrCorIn1[0];
	////adouble y1 = TotGyrCorIn1_b1_F[0];
	////adouble y2 = TotGyrCorIn1_b1_F[1];
	////adouble y3 = TotGyrCorIn1_b1_F[2];
	////double TotGyrCorIn1_b1_F_f1;
	////double TotGyrCorIn1_b1_F_f2;
	////double TotGyrCorIn1_b1_F_f3;
	////   
	////y1 >>= TotGyrCorIn1_b1_F_f1;
	////y2 >>= TotGyrCorIn1_b1_F_f2;
	////y3 >>= TotGyrCorIn1_b1_F_f3;  

	//trace_off();

	///*double xp[4];
	//xp[0] = Q.get(0).getValue(); xp[1] = U.get(0).getValue();
	//xp[2] = Q.get(1).getValue(); xp[3] = U.get(1).getValue();*/

	//double** J;
	//J = myalloc(3, 4);
	//jacobian(1, 3, 4, xp, J);
	//double *v1, *z1;
	//v1 = myalloc(4); z1 = myalloc(3);
	//v1[0] = 1.0;
	//v1[1] = 1.0;
	//v1[2] = 1.0;
	//v1[3] = 1.0;
	//jac_vec(1, 3, 4, xp, v1, z1);

	////printf("Jacobian \n %f %f %f %f \n  %f %f %f %f \n  %f %f %f %f \n", J[0], J[1], J[2], J[3], J[4], J[5], J[6], J[7], J[8], J[9], J[10], J[11]);
	///*printf("Jacobian \n %f %f %f %f \n  %f %f %f %f \n  %f %f %f %f \n",
	//     J[0][0], J[0][1], J[0][2], J[0][3],
	//     J[1][0], J[1][1], J[1][2], J[1][3],
	//     J[2][0], J[2][1], J[2][2], J[2][3]);
	//printf("Jacobian vector %f %f %f %f \n ", z1[0], z1[1], z1[2], z1[3]);*/
}






//#include "Simbody.h"
//#include "SimTKcommon.h"
//#include <adolc.h>
//#include <adolc_sparse.h>
//#include "SimTKmath.h"
//#include <iostream>
//#include <iterator>
//#include <random>
//#include <cassert>
//#include <algorithm>
//#include <chrono>
//
//using namespace SimTK;



//Vector_<SpatialVec> CalculateMomentums(State state, MobilizedBody::Pin pendulum1, MobilizedBody::Pin pendulum2, SimbodyMatterSubsystem matter){
//	printf("test0a");
//	MobilizedBodyIndex indxpd1 = pendulum1.getMobilizedBodyIndex();
//	MobilizedBodyIndex indxpd2 = pendulum2.getMobilizedBodyIndex();
//
//	printf("test0b");
//	printf("getNY=%d", state.getNY());
//	SpatialVec pdl1_CorAcc = matter.getTotalCoriolisAcceleration(state, indxpd1);
//	SpatialVec pdl2_CorAcc = matter.getTotalCoriolisAcceleration(state, indxpd2);
//
//	printf("test0c");
//	SpatialVec pdl1_GyrFor = matter.getGyroscopicForce(state, indxpd1);
//	SpatialVec pdl2_GyrFor = matter.getGyroscopicForce(state, indxpd2);
//
//	printf("test0d");
//	SpatialInertia I1 = pendulum1.getBodySpatialInertiaInGround(state);
//	SpatialInertia I2 = pendulum2.getBodySpatialInertiaInGround(state);
//
//	printf("test0e");
//	SpatialVec TotGyrCorIn1;
//	SpatialVec TotGyrCorIn2;
//
//	printf("test0f");
//	TotGyrCorIn1 = I1*pdl1_CorAcc + pdl1_GyrFor;
//	TotGyrCorIn2 = I2*pdl2_CorAcc + pdl2_GyrFor;
//
//	printf("test0g");
//	Vector_<SpatialVec> Output(2);
//	Output.set(0, TotGyrCorIn1);
//	Output.set(1, TotGyrCorIn2);
//	printf("end");
//	return Output;
//	}
//
//Matrix CalculateFD(State state, MobilizedBody::Pin pendulum1, MobilizedBody::Pin pendulum2, SimbodyMatterSubsystem matter, MultibodySystem system){
//	Matrix Jac(3, 4);
//	printf("test0");
//	State state0 = state;
//	printf("test1");
//	Vector_<SpatialVec> M0 = CalculateMomentums(state0, pendulum1, pendulum2, matter);
//	printf("test2");
//	Vector state1 = state.getY();
//	printf("test3");
//	state1.set(0, state1.get(0) + 1e-7);
//	printf("test4");
//	State Sstate1 = state;
//	Sstate1.setY(state1);
//	printf("test5");
//	system.realize(Sstate1, Stage::Velocity);
//	printf("test6");
//	Vector_<SpatialVec> M1 = CalculateMomentums(Sstate1, pendulum1, pendulum2, matter);
//
//	Vector state2 = state.getY();
//	state2.set(0, state2.get(1) + 1e-7);
//	State Sstate2 = state;
//	Sstate2.setY(state2);
//	system.realize(Sstate2, Stage::Velocity);
//	Vector_<SpatialVec> M2 = CalculateMomentums(Sstate2, pendulum1, pendulum2, matter);
//
//	Vector state3 = state.getY();
//	state3.set(0, state3.get(2) + 1e-7);
//	State Sstate3 = state;
//	Sstate3.setY(state3);
//	system.realize(Sstate3, Stage::Velocity);
//	Vector_<SpatialVec> M3 = CalculateMomentums(Sstate3, pendulum1, pendulum2, matter);
//
//	Vector state4 = state.getY();
//	state4.set(0, state4.get(3) + 1e-7);
//	State Sstate4 = state;
//	Sstate4.setY(state4);
//	system.realize(Sstate4, Stage::Velocity);
//	Vector_<SpatialVec> M4 = CalculateMomentums(Sstate4, pendulum1, pendulum2, matter);
//
//	for (int m = 0; m < 3; m++) {
//		Jac.set(m, 0, (M1[0].get(0).get(m) - M0[0].get(0).get(m)) / 1e-7);
//		Jac.set(m, 1, (M2[0].get(0).get(m) - M0[0].get(0).get(m)) / 1e-7);
//		Jac.set(m, 2, (M3[0].get(0).get(m) - M0[0].get(0).get(m)) / 1e-7);
//		Jac.set(m, 3, (M4[0].get(0).get(m) - M0[0].get(0).get(m)) / 1e-7);
//	}
//	return Jac;
//}
//
//
//int main() {
//	// Define the system.
//	MultibodySystem system;
//	SimbodyMatterSubsystem matter(system);
//	GeneralForceSubsystem forces(system);
//	Force::UniformGravity gravity(forces, matter, Vec3(10, Real(-9.8), 3));
//	
//	// Describe mass and visualization properties for a generic body.
//	Body::Rigid bodyInfo(MassProperties(1.0, Vec3(0), UnitInertia(1)));
//	//bodyInfo.addDecoration(Transform(), DecorativeSphere(0.1));
//
//	// Create the moving (mobilized) bodies of the pendulum.
//	MobilizedBody::Pin pendulum1(matter.Ground(), Transform(Vec3(0)),
//		bodyInfo, Transform(Vec3(0, 1, 0)));
//	MobilizedBody::Pin pendulum2(pendulum1, Transform(Vec3(0)),
//		bodyInfo, Transform(Vec3(0, 1, 0)));
//
//	// Set up visualization.
//	/*Visualizer viz(system);
//	system.addEventReporter(new Visualizer::Reporter(viz, 0.01));*/
//
//	// Initialize the system and state.
//	State state = system.realizeTopology();
//	pendulum2.setRate(state, 5.0);
//
//	//// Our implementation
// //   system.realize(state, SimTK::Stage::Dynamics);
//	
//	const int n = 4;
//	const int m = 3;
//	double** J = myalloc(m, n);
//	SimTK::Vector x_vector(n);
//	std::vector<adouble> x(n);
//	std::vector<adouble> y(m);
//	std::vector<double> py(m);
//
//	short int tag = 1;
//	trace_on(tag);
//
//	int nb = matter.getNumBodies();
//	system.realize(state, Stage::Velocity);
//
//	MobilizedBodyIndex indxpd1 = pendulum1.getMobilizedBodyIndex();
//	MobilizedBodyIndex indxpd2 = pendulum2.getMobilizedBodyIndex();
//
//	SpatialVec pdl1_CorAcc = matter.getTotalCoriolisAcceleration(state, indxpd1);
//	SpatialVec pdl2_CorAcc = matter.getTotalCoriolisAcceleration(state, indxpd2);
//
//	SpatialVec pdl1_GyrFor = matter.getGyroscopicForce(state, indxpd1);
//	SpatialVec pdl2_GyrFor = matter.getGyroscopicForce(state, indxpd2);
//
//	SpatialInertia I1 = pendulum1.getBodySpatialInertiaInGround(state);
//	SpatialInertia I2 = pendulum2.getBodySpatialInertiaInGround(state);
//
//	SpatialVec TotGyrCorIn1;
//	SpatialVec TotGyrCorIn2;
//	
//	TotGyrCorIn1 = I1*pdl1_CorAcc + pdl1_GyrFor;
//	TotGyrCorIn2 = I2*pdl2_CorAcc + pdl2_GyrFor;
//
//	Vector a = state.getY();
//	adouble aa = a.get(0);
//
//	printf("Number of Qs %d", state.getNQ());
//	printf("Number of Us %d", state.getNU());
//
//	//
//
//	///// This differentiates a function R^n -> R^m at px using ADOL-C.
//	//void auto_jacobian(int n, int m, const double* px, double** J) {
//
//
//
//	//	// Start recording information for computing derivatives.
//
//
//		//SimTK::vector<adouble> x(n);
//		//SimTK::vector<adouble> y(m);
//		//SimTK::vector<double> py(m); // p for "passive variable;" ADOL-C terminology.
//		printf("test0 \n");
//		printf("Size TotGyrCorIn1 %d \n", TotGyrCorIn1.size());
//		printf("Mass I1 %f \n", I1.getMass());
//		printf("Get Mass pendulum 1: %f \n", pendulum1.getBodyMass(state).getValue());
//		printf("Mass I2 %f \n", I2.getMass());
//		printf("Get Mass pendulum 2 %f \n", pendulum2.getBodyMass(state).getValue());
//		printf("size pdl1_CorAcc = %d \n", pdl1_CorAcc.size());
//		printf("size pdl1_GyrFor = %d \n", pdl1_GyrFor.size());
//
//		printf("test1 \n");
//		Vec3 pdl1_CorAcc_n1_Vec3 = pdl1_CorAcc[0];
//		
//	/*	y[0] >>= pdl1_CorAcc_n1_Vec3[0].getValue();
//		y[1] >>= pdl1_CorAcc_n1_Vec3[1].getValue();
//		y[2] >>= pdl1_CorAcc_n1_Vec3[2].getValue();*/
//
//		x_vector[0] <<= a.get(0).getValue();
//		x_vector[1] <<= a.get(1).getValue();
//		x_vector[2] <<= a.get(2).getValue();
//		x_vector[3] <<= a.get(3).getValue();
//
//		std::vector<double> vec_dependent(3);
//		vec_dependent[0] = pdl1_CorAcc_n1_Vec3[0].getValue();
//		vec_dependent[1] = pdl1_CorAcc_n1_Vec3[1].getValue();
//		vec_dependent[2] = pdl1_CorAcc_n1_Vec3[2].getValue();
//
//		y[0] >>= vec_dependent[0];
//		y[1] >>= vec_dependent[1];
//		y[2] >>= vec_dependent[2];
//
//		double px[n];
//		/*px[0] = 0; px[1] = 0; px[2] = 0; px[3] = 5;*/
//		px[0] = x_vector[0].getValue();
//		px[1] = x_vector[1].getValue();
//		px[2] = x_vector[2].getValue();
//		px[3] = x_vector[3].getValue();
//
//		// Indicate independent variables.
//		/*for (int i = 0; i < n; ++i) x[i] <<= px[i];*/
//		
//	//	// Evaluate function.
//	//	constraint_function_dense(n, m, x.data(), y.data());
//
//
//
//	//	// Indicate dependent variables.
//	//	for (int j = 0; j < m; ++j) y[j] >>= py[j];
//
//	//	// Stop recording.
//		trace_off();
//		jacobian(tag, m, n, px, J);
//		printf("J: \n, %f %f %f %f \n, %f %f %f %f \n, %f %f %f %f \n, %f %f %f %f \n", J[0][0], J[0][1], J[0][2], J[0][3], J[1][0], J[1][1], J[1][2], J[1][3], J[2][0], J[2][1], J[2][2], J[2][3]);
//
//		px[0] = 0.1;
//		px[1] = 0.2;
//		px[2] = 0.1;
//		px[3] = 0.2;
//		double* y1 = myalloc(3);
//		double* x1 = myalloc(4);
//		jac_vec(tag, m, n, px, x1, y1);
//		printf("y1= [%f %f %f] \n", y1[0],y1[1],y1[2]);
//		printf("Get NY %d \n", state.getNY());
//		Matrix Jac_FD = CalculateFD(state, pendulum1, pendulum2, matter, system);
//		//printf("Jac FD: \n, %f %f %f %f \n, %f %f %f %f \n, %f %f %f %f \n, %f %f %f %f \n", Jac_FD[0][0], Jac_FD[0][1], Jac_FD[0][2], Jac_FD[0][3], Jac_FD[1][0], Jac_FD[1][1], Jac_FD[1][2], Jac_FD[1][3], Jac_FD[2][0], Jac_FD[2][1], Jac_FD[2][2], Jac_FD[2][3]);
//
//
//	//	// Use the recorded tape to compute the jacobian.
//	//	int success = jacobian(tag, m, n, px, J);
//	//	assert(success == 3);
//	//}
//
//
//
//}
//
//
//	//// Simulate for 20 seconds.
//	//RungeKuttaMersonIntegrator integ(system);
//	//TimeStepper ts(system, integ);
//	//ts.initialize(state);
//	//ts.stepTo(20.0);
//	//return ;0


