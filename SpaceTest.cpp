#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <utility>

#include"Space.h"

void FixedThrustRocketEngineTest() {
	bool testFailed = false;
	double engineThrust = 100.0;
	double engineTime = 10.0;

	FixedThrustRocketEngine engine("engine1", Vector(1.0, 0.0, 0.0), Vector(1.0, 0.0, 0.0), engineThrust, engineTime);

	double timeToBurnOutHalfFuel = engineTime / 2.0;
	double timeToBurnOutAllFuel = engineTime * 100;

	if (!engine.enable()) {
		std::cout << "FAIL: Can\'t enable fixed thrust rocket engine that was never used" << std::endl;
		testFailed = true;
	}

	engine.update(timeToBurnOutHalfFuel);

	if (engine.disable()) {
		std::cout << "FAIL: It\'s possible to disable fixed thrust rocket engine" << std::endl;
		testFailed = true;
	}

	if (std::fabs(engine.getThrustValue() - engineThrust) >= EPS) {
		std::cout << "FAIL: Enabled fixed thrust rocket engine has no thrust" << std::endl;
		testFailed = true;
	}

	engine.update(timeToBurnOutAllFuel);

	if (engine.getThrustValue() > EPS) {
		std::cout << "Out of fuel fixed thrust rocket engine still has non-zero thrust" << std::endl;
		testFailed = true;
	}

	if (engine.enable()) {
		std::cout << "It\'s possible to enable fixed thrust rocket engine at least twice" << std::endl;
		testFailed = true;
	}

	if (testFailed) {
		std::cout << "Fixed thrust rocket engine test failed" << std::endl;
	}
	else {
		std::cout << "Fixed thrust rocket engine test passed" << std::endl;
	}
}

void RocketTest() {
	bool testFailed = false;

	double rocketMass = 1000.0;
	double rocketHeight = 10.0;
	double rocketRadius = 1.0;

	double cylinderIntertiaTensorItemXY = (1.0 / 12.0) * rocketMass * \
		(3 * rocketRadius * rocketRadius + rocketHeight * rocketHeight);

	double cylinderIntertiaTensorItemZ = rocketMass * rocketRadius * rocketRadius / 2;

	Matrix rocketIntertiaTensor(
		cylinderIntertiaTensorItemXY, 0.0, 0.0,
		0.0, cylinderIntertiaTensorItemXY, 0.0,
		0.0, 0.0, cylinderIntertiaTensorItemZ
	);

	Rocket rocket(rocketIntertiaTensor, rocketMass, Vector(100.0, 0.0, 0.0), Vector());

	double fuelTankCapacity = 10.0;
	int fuelTank = rocket.addFuelTank(fuelTankCapacity);

	double engineThrust = 100.0;
	double engineFuelConsumption = 1.0;
	double timeToBurnOutAllFuel = fuelTankCapacity / engineFuelConsumption;

	rocket.addVariableThrustEngine("engine1", Vector(0.0, 0.0, -rocketHeight / 2.0), Vector(0.0, 0.0, -1.0), engineThrust, fuelTank, engineFuelConsumption);

	if (!rocket.setEngineFuelComsumption("engine1", engineFuelConsumption)) {
		testFailed = true;
		std::cout << "Can\'t set variable thrust engine fuel consumption" << std::endl;
	}

	if (!rocket.enableEngine("engine1")) {
		testFailed = true;
		std::cout << "Can\'t enable newer used variable thrust engine" << std::endl;
	}



	rocket.update(timeToBurnOutAllFuel);

	Vector rocketVelocityAfterAllFuelWasBurned = rocket.getVelocity();
	//std::cout << rocketVelocityAfterAllFuelWasBurned.z << ' ' << engineThrust / rocketMass * timeToBurnOutAllFuel << std::endl;
	if (std::fabs(rocketVelocityAfterAllFuelWasBurned.z - engineThrust / rocketMass * timeToBurnOutAllFuel) > EPS) {
		testFailed = true;
		std::cout << "Unexpected rocket velocity..." << std::endl;
	}

	rocket.update(timeToBurnOutAllFuel);

	if (std::fabs(rocketVelocityAfterAllFuelWasBurned.z - rocket.getVelocity().z) > EPS) {
		//std::cout << rocket.getVelocity().z << ' ' << rocketVelocityAfterAllFuelWasBurned.z << std::endl;
		testFailed = true;
		std::cout << "Engines work after all the fuel was burned out" << std::endl;
	}

	if (testFailed) {
		std::cout << "Rocket test failed" << std::endl;
	}
	else {
		std::cout << "Rocket test passed" << std::endl;
	}
}

int main() {
	FixedThrustRocketEngineTest();
	RocketTest();
	return 0;
}