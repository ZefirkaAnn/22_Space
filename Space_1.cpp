#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <utility>

#include"Space.h"

Vector operator+ (Vector const& lhv, Vector const& rhv) {     //������� ��������� ������� (������������ ������)
	return Vector(lhv.x + rhv.x, lhv.y + rhv.y, lhv.z + rhv.z);
}
Vector operator* (Vector const& lhv, double rhv) {            //������� �������� ������� �� ������ (����� � ������)
	return Vector(lhv.x * rhv, lhv.y * rhv, lhv.z * rhv);
}
Vector operator* (double lhv, Vector const& rhv) {
	return rhv * lhv;
}

Matrix operator/ (Matrix const m, double v) {                 //������� ������� �� ����� (������������) - ������� �������� �������        
	Matrix r = m;
	for (int i = 0; i < 9; ++i) {
		r.data[i] = m.data[i] / v;
	}
	return r;
}

Vector operator* (Matrix const m, Vector const& v) {          //������� �������� ������� �� ������
	/*
		( 0 1 2 ) * (x)
		( 3 4 5 ) * (y)
		( 6 7 8 ) * (z)
	*/

	return Vector(
		m.data[0] * v.x + m.data[1] * v.y + m.data[2] * v.z,
		m.data[3] * v.x + m.data[4] * v.y + m.data[5] * v.z,
		m.data[6] * v.x + m.data[7] * v.y + m.data[8] * v.z
	);
}

Matrix Adj(Matrix const& m) {                                 //���������� �������� ������� * ������������
	Matrix adj;

	adj.data[0] = m.data[4] * m.data[8] - m.data[5] * m.data[7];
	adj.data[1] = -(m.data[3] * m.data[8] - m.data[5] * m.data[6]);
	adj.data[2] = m.data[3] * m.data[7] - m.data[4] * m.data[6];
	adj.data[3] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7]);
	adj.data[4] = m.data[0] * m.data[8] - m.data[6] * m.data[2];
	adj.data[5] = -(m.data[0] * m.data[7] - m.data[6] * m.data[1]);
	adj.data[6] = m.data[1] * m.data[5] - m.data[4] * m.data[2];
	adj.data[7] = -(m.data[0] * m.data[5] - m.data[3] * m.data[2]);
	adj.data[8] = m.data[0] * m.data[4] - m.data[3] * m.data[1];

	return adj;
}

Matrix Inverce(Matrix const& m) {                              //�������� �������
	double det = m.det();

	Matrix adj = Adj(m);
	adj.transpose();

	return adj / det;
}

Vector Cross(Vector const& lhv, Vector const& rhv) {            //������� ������� ��������� ������������
	/*
		( i  j  k  )
		( x1 y1 z1 )
		( x2 y2 z2 )

		x = y1 * z2 - z1 * y2
		y = z1 * x2 - x1 * z2
		z = x1 * z2 - x2 * z1
	*/
	return Vector(
		lhv.y * rhv.z - lhv.z * rhv.y,
		lhv.z * rhv.x - lhv.x * rhv.z,
		lhv.x * rhv.z - lhv.z * rhv.x
	);
}

const bool FuelTank::consumeFuel(double amount) {                          //����������� �������: ��� ����, ����� ������ �������� 
	if (currentFuelAmount < amount) {                            //�� ���������� ������� � ����: ���� ���� (���), �� ��������� ��
		currentFuelAmount = 0;                                   //amount - ���-�� �������������� ����� (���� � ����������)
		return false;
	}

	currentFuelAmount -= amount;
	return true;
}

int FuelTanksController::addTank(double maxFuelAmount) {    //� ��� �� ���� � ������ ����... ����� ������ ����...
	int nextTankNumber = fuelTanks.size();                        //����� ���������� ���� - ������ �������
	fuelTanks.push_back(FuelTank(nextTankNumber, maxFuelAmount)); //��� � �������� � ����� ������� (����� ������ ����...)
	return nextTankNumber;
}

const bool FuelTanksController::consumeFuelFromTank(int tankNumber, double fuelAmount) { //�������� �� "��������" ��������� ����� ����, ���� �������� ��������
	if (!isTankNumberValid(tankNumber)) {                           // �� ���� ��������� �� ����������� �������
		return false;
	}

	return fuelTanks[tankNumber].consumeFuel(fuelAmount);           //��������������� ����������� ���������
}

const bool Rocket::addVariableThrustEngine(std::string const& name, Vector const& position, Vector const& thrustNormal, double maxThrust, int tankNumber, double maxFuelConsumption) {
	if (!isEngineNameUnique(name) || !isTankNumberValid(tankNumber)) {
		return false;
	}                                                                   //�������� ��������� � ������������ �����

	VariableThrustRocketEngine engine(name, position, thrustNormal, maxThrust, fuelTanksController, tankNumber, maxFuelConsumption);
	variableThrustRocketEngines.push_back(engine);
	return true;
}

const bool Rocket::addFixedThrustEngine(std::string const& name, Vector const& position, Vector const& thrustNormal, double maxThrust, double runningDuration) {
	if (!isEngineNameUnique(name)) {
		return false;
	}                                                                    //�������� ��������������� ���������

	FixedThrustRocketEngine engine(name, position, thrustNormal, maxThrust, runningDuration);
	fixedThrustRocketEngines.push_back(engine);
	return true;
}

const bool Rocket::enableEngine(std::string const& name) {                       //��������� ����� ����������
	for (auto& engine : variableThrustRocketEngines) {                   //��� �� ������ �� ������ ����������, ������� ������
		if (engine.getName() == name) {
			return engine.enable();
		}
	}
	for (auto& engine : fixedThrustRocketEngines) {
		if (engine.getName() == name) {
			return engine.enable();
		}
	}

	return false;
}

const bool Rocket::disableEngine(std::string const& name) {                       //���������� ���������
	for (auto& engine : variableThrustRocketEngines) {
		if (engine.getName() == name) {
			return engine.disable();
		}
	}
	for (auto& engine : fixedThrustRocketEngines) {
		if (engine.getName() == name) {
			return engine.disable();
		}
	}

	return false;
}

const bool Rocket::setEngineFuelComsumption(std::string const& name, double fuelConsumption) {
	for (auto& engine : variableThrustRocketEngines) {
		if (engine.getName() == name) {                                   //���������� ����� ����������� ������� �� ������������� ���������
			return engine.setFuelConsumption(fuelConsumption);            //���������������, ����� ��� ���� ������ ����������� ����������
		}
	}
	return false;
}

void Rocket::update(double dt) {
	//std::cout << velocity.x << ' ' << velocity.y << std::endl;
	Vector M;                                                              //��������� ������ ����
	Vector F;                                                              //��������� ����
	//std::cout << F.x << ' ' << F.y << std::endl;

	for (int i = 0; i < variableThrustRocketEngines.size(); ++i) {
		variableThrustRocketEngines[i].update(dt);

		M = M + Cross(variableThrustRocketEngines[i].getPosition(), variableThrustRocketEngines[i].getThrustVector());  //M=[r*F], ���������� ������� �� �������, ��� ��� ��� �����
		F = F + variableThrustRocketEngines[i].getThrustVector();
	}

	for (int i = 0; i < fixedThrustRocketEngines.size(); ++i) {
		fixedThrustRocketEngines[i].update(dt);

		M = M + Cross(fixedThrustRocketEngines[i].getPosition(), fixedThrustRocketEngines[i].getThrustVector());
		F = F + fixedThrustRocketEngines[i].getThrustVector();
	}

	F = F + outherForce;                                                     //��� ����, ����������� �� ������

	Vector angularAcceleration = invInertiaTensor * M;                       //�� ������ ������ ����������
	Vector acceleration = F * (-1.0 / mass);                                 //����������� ��������� (����� �������� ��� ����������� ��������)

	angularVelocity = angularVelocity + angularAcceleration * dt;            //������� ���������*dt = ������� ��������
	velocity = velocity + acceleration * dt;                                 //���������*dt + ��������� �������� = ������� ��������

	angle = angle + (angularVelocity + angularAcceleration * dt * 0.5) * dt; //������� ���� = ���.����+w*t+e*dt^2/2
	position = position + (velocity + acceleration * dt * 0.5) * dt;         //������� ��������� = ��������� ����������+v*t+a*t^2/2
}