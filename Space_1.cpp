#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <utility>

#include"Space.h"

Vector operator+ (Vector const& lhv, Vector const& rhv) {     //научили сладывать вектора (использовала ссылки)
	return Vector(lhv.x + rhv.x, lhv.y + rhv.y, lhv.z + rhv.z);
}
Vector operator* (Vector const& lhv, double rhv) {            //научили умножать вектора на скаляр (слева и справа)
	return Vector(lhv.x * rhv, lhv.y * rhv, lhv.z * rhv);
}
Vector operator* (double lhv, Vector const& rhv) {
	return rhv * lhv;
}

Matrix operator/ (Matrix const m, double v) {                 //деление матрицы на число (определитель) - считаем обратную матрицу        
	Matrix r = m;
	for (int i = 0; i < 9; ++i) {
		r.data[i] = m.data[i] / v;
	}
	return r;
}

Vector operator* (Matrix const m, Vector const& v) {          //научили умножать матрицу на вектор
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

Matrix Adj(Matrix const& m) {                                 //вычисление обратной матрицы * определитель
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

Matrix Inverce(Matrix const& m) {                              //обратная матрица
	double det = m.det();

	Matrix adj = Adj(m);
	adj.transpose();

	return adj / det;
}

Vector Cross(Vector const& lhv, Vector const& rhv) {            //научили считать векторное произведение
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

const bool FuelTank::consumeFuel(double amount) {                          //потребление топлива: для того, чтобы делать проверку 
	if (currentFuelAmount < amount) {                            //на содержание топлива в баке: если есть (нет), то уменьшаем на
		currentFuelAmount = 0;                                   //amount - кол-во израсходанного сырья (флаг о завершении)
		return false;
	}

	currentFuelAmount -= amount;
	return true;
}

int FuelTanksController::addTank(double maxFuelAmount) {    //у нас же есть и другие баки... пусть всегда есть...
	int nextTankNumber = fuelTanks.size();                        //номер следующего бака - размер вектора
	fuelTanks.push_back(FuelTank(nextTankNumber, maxFuelAmount)); //вот и добавили с таким номером (пусть всегда есть...)
	return nextTankNumber;
}

const bool FuelTanksController::consumeFuelFromTank(int tankNumber, double fuelAmount) { //проверка на "неудачно" указанный номер бака, если проверка пройдена
	if (!isTankNumberValid(tankNumber)) {                           // то надо уменьшить на потраченное топливо
		return false;
	}

	return fuelTanks[tankNumber].consumeFuel(fuelAmount);           //воспользовалась перегрузкой оператора
}

const bool Rocket::addVariableThrustEngine(std::string const& name, Vector const& position, Vector const& thrustNormal, double maxThrust, int tankNumber, double maxFuelConsumption) {
	if (!isEngineNameUnique(name) || !isTankNumberValid(tankNumber)) {
		return false;
	}                                                                   //создание двигателя с регулируемой тягой

	VariableThrustRocketEngine engine(name, position, thrustNormal, maxThrust, fuelTanksController, tankNumber, maxFuelConsumption);
	variableThrustRocketEngines.push_back(engine);
	return true;
}

const bool Rocket::addFixedThrustEngine(std::string const& name, Vector const& position, Vector const& thrustNormal, double maxThrust, double runningDuration) {
	if (!isEngineNameUnique(name)) {
		return false;
	}                                                                    //создание нерегулируемого двигателя

	FixedThrustRocketEngine engine(name, position, thrustNormal, maxThrust, runningDuration);
	fixedThrustRocketEngines.push_back(engine);
	return true;
}

const bool Rocket::enableEngine(std::string const& name) {                       //включение наших двигателей
	for (auto& engine : variableThrustRocketEngines) {                   //как бы вектор из разных двигателей, поэтому ссылка
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

const bool Rocket::disableEngine(std::string const& name) {                       //выключение двгателей
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
		if (engine.getName() == name) {                                   //определяем объем потребления топлива от регулируемого двигателя
			return engine.setFuelConsumption(fuelConsumption);            //проконтролируем, чтобы оно было меньше максимально возможного
		}
	}
	return false;
}

void Rocket::update(double dt) {
	//std::cout << velocity.x << ' ' << velocity.y << std::endl;
	Vector M;                                                              //начальный момент силы
	Vector F;                                                              //начальная сила
	//std::cout << F.x << ' ' << F.y << std::endl;

	for (int i = 0; i < variableThrustRocketEngines.size(); ++i) {
		variableThrustRocketEngines[i].update(dt);

		M = M + Cross(variableThrustRocketEngines[i].getPosition(), variableThrustRocketEngines[i].getThrustVector());  //M=[r*F], притяжение планеты не считаем, так как нет плеча
		F = F + variableThrustRocketEngines[i].getThrustVector();
	}

	for (int i = 0; i < fixedThrustRocketEngines.size(); ++i) {
		fixedThrustRocketEngines[i].update(dt);

		M = M + Cross(fixedThrustRocketEngines[i].getPosition(), fixedThrustRocketEngines[i].getThrustVector());
		F = F + fixedThrustRocketEngines[i].getThrustVector();
	}

	F = F + outherForce;                                                     //все силы, действующие на ракету

	Vector angularAcceleration = invInertiaTensor * M;                       //из вывода формул получилось
	Vector acceleration = F * (-1.0 / mass);                                 //направление ускорения (минус подобран под придуманные значения)

	angularVelocity = angularVelocity + angularAcceleration * dt;            //угловое ускорение*dt = угловая скорость
	velocity = velocity + acceleration * dt;                                 //ускорение*dt + начальная скорость = текущая скорость

	angle = angle + (angularVelocity + angularAcceleration * dt * 0.5) * dt; //текущий угол = нач.угол+w*t+e*dt^2/2
	position = position + (velocity + acceleration * dt * 0.5) * dt;         //такущее положение = начальные координаты+v*t+a*t^2/2
}