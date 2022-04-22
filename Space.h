#ifndef TD_SPACEROCKETS_H_
#define TD_SPACEROCKETS_H_

#define EPS 1e-9

struct Vector {
	double x;
	double y;
	double z;

	Vector() :
		x(0.0), y(0.0), z(0.0) {}                                   //конструктор по умолчанию

	inline Vector(double x, double y, double z) :
		x(x), y(y), z(z) {}                                        //для встраивания тела функции вместо ее вызова

	inline const double length() const {
		return sqrt(x * x + y * y + z * z);                        //длина вектора (для встраивания тела функции вместо ее вызова)
	}

	void normalize() {                                             //нормируем
		double l = length();

		if (l > EPS) {
			x /= l;
			y /= l;
			z /= l;
		}
	}
};

Vector operator+ (Vector const& lhv, Vector const& rhv);

Vector operator* (Vector const& lhv, double rhv);

Vector operator* (double lhv, Vector const& rhv);

struct Matrix {                                                      //для работы с матрицей (тензором инерции) 
	Matrix() {                             //конструктор             //угловое ускорение (вектор) = обратная матрица тензора инерции (матрица) * момент силы (вектор)
		for (int i = 0; i < 9; ++i)
			data[i] = 0;
	}

	Matrix(double a00, double a01, double a02,
		double a10, double a11, double a12,
		double a20, double a21, double a22) {
		data[0] = a00; data[1] = a01; data[2] = a02;
		data[3] = a10; data[4] = a11; data[5] = a12;
		data[6] = a20; data[7] = a21; data[8] = a22;
	}

	/*
		( 0 1 2 )
		( 3 4 5 )
		( 6 7 8 )
	*/
	double data[9];                                                 //для каждой ракеты новая матрица

	void transpose() {
		std::swap(data[1], data[3]);
		std::swap(data[2], data[6]);
		std::swap(data[5], data[7]);
	}

	const double det() const {
		return data[0] * (data[4] * data[8] - data[5] * data[7]) -
			data[1] * (data[3] * data[8] - data[5] * data[6]) +
			data[2] * (data[3] * data[7] - data[4] * data[6]);
	}
};

Matrix operator/ (Matrix const m, double v);

Vector operator* (Matrix const m, Vector const& v);

Matrix Adj(Matrix const& m);

Matrix Inverce(Matrix const& m);

Vector Cross(Vector const& lhv, Vector const& rhv);

class FuelTank final {                                               //Топливный бак (запрет наследования от этого класса)
public:
	FuelTank(int tankNumber, double maxFuelAmount) :                 //наш конструктор (копирующий конструктор)
		tankNumber(tankNumber), maxFuelAmount(maxFuelAmount), currentFuelAmount(maxFuelAmount) {}

	const bool consumeFuel(double amount);                         //потребление топлива: для того, чтобы делать проверку              //!нельзя трогать

	double getMaxFuelAmount() const {                             //! плохо не будет
		return maxFuelAmount;
	}
	double getCurrentFuelAmount() const {                   //! тот, кто следит знает лучше, сколько текущего топлива
		return currentFuelAmount;
	}
	int getNumber() const {                                 //! тот, кто шарит - может исправить
		return tankNumber;
	}

private:
	int tankNumber;
	double maxFuelAmount;
	double currentFuelAmount;
};

class FuelTanksController final {                                     //Администратор топливных баков (запрет на наследование)
public:
	FuelTanksController() = default;                                  //Конструктор по умолчанию

	int addTank(double maxFuelAmount);                          // у нас же есть и другие баки... пусть всегда есть...

	const bool consumeFuelFromTank(int tankNumber, double fuelAmount); //проверка на "неудачно" указанный номер бака, если проверка пройдена

	const bool isTankNumberValid(int tankNumber) {
		return tankNumber < fuelTanks.size();
	}

private:
	std::vector<FuelTank> fuelTanks;                                    //вектор <тип> для создания нового бака...
};


class RocketEngine {                                                    //ракетный двигатель (затем двигатели с не/регулируемой тягой)
public:
	RocketEngine(std::string const& name, Vector const& position, Vector const& thrustNormal, double maxThrust) :
		name(name), position(position), thrustNormal(thrustNormal), currentThrust(0.0), maxThrust(maxThrust) {}

	virtual const bool enable() = 0;                                    //флажки для включения/выключения (переопределим уже для каждого типа двигателя)
	virtual const bool disable() = 0;
	virtual void update(double dt) = 0;                                 //обновление: слежка за изменением параметров при космическом полёте от времени

	double getThrustValue() const {
		return currentThrust;
	}
	double getMaxThrustValue() const {
		return maxThrust;
	}
	Vector getThrustVector() const {                             //тяга = нормальный вектр тяги * тяга на данный момент
		return thrustNormal * currentThrust;
	}
	Vector getThrustNormal() const {
		return thrustNormal;
	}
	Vector getPosition() const {
		return position;
	}

	std::string const getName() const {
		return name;
	}

protected:
	void setThrustValue(double thrust) {                                 //только для наследников (текущее топливо согласно расходу наследников)
		currentThrust = thrust;
	}

private:
	std::string name;
	Vector position;
	Vector thrustNormal;                                                 //надо же следить за баками
	double currentThrust;
	double maxThrust;
};

class VariableThrustRocketEngine final : public RocketEngine {           //двигатель  регулируемой тягой
public:
	VariableThrustRocketEngine(std::string const& name, Vector const& position, Vector const& thrustNormal, double maxThrust, FuelTanksController& fuelTanksController,
		int tankNumber, double maxFuelConsumption) :
		RocketEngine(name, position, thrustNormal, maxThrust),
		fuelTanksController(fuelTanksController),
		tankNumber(tankNumber),
		maxFuelConsumption(maxFuelConsumption),
		currentFuelConsumption(0.0) {}

	const bool enable() override {                                       //метод из RocketEngine: включение с заданной тягой
		setThrustValue(getMaxThrustValue() * (currentFuelConsumption / maxFuelConsumption));
		return true;
	}
	const bool disable() override {                                      //метод из RocketEngine: нет тяги
		setThrustValue(0.0);
		return true;
	}

	void update(double dt) override {                                    //метод из RocketEngine: проверка на наличие топлива в баке+ существование бака с заданым номером
		if (!fuelTanksController.consumeFuelFromTank(tankNumber, currentFuelConsumption * dt)) {
			setThrustValue(0.0);
		}
	}

	const bool setFuelConsumption(double fuelConsumption) {              //контроль потребления топлива
		if (fuelConsumption > maxFuelConsumption) {
			return false;
		}

		currentFuelConsumption = fuelConsumption;
		return true;
	}

private:
	FuelTanksController& fuelTanksController;                           //пользуемся методом из администратора (ссылка, так как находимся в наследнике)
	int tankNumber;
	double maxFuelConsumption;
	double currentFuelConsumption;
};

class FixedThrustRocketEngine final : public RocketEngine {             //Нерегулируемый двигатель
public:
	FixedThrustRocketEngine(std::string const& name, Vector const& position, Vector const& thrustNormal, double maxThrust, double runningDuration) :
		RocketEngine(name, position, thrustNormal, maxThrust),
		runningDuration(runningDuration), state(READY) {}

	const bool enable() override {                                      //включение двигателя 
		if (state == READY) {
			setThrustValue(getMaxThrustValue());                        //тяга не меняется
			state = RUNNING;
			return true;
		}
		else {
			return false;
		}
	}

	const bool disable() override {                                     // выключить после вкл. невозможно
		return false;
	}

	void update(double dt) override {                                   // должен быть включен+ время работы двигателя
		if (state != RUNNING)
			return;

		if (dt > runningDuration) {                                        //dt - время работы двигателя
			setThrustValue(0.0);
			runningDuration = 0.0;
			state = FINISHED;
		}
		else {
			runningDuration -= dt;
		}
	}

private:
	enum State {                                                           //для состояний готов/включен/отработал
		READY,
		RUNNING,
		FINISHED,
	};

	double runningDuration;                                                //длительность работы после вкл.
	State state;                                                           //состояние из готов/включен/отработал
};

class Rocket final {                                                       //Космический корабль
public:
	Rocket(Matrix const& intertiaTensor, double mass, Vector position, Vector angle) :
		invInertiaTensor(Inverce(intertiaTensor)), mass(mass), position(position), angle(angle) {
		velocity.x = 18.167;                                               //больше первой космической для данных
	}

	const bool addVariableThrustEngine(std::string const& name, Vector const& position, Vector const& thrustNormal, double maxThrust, int tankNumber, double maxFuelConsumption);

	const bool addFixedThrustEngine(std::string const& name, Vector const& position, Vector const& thrustNormal, double maxThrust, double runningDuration);

	const int addFuelTank(double fuelCapacity) {
		return fuelTanksController.addTank(fuelCapacity);                    //создание бака с топливом
	}

	const bool enableEngine(std::string const& name);                       //включение наших двигателей

	const bool disableEngine(std::string const& name);                      //выключение двгателей

	const bool setEngineFuelComsumption(std::string const& name, double fuelConsumption);

	void applyForce(Vector const& force) {
		outherForce = force;                                                   //сила притяжения планеты
	}

	void update(double dt);

	const Vector getPosition() const {
		return position;
	}
	const Vector getVelocity() const {
		return velocity;
	}

private:
	const bool isEngineNameUnique(std::string const& name) {                     //проверка на "неудачные" названия двигателей
		for (int i = 0; i < variableThrustRocketEngines.size(); ++i) {
			if (variableThrustRocketEngines[i].getName() == name)
				return false;
		}
		for (int i = 0; i < fixedThrustRocketEngines.size(); ++i) {
			if (fixedThrustRocketEngines[i].getName() == name)
				return false;
		}
		return true;
	}

	const bool isTankNumberValid(int tankNumber) {                         //проверка на "неудачный" номер бака
		return fuelTanksController.isTankNumberValid(tankNumber);
	}

	FuelTanksController fuelTanksController;                               //для проверки баков
	std::vector<VariableThrustRocketEngine> variableThrustRocketEngines;   //вектор <тип> для создания нового двигателя с рег. тягой...
	std::vector<FixedThrustRocketEngine> fixedThrustRocketEngines;         //вектор <тип> для создания нового нерегулируемого двигателя...

	Vector outherForce;                                                    //сила притяжения планеты
	Vector position;

	Vector velocity;
	Vector angularVelocity;                                                //угловая скорость

	Vector angle;
	Matrix invInertiaTensor;                                               //обратная матрица тензора инерции

	double mass;
};
#endif