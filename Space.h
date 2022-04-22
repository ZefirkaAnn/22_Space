#ifndef TD_SPACEROCKETS_H_
#define TD_SPACEROCKETS_H_

#define EPS 1e-9

struct Vector {
	double x;
	double y;
	double z;

	Vector() :
		x(0.0), y(0.0), z(0.0) {}                                   //����������� �� ���������

	inline Vector(double x, double y, double z) :
		x(x), y(y), z(z) {}                                        //��� ����������� ���� ������� ������ �� ������

	inline const double length() const {
		return sqrt(x * x + y * y + z * z);                        //����� ������� (��� ����������� ���� ������� ������ �� ������)
	}

	void normalize() {                                             //���������
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

struct Matrix {                                                      //��� ������ � �������� (�������� �������) 
	Matrix() {                             //�����������             //������� ��������� (������) = �������� ������� ������� ������� (�������) * ������ ���� (������)
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
	double data[9];                                                 //��� ������ ������ ����� �������

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

class FuelTank final {                                               //��������� ��� (������ ������������ �� ����� ������)
public:
	FuelTank(int tankNumber, double maxFuelAmount) :                 //��� ����������� (���������� �����������)
		tankNumber(tankNumber), maxFuelAmount(maxFuelAmount), currentFuelAmount(maxFuelAmount) {}

	const bool consumeFuel(double amount);                         //����������� �������: ��� ����, ����� ������ ��������              //!������ �������

	double getMaxFuelAmount() const {                             //! ����� �� �����
		return maxFuelAmount;
	}
	double getCurrentFuelAmount() const {                   //! ���, ��� ������ ����� �����, ������� �������� �������
		return currentFuelAmount;
	}
	int getNumber() const {                                 //! ���, ��� ����� - ����� ���������
		return tankNumber;
	}

private:
	int tankNumber;
	double maxFuelAmount;
	double currentFuelAmount;
};

class FuelTanksController final {                                     //������������� ��������� ����� (������ �� ������������)
public:
	FuelTanksController() = default;                                  //����������� �� ���������

	int addTank(double maxFuelAmount);                          // � ��� �� ���� � ������ ����... ����� ������ ����...

	const bool consumeFuelFromTank(int tankNumber, double fuelAmount); //�������� �� "��������" ��������� ����� ����, ���� �������� ��������

	const bool isTankNumberValid(int tankNumber) {
		return tankNumber < fuelTanks.size();
	}

private:
	std::vector<FuelTank> fuelTanks;                                    //������ <���> ��� �������� ������ ����...
};


class RocketEngine {                                                    //�������� ��������� (����� ��������� � ��/������������ �����)
public:
	RocketEngine(std::string const& name, Vector const& position, Vector const& thrustNormal, double maxThrust) :
		name(name), position(position), thrustNormal(thrustNormal), currentThrust(0.0), maxThrust(maxThrust) {}

	virtual const bool enable() = 0;                                    //������ ��� ���������/���������� (������������� ��� ��� ������� ���� ���������)
	virtual const bool disable() = 0;
	virtual void update(double dt) = 0;                                 //����������: ������ �� ���������� ���������� ��� ����������� ����� �� �������

	double getThrustValue() const {
		return currentThrust;
	}
	double getMaxThrustValue() const {
		return maxThrust;
	}
	Vector getThrustVector() const {                             //���� = ���������� ����� ���� * ���� �� ������ ������
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
	void setThrustValue(double thrust) {                                 //������ ��� ����������� (������� ������� �������� ������� �����������)
		currentThrust = thrust;
	}

private:
	std::string name;
	Vector position;
	Vector thrustNormal;                                                 //���� �� ������� �� ������
	double currentThrust;
	double maxThrust;
};

class VariableThrustRocketEngine final : public RocketEngine {           //���������  ������������ �����
public:
	VariableThrustRocketEngine(std::string const& name, Vector const& position, Vector const& thrustNormal, double maxThrust, FuelTanksController& fuelTanksController,
		int tankNumber, double maxFuelConsumption) :
		RocketEngine(name, position, thrustNormal, maxThrust),
		fuelTanksController(fuelTanksController),
		tankNumber(tankNumber),
		maxFuelConsumption(maxFuelConsumption),
		currentFuelConsumption(0.0) {}

	const bool enable() override {                                       //����� �� RocketEngine: ��������� � �������� �����
		setThrustValue(getMaxThrustValue() * (currentFuelConsumption / maxFuelConsumption));
		return true;
	}
	const bool disable() override {                                      //����� �� RocketEngine: ��� ����
		setThrustValue(0.0);
		return true;
	}

	void update(double dt) override {                                    //����� �� RocketEngine: �������� �� ������� ������� � ����+ ������������� ���� � ������� �������
		if (!fuelTanksController.consumeFuelFromTank(tankNumber, currentFuelConsumption * dt)) {
			setThrustValue(0.0);
		}
	}

	const bool setFuelConsumption(double fuelConsumption) {              //�������� ����������� �������
		if (fuelConsumption > maxFuelConsumption) {
			return false;
		}

		currentFuelConsumption = fuelConsumption;
		return true;
	}

private:
	FuelTanksController& fuelTanksController;                           //���������� ������� �� �������������� (������, ��� ��� ��������� � ����������)
	int tankNumber;
	double maxFuelConsumption;
	double currentFuelConsumption;
};

class FixedThrustRocketEngine final : public RocketEngine {             //�������������� ���������
public:
	FixedThrustRocketEngine(std::string const& name, Vector const& position, Vector const& thrustNormal, double maxThrust, double runningDuration) :
		RocketEngine(name, position, thrustNormal, maxThrust),
		runningDuration(runningDuration), state(READY) {}

	const bool enable() override {                                      //��������� ��������� 
		if (state == READY) {
			setThrustValue(getMaxThrustValue());                        //���� �� ��������
			state = RUNNING;
			return true;
		}
		else {
			return false;
		}
	}

	const bool disable() override {                                     // ��������� ����� ���. ����������
		return false;
	}

	void update(double dt) override {                                   // ������ ���� �������+ ����� ������ ���������
		if (state != RUNNING)
			return;

		if (dt > runningDuration) {                                        //dt - ����� ������ ���������
			setThrustValue(0.0);
			runningDuration = 0.0;
			state = FINISHED;
		}
		else {
			runningDuration -= dt;
		}
	}

private:
	enum State {                                                           //��� ��������� �����/�������/���������
		READY,
		RUNNING,
		FINISHED,
	};

	double runningDuration;                                                //������������ ������ ����� ���.
	State state;                                                           //��������� �� �����/�������/���������
};

class Rocket final {                                                       //����������� �������
public:
	Rocket(Matrix const& intertiaTensor, double mass, Vector position, Vector angle) :
		invInertiaTensor(Inverce(intertiaTensor)), mass(mass), position(position), angle(angle) {
		velocity.x = 18.167;                                               //������ ������ ����������� ��� ������
	}

	const bool addVariableThrustEngine(std::string const& name, Vector const& position, Vector const& thrustNormal, double maxThrust, int tankNumber, double maxFuelConsumption);

	const bool addFixedThrustEngine(std::string const& name, Vector const& position, Vector const& thrustNormal, double maxThrust, double runningDuration);

	const int addFuelTank(double fuelCapacity) {
		return fuelTanksController.addTank(fuelCapacity);                    //�������� ���� � ��������
	}

	const bool enableEngine(std::string const& name);                       //��������� ����� ����������

	const bool disableEngine(std::string const& name);                      //���������� ���������

	const bool setEngineFuelComsumption(std::string const& name, double fuelConsumption);

	void applyForce(Vector const& force) {
		outherForce = force;                                                   //���� ���������� �������
	}

	void update(double dt);

	const Vector getPosition() const {
		return position;
	}
	const Vector getVelocity() const {
		return velocity;
	}

private:
	const bool isEngineNameUnique(std::string const& name) {                     //�������� �� "���������" �������� ����������
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

	const bool isTankNumberValid(int tankNumber) {                         //�������� �� "���������" ����� ����
		return fuelTanksController.isTankNumberValid(tankNumber);
	}

	FuelTanksController fuelTanksController;                               //��� �������� �����
	std::vector<VariableThrustRocketEngine> variableThrustRocketEngines;   //������ <���> ��� �������� ������ ��������� � ���. �����...
	std::vector<FixedThrustRocketEngine> fixedThrustRocketEngines;         //������ <���> ��� �������� ������ ��������������� ���������...

	Vector outherForce;                                                    //���� ���������� �������
	Vector position;

	Vector velocity;
	Vector angularVelocity;                                                //������� ��������

	Vector angle;
	Matrix invInertiaTensor;                                               //�������� ������� ������� �������

	double mass;
};
#endif