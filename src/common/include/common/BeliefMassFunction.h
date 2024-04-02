

class BeliefMassFunction
{
public:
    enum class State
    {
        FREE = 0,   // F
        OCCUPIED,   // O
        UNKNOWN,    // Omega
        CONFLICT,   // Empty set
    };

    // - Methods
    // - - Constructors
    // Default state for EOGM (unknown)
    BeliefMassFunction();
    // From lidar
    BeliefMassFunction(State state, float mass);
    // - - Destructor
    ~BeliefMassFunction() = default;

    // - - Setters
    void setMass(State state, float mass);

    // - - Getters
    float getConjunctionLevel(const BeliefMassFunction& other, State state) const;
    float getMass(State state) const;
    State getState() const;
    float getOccupancyProbability() const;

    // - - Operators
    BeliefMassFunction &operator+=(BeliefMassFunction const &other);
    BeliefMassFunction operator+(const BeliefMassFunction &other) const;

private:
    // - Attributes
    float masses[4];
};