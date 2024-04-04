

class BeliefMassFunction
{
public:
    enum class State
    {
        FREE = 0, // F
        OCCUPIED, // O
        UNKNOWN,  // Omega
        CONFLICT, // Empty set
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
    inline void setMass(State state, float mass)
    {
        this->masses[static_cast<int>(state)] = mass;
    }

    // - - Getters
    inline float getMass(State state) const
    {
        return this->masses[static_cast<int>(state)];
    }
    float getConjunctionLevel(const BeliefMassFunction &other, State state) const;
    State getState() const;
    float getOccupancyProbability() const;
    float getFreeProbability() const;
    // Optimized version of computeConjunctionLevels using AVX2
    static void computeConjunctionLevels(BeliefMassFunction* [8], const BeliefMassFunction* [8]);  


    // - - Operators
    BeliefMassFunction &operator+=(BeliefMassFunction const &other);
    BeliefMassFunction operator+(const BeliefMassFunction &other) const;

private:
    // - Attributes
    float masses[4];
};