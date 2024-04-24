#include <string>

#define AGE_FACTOR 0.2 

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

    /// @brief Get the Conjunction Level between this mass function and another one for a given state
    /// @details This is more a proof of concept than a real useful function due to its poor performance. Use `computeConjunctionLevels` instead.
    /// @param other  The other mass function
    /// @param state  The state for which we want to compute the conjunction level
    /// @return float  The conjunction level
    float getConjunctionLevel(const BeliefMassFunction &other, State state) const;
    State getState() const;

    float getOccupancyProbability() const;
    float getFreeProbability() const;

    /// @brief Compute the conjunction levels between two arrays of 8 mass functions.
    /// @details This function does the same thing as `getConjunctionLevel` but for all possible states and using SIMD (AVX2) instructions for better performance.
    /// @param a The first array of mass functions.
    /// @param b The second array of mass functions.
    static void computeConjunctionLevels(BeliefMassFunction *[8], const BeliefMassFunction *[8]);

    static void computeProbabilitiesScaled(const BeliefMassFunction *[8], float [2][8], float);

    void considerAge(float update_time);

    static void considerAges(float update_time, BeliefMassFunction *[8]);
    // - - Operators
    BeliefMassFunction &operator+=(BeliefMassFunction const &other);
    BeliefMassFunction operator+(const BeliefMassFunction &other) const;

private:
    // - Attributes
    float masses[4];
    float last_update;
};