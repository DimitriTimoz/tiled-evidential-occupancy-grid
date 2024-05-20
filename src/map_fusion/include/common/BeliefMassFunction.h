#include <string>

#define AGE_FACTOR 0.2 

/// @brief The Belief Mass Function class
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

    /// @brief Construct a new Belief Mass Function object with an unknown state.
    BeliefMassFunction();
    // From lidar

    /// @brief Construct a new Belief Mass Function object with a given state and mass.
    /// @param state
    /// @param mass
    /// @note This is usually used to convert laser scan data to a mass function.
    BeliefMassFunction(State state, float mass);

    // - - Destructor
    ~BeliefMassFunction() = default;

    // - - Setters

    /// @brief Set the mass for a given state
    /// @param state
    /// @param mass
    inline void setMass(State state, float mass)
    {
        this->masses[static_cast<int>(state)] = mass;
    }

    // - - Getters
    /// @brief Get the mass for a given state
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

    /// @brief Get the state of the mass function
    /// @return State
    State getState() const;

    /// @brief Get the occupancy probability of the mass function
    /// @return float
    float getOccupancyProbability() const;

    /// @brief Get the free probability of the mass function
    /// @return float
    float getFreeProbability() const;

    /// @brief Compute the conjunction levels between two arrays of 8 mass functions.
    /// @details This function does the same thing as `getConjunctionLevel` but for all possible states and using SIMD (AVX2) instructions for better performance.
    /// @param a The first array of mass functions.
    /// @param b The second array of mass functions.
    static void computeConjunctionLevels(BeliefMassFunction *[8], const BeliefMassFunction *[8]);

    /// @brief Compute the pignistic probability of a given state for a given mass function.
    /// @param a An array of 8 BeliefMassFunction.
    /// @param results An array of 8 floats to store the results.
    /// @param scale The scale of the pignistic probability (usually 1).
    static void computeProbabilitiesScaled(const BeliefMassFunction *a[8], float results[2][8], float scale);

    /// @brief Consider the age the last update of the mass function to attenuate the masses.
    /// @param new last update_time 
    void considerAge(float update_time);

    /// @brief Consider the age the last update of the mass functions to attenuate the masses.
    /// @param update_time The time of the last update.
    /// @param a An array of 8 BeliefMassFunction.
    static void considerAges(const float *update_time[8], BeliefMassFunction *a[8]);


private:
    // - Attributes

    /// @brief The masses of the mass function
    float masses[4];

    /// @brief The time of the last update
    float last_update = 0.0;
};
