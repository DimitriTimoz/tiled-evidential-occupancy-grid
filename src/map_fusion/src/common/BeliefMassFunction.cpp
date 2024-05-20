#include "common/BeliefMassFunction.h"
#include <immintrin.h>
#include <ros/ros.h>

BeliefMassFunction::BeliefMassFunction()
{
    this->setMass(State::FREE, 0.0);
    this->setMass(State::OCCUPIED, 0.0);
    this->setMass(State::UNKNOWN, 1.0);
    this->setMass(State::CONFLICT, 0.0);
}

BeliefMassFunction::BeliefMassFunction(State state, float mass) : BeliefMassFunction()
{
    switch (state)
    {
    case State::UNKNOWN:
        mass = 1;
    case State::FREE:
    case State::OCCUPIED:
        this->setMass(state, mass);
        this->setMass(State::UNKNOWN, 1 - mass);
        break;
    // - We cannot set the mass of a conflict state
    default:
        break;
    }
}

BeliefMassFunction::State BeliefMassFunction::getState() const
{
    int maximum_index = -1; // This should raise an runtime error if all mass values are equals to 0 which is not possible
    float maximum_mass = 0.0;
    for (int i = static_cast<int>(State::FREE); i < sizeof(BeliefMassFunction::masses) / sizeof(float); i++)
    {
        if (this->masses[i] > maximum_mass)
        {
            maximum_index = i;
            maximum_mass = this->masses[i];
        }
    }

    return static_cast<State>(maximum_index);
}

float BeliefMassFunction::getConjunctionLevel(const BeliefMassFunction &other, State state) const
{
    switch (state)
    {
    case State::FREE:
    case State::OCCUPIED:
        return this->getMass(state) * other.getMass(state) +
               this->getMass(State::UNKNOWN) * other.getMass(state) +
               this->getMass(state) * other.getMass(State::UNKNOWN);
    case State::UNKNOWN:
        return this->getMass(State::UNKNOWN) * other.getMass(State::UNKNOWN);
    case State::CONFLICT:
        return this->getMass(State::FREE) * other.getMass(State::OCCUPIED) +
               this->getMass(State::OCCUPIED) * other.getMass(State::FREE);
    }
}

/// @brief Load 8 masses from an array of 8 BeliefMassFunction
#define VECTORIZE_MASSES(a, state) _mm256_set_ps(a[0]->getMass(state), a[1]->getMass(state), a[2]->getMass(state), a[3]->getMass(state), a[4]->getMass(state), a[5]->getMass(state), a[6]->getMass(state), a[7]->getMass(state))

void BeliefMassFunction::computeConjunctionLevels(BeliefMassFunction *a[8], const BeliefMassFunction *b[8])
{
    // - Compute conjunction level of free : (F_1 * F_2) + (F_1 * Omega_2) + (Omega_1 * F_2)

    __m256 a_vector = VECTORIZE_MASSES(a, State::FREE);     // Load all 8 free masses of a
    __m256 b_vector = VECTORIZE_MASSES(b, State::FREE);     // Load all 8 free masses of b
    __m256 free_result = _mm256_mul_ps(a_vector, b_vector); // F_1 * F_2

    a_vector = VECTORIZE_MASSES(a, State::FREE);                    // Load all 8 free masses of a
    b_vector = VECTORIZE_MASSES(b, State::UNKNOWN);                 // Load all 8 unknown masses of b
    free_result = _mm256_fmadd_ps(a_vector, b_vector, free_result); // (F_1 * Omega_2) + (F_1 * F_2)

    a_vector = VECTORIZE_MASSES(a, State::UNKNOWN);                 // Load all 8 unknown masses of a
    b_vector = VECTORIZE_MASSES(b, State::FREE);                    // Load all 8 free masses of b
    free_result = _mm256_fmadd_ps(a_vector, b_vector, free_result); // (Omega_1 * F_2) + (F_1 * Omega_2) + (F_1 * F_2)

    // - Compute conjunction level of occupied : (O_1 * O_2) + (O_1 * Omega_2) + (Omega_1 * O_2)

    a_vector = VECTORIZE_MASSES(a, State::OCCUPIED);            // Load all 8 occupied masses of a
    b_vector = VECTORIZE_MASSES(b, State::OCCUPIED);            // Load all 8 occupied masses of b
    __m256 occupied_result = _mm256_mul_ps(a_vector, b_vector); // O_1 * O_2

    a_vector = VECTORIZE_MASSES(a, State::OCCUPIED);                        // Load all 8 occupied masses of a
    b_vector = VECTORIZE_MASSES(b, State::UNKNOWN);                         // Load all 8 unknown masses of b
    occupied_result = _mm256_fmadd_ps(a_vector, b_vector, occupied_result); // (O_1 * Omega_2) + (O_1 * O_2)

    a_vector = VECTORIZE_MASSES(a, State::UNKNOWN);                         // Load all 8 unknown masses of a
    b_vector = VECTORIZE_MASSES(b, State::OCCUPIED);                        // Load all 8 occupied masses of b
    occupied_result = _mm256_fmadd_ps(a_vector, b_vector, occupied_result); // (Omega_1 * O_2) + (O_1 * Omega_2) + (O_1 * O_2)

    // - Compute conjunction level of conflict : (F_1 * O_2) + (O_1 * F_2)

    a_vector = VECTORIZE_MASSES(a, State::FREE);                // Load all 8 free masses of a
    b_vector = VECTORIZE_MASSES(b, State::OCCUPIED);            // Load all 8 occupied masses of b
    __m256 conflict_result = _mm256_mul_ps(a_vector, b_vector); // F_1 * O_2

    a_vector = VECTORIZE_MASSES(a, State::OCCUPIED);                        // Load all 8 occupied masses of a
    b_vector = VECTORIZE_MASSES(b, State::FREE);                            // Load all 8 free masses of b
    conflict_result = _mm256_fmadd_ps(a_vector, b_vector, conflict_result); // (O_1 * F_2) + (F_1 * O_2)

    // - Unknown

    a_vector = VECTORIZE_MASSES(a, State::UNKNOWN);            // Load all 8 unknown masses of a
    b_vector = VECTORIZE_MASSES(b, State::UNKNOWN);            // Load all 8 unknown masses of b
    __m256 unknown_result = _mm256_mul_ps(a_vector, b_vector); // Omega_1 * Omega_2

    // - Normalize

    // - - Sum everything up for normalization

    __m256 sum = _mm256_add_ps(free_result, occupied_result); // Sum of free and occupied
    sum = _mm256_add_ps(sum, conflict_result);                // Sum of free, occupied and conflict
    sum = _mm256_add_ps(sum, unknown_result);                 // Sum of free, occupied, conflict and unknown

    // - - Apply normalization

    free_result = _mm256_div_ps(free_result, sum);         // free_result = free_result / sum
    occupied_result = _mm256_div_ps(occupied_result, sum); // occupied_result = occupied_result / sum
    conflict_result = _mm256_div_ps(conflict_result, sum); // conflict_result = conflict_result / sum
    unknown_result = _mm256_div_ps(unknown_result, sum);   // unknown_result = unknown_result / sum

    // - Store results

    alignas(32) float results[4][8];
    _mm256_store_ps(results[0], free_result);
    _mm256_store_ps(results[1], occupied_result);
    _mm256_store_ps(results[2], conflict_result);
    _mm256_store_ps(results[3], unknown_result);

    // - Set the masses
    for (int i = 0; i < 8; i++)
    {
        int j = 7 - i;
        a[j]->setMass(State::FREE, results[0][i]);
        a[j]->setMass(State::OCCUPIED, results[1][i]);
        a[j]->setMass(State::CONFLICT, results[2][i]);
        a[j]->setMass(State::UNKNOWN, results[3][i]);
    }
}

float BeliefMassFunction::getOccupancyProbability() const
{
    return this->getMass(State::OCCUPIED) / (1 - this->getMass(State::CONFLICT));
}

float BeliefMassFunction::getFreeProbability() const
{
    return this->getMass(State::FREE) / (1 - this->getMass(State::CONFLICT));
}

void BeliefMassFunction::computeProbabilitiesScaled(const BeliefMassFunction *a[8], float results[2][8], float scale)
{
    __m256 conflict = VECTORIZE_MASSES(a, State::CONFLICT); // Load all 8 conflict masses
    __m256 s = _mm256_set1_ps(scale);                       // Set all 8 floats to the same value (scale)
    conflict = _mm256_fnmadd_ps(conflict, s, s);            // conflict = scale - conflict * scale

    __m256 free = VECTORIZE_MASSES(a, State::FREE);         // Load all 8 free masses
    __m256 occupied = VECTORIZE_MASSES(a, State::OCCUPIED); // Load all 8 occupied masses

    free = _mm256_div_ps(free, conflict);         // free = free / conflict
    occupied = _mm256_div_ps(occupied, conflict); // occupied = occupied / conflict

    _mm256_store_ps(results[0], free);     // Store the 8 free probabilities in the results array
    _mm256_store_ps(results[1], occupied); // Store the 8 occupied probabilities in the results array
}

void BeliefMassFunction::considerAge(float update_time) {
    float age = update_time - this->last_update;
    float alpha = exp2(-age/AGE_FACTOR);
    this->masses[0] *= alpha;
    this->masses[1] *= alpha;
    this->masses[2] *= alpha;
    this->masses[3] = this->masses[3] * alpha + (1 - alpha);
    this->last_update = update_time;
}

// Placeholder for exponential calculation
__m256 exp256_ps(__m256 x);

// Like the previous function but with AVX2 instructions
void BeliefMassFunction::considerAges(const float *update_time[8], BeliefMassFunction *a[8])
{
    __m256 occupancy = VECTORIZE_MASSES(a, State::OCCUPIED);
    __m256 free = VECTORIZE_MASSES(a, State::FREE);
    __m256 unknown = VECTORIZE_MASSES(a, State::UNKNOWN);
    __m256 conflict = VECTORIZE_MASSES(a, State::CONFLICT);

    // Correctly load from an array of pointers to floats
    __m256 age = _mm256_load_ps(update_time[0]);
    __m256 last_update = _mm256_set1_ps(a[0]->last_update);
    age = _mm256_sub_ps(age, last_update);

    __m256 alpha = _mm256_div_ps(age, _mm256_set1_ps(AGE_FACTOR));

    // Applying an exponential function
    alpha = exp256_ps(alpha);  // Replace this with your implementation of exp for AVX2

    // Apply the aging factor to all belief states
    free = _mm256_mul_ps(free, alpha);
    occupancy = _mm256_mul_ps(occupancy, alpha);
    conflict = _mm256_mul_ps(conflict, alpha);
    unknown = _mm256_mul_ps(unknown, alpha);
    unknown = _mm256_add_ps(unknown, _mm256_sub_ps(_mm256_set1_ps(1.0), alpha));
}

__m256 exp256_ps(__m256 x) {
    x = _mm256_min_ps(x, _mm256_set1_ps(88.0f));
    x = _mm256_max_ps(x, _mm256_set1_ps(-88.0f));

    // coefficients for a Taylor Series approximation of exp(x) around 0
    __m256 c1 = _mm256_set1_ps(1.0f);
    __m256 c2 = _mm256_set1_ps(1.0f / 2.0f);
    __m256 c3 = _mm256_set1_ps(1.0f / 6.0f);
    __m256 c4 = _mm256_set1_ps(1.0f / 24.0f);

    __m256 x2 = _mm256_mul_ps(x, x);
    __m256 x3 = _mm256_mul_ps(x2, x);
    __m256 x4 = _mm256_mul_ps(x3, x);

    __m256 res = _mm256_add_ps(c1, x);
    res = _mm256_add_ps(res, _mm256_mul_ps(c2, x2));
    res = _mm256_add_ps(res, _mm256_mul_ps(c3, x3));
    res = _mm256_add_ps(res, _mm256_mul_ps(c4, x4));

    return res;
}
