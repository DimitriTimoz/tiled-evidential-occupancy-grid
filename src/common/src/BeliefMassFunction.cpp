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

#define VECTORIZE_MASSES(a, state) _mm256_set_ps(a[0]->getMass(state), a[1]->getMass(state), a[2]->getMass(state), a[3]->getMass(state), a[4]->getMass(state), a[5]->getMass(state), a[6]->getMass(state), a[7]->getMass(state))

void BeliefMassFunction::computeConjunctionLevels(BeliefMassFunction *a[8], const BeliefMassFunction *b[8])
{
    // General operation :
    // 1 : Free : (F_1 * F_2) + (F_1 * Omega_2) + (Omega_1 * F_2)
    // 2 : Occupied : (O_1 * O_2) + (O_1 * Omega_2) + (Omega_1 * O_2)
    // 3 : Conflict : (F_1 * O_2) + (O_1 * F_2)
    // 4 : Unknown : Omega_1 * Omega_2

    // - Free

    __m256 a_vector = VECTORIZE_MASSES(a, State::FREE);
    __m256 b_vector = VECTORIZE_MASSES(b, State::FREE);
    __m256 free_result = _mm256_mul_ps(a_vector, b_vector);

    a_vector = VECTORIZE_MASSES(a, State::FREE);
    b_vector = VECTORIZE_MASSES(b, State::UNKNOWN);
    free_result = _mm256_fmadd_ps(a_vector, b_vector, free_result);

    a_vector = VECTORIZE_MASSES(a, State::UNKNOWN);
    b_vector = VECTORIZE_MASSES(b, State::FREE);
    
    free_result = _mm256_fmadd_ps(a_vector, b_vector, free_result);

    // - Occupied

    a_vector = VECTORIZE_MASSES(a, State::OCCUPIED);
    b_vector = VECTORIZE_MASSES(b, State::OCCUPIED);
    __m256 occupied_result = _mm256_mul_ps(a_vector, b_vector);

    a_vector = VECTORIZE_MASSES(a, State::OCCUPIED);
    b_vector = VECTORIZE_MASSES(b, State::UNKNOWN);
    occupied_result = _mm256_fmadd_ps(a_vector, b_vector, occupied_result);

    a_vector = VECTORIZE_MASSES(a, State::UNKNOWN);
    b_vector = VECTORIZE_MASSES(b, State::OCCUPIED);
    occupied_result = _mm256_fmadd_ps(a_vector, b_vector, occupied_result);

    // - Conflict

    a_vector = VECTORIZE_MASSES(a, State::FREE);
    b_vector = VECTORIZE_MASSES(b, State::OCCUPIED);
    __m256 conflict_result = _mm256_mul_ps(a_vector, b_vector);

    a_vector = VECTORIZE_MASSES(a, State::OCCUPIED);
    b_vector = VECTORIZE_MASSES(b, State::FREE);
    conflict_result = _mm256_fmadd_ps(a_vector, b_vector, conflict_result);

    // - Unknown

    a_vector = VECTORIZE_MASSES(a, State::UNKNOWN);
    b_vector = VECTORIZE_MASSES(b, State::UNKNOWN);
    __m256 unknown_result = _mm256_mul_ps(a_vector, b_vector);

    // - Sum everything up for normalization

    __m256 sum = _mm256_add_ps(free_result, occupied_result);
    sum = _mm256_add_ps(sum, conflict_result);
    sum = _mm256_add_ps(sum, unknown_result);

    // - Normalize

    free_result = _mm256_div_ps(free_result, sum);
    occupied_result = _mm256_div_ps(occupied_result, sum);
    conflict_result = _mm256_div_ps(conflict_result, sum);
    unknown_result = _mm256_div_ps(unknown_result, sum);


    alignas(32) float results[4][8];
    _mm256_store_ps(results[0], free_result);
    _mm256_store_ps(results[1], occupied_result);
    _mm256_store_ps(results[2], conflict_result);
    _mm256_store_ps(results[3], unknown_result);

    for (int i = 0; i < 8; i++)
    {
        int j = 7 - i;
        a[j]->setMass(State::FREE, results[0][i]);
        a[j]->setMass(State::OCCUPIED, results[1][i]);
        a[j]->setMass(State::CONFLICT, results[2][i]);
        a[j]->setMass(State::UNKNOWN, results[3][i]);
    }

}

BeliefMassFunction BeliefMassFunction::operator+(const BeliefMassFunction &other) const
{
    BeliefMassFunction result;

    // - Conjunctive combination
    result.setMass(State::FREE, this->getConjunctionLevel(other, State::FREE));
    result.setMass(State::OCCUPIED, this->getConjunctionLevel(other, State::OCCUPIED));
    result.setMass(State::UNKNOWN, this->getConjunctionLevel(other, State::UNKNOWN));
    result.setMass(State::CONFLICT, this->getConjunctionLevel(other, State::CONFLICT));

    return result;
}

float BeliefMassFunction::getOccupancyProbability() const
{
    return this->getMass(State::OCCUPIED) / (1 - this->getMass(State::CONFLICT));
}

float BeliefMassFunction::getFreeProbability() const
{
    return this->getMass(State::FREE) / (1 - this->getMass(State::CONFLICT));
}

BeliefMassFunction &BeliefMassFunction::operator+=(BeliefMassFunction const &other)
{
    return *this = *this + other;
}

void BeliefMassFunction::computeProbabilitiesScaled(const BeliefMassFunction *a[8], float results[2][8], float scale)
{
    __m256 conflict = VECTORIZE_MASSES(a, State::CONFLICT);
    __m256 s = _mm256_set1_ps(scale);
    conflict = _mm256_fnmadd_ps(conflict, s, s);
    
    __m256 free = VECTORIZE_MASSES(a, State::FREE);
    __m256 occupied = VECTORIZE_MASSES(a, State::OCCUPIED);

    free = _mm256_div_ps(free, conflict);
    occupied = _mm256_div_ps(occupied, conflict);

    _mm256_store_ps(results[0], free);
    _mm256_store_ps(results[1], occupied);
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

// Like the previous function but with AVX2 instructions
void BeliefMassFunction::considerAges(float update_time, BeliefMassFunction *[8])
{
    __m256 occupancy = VECTORIZE_MASSES(a, State::OCCUPIED);
    __m256 free = VECTORIZE_MASSES(a, State::FREE);
    __m256 unknown = VECTORIZE_MASSES(a, State::UNKNOWN);
    __m256 conflict = VECTORIZE_MASSES(a, State::CONFLICT);

    __m256 age = _mm256_set1_ps(update_time);
    __m256 last_update = _mm256_load_ps(a[0]->last_update);
    age = _mm256_sub_ps(age, last_update);
    __m256 alpha = _mm256_div_ps(age, _mm256_set1_ps(AGE_FACTOR));
    alpha = _mm256_exp_ps(alpha);

    free = _mm256_mul_ps(free, alpha);
    occupancy = _mm256_mul_ps(occupancy, alpha);
    conflict = _mm256_mul_ps(conflict, alpha);
    unknown = _mm256_mul_ps(unknown, alpha);
    unknown = _mm256_add_ps(unknown, _mm256_sub_ps(_mm256_set1_ps(1.0), alpha));
}
