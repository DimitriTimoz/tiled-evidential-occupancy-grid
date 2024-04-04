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

#define VECTORIZE(a, state) _mm256_set_ps(a[0]->getMass(state), a[1]->getMass(state), a[2]->getMass(state), a[3]->getMass(state), a[4]->getMass(state), a[5]->getMass(state), a[6]->getMass(state), a[7]->getMass(state))

void BeliefMassFunction::computeConjunctionLevels(BeliefMassFunction *a[8], const BeliefMassFunction *b[8])
{
    // General operation :
    // 1 : Free : (F_1 * F_2) + (F_1 * Omega_2) + (Omega_1 * F_2)
    // 2 : Occupied : (O_1 * O_2) + (O_1 * Omega_2) + (Omega_1 * O_2)
    // 3 : Conflict : (F_1 * O_2) + (O_1 * F_2)
    // 4 : Unknown : Omega_1 * Omega_2

    // - Free

    __m256 a_vector = VECTORIZE(a, State::FREE);
    __m256 b_vector = VECTORIZE(b, State::FREE);
    __m256 free_result = _mm256_mul_ps(a_vector, b_vector);

    a_vector = VECTORIZE(a, State::FREE);
    b_vector = VECTORIZE(b, State::UNKNOWN);
    free_result = _mm256_fmadd_ps(a_vector, b_vector, free_result);

    a_vector = VECTORIZE(a, State::UNKNOWN);
    b_vector = VECTORIZE(b, State::FREE);
    
    free_result = _mm256_fmadd_ps(a_vector, b_vector, free_result);

    // - Occupied

    a_vector = VECTORIZE(a, State::OCCUPIED);
    b_vector = VECTORIZE(b, State::OCCUPIED);
    __m256 occupied_result = _mm256_mul_ps(a_vector, b_vector);

    a_vector = VECTORIZE(a, State::OCCUPIED);
    b_vector = VECTORIZE(b, State::UNKNOWN);
    occupied_result = _mm256_fmadd_ps(a_vector, b_vector, occupied_result);

    a_vector = VECTORIZE(a, State::UNKNOWN);
    b_vector = VECTORIZE(b, State::OCCUPIED);
    occupied_result = _mm256_fmadd_ps(a_vector, b_vector, occupied_result);

    // - Conflict

    a_vector = VECTORIZE(a, State::FREE);
    b_vector = VECTORIZE(b, State::OCCUPIED);
    __m256 conflict_result = _mm256_mul_ps(a_vector, b_vector);

    a_vector = VECTORIZE(a, State::OCCUPIED);
    b_vector = VECTORIZE(b, State::FREE);
    conflict_result = _mm256_fmadd_ps(a_vector, b_vector, conflict_result);

    // - Unknown

    a_vector = VECTORIZE(a, State::UNKNOWN);
    b_vector = VECTORIZE(b, State::UNKNOWN);
    __m256 unknown_result = _mm256_mul_ps(a_vector, b_vector);


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