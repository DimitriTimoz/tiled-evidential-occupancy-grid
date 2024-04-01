#include "common/BeliefMassFunction.h"

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

void BeliefMassFunction::setMass(State state, float mass)
{
    switch (state)
    {
    case State::FREE:
        this->masses[0] = mass;
        break;
    case State::OCCUPIED:
        this->masses[1] = mass;
        break;
    case State::UNKNOWN:
        this->masses[2] = mass;
        break;
    case State::CONFLICT:
        this->masses[3] = mass;
        break;
    default:
        break;
    }
}

float BeliefMassFunction::getMass(State state) const
{
    switch (state)
    {
    case State::FREE:
        return this->masses[0];
    case State::OCCUPIED:
        return this->masses[1];
    case State::UNKNOWN:
        return this->masses[2];
    case State::CONFLICT:
        return this->masses[3];
    default:
        return 0.0;
    }
}

BeliefMassFunction::State BeliefMassFunction::getState() const
{
    int maximum_index = -1; // This should raise an runtime error if all mass values are equals to 0 which is not possible
    float maximum_mass = 0.0;
    for (int i = static_cast<int>(State::FREE); i < sizeof(BeliefMassFunction::masses) / sizeof(float); i++)
    {
        if (this->masses[i] > maximum_mass) {
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

float BeliefMassFunction::getOccupancyProbability() const
{
    float sum = 0.0;
    for (int i = static_cast<int>(State::FREE); i < sizeof(BeliefMassFunction::masses) / sizeof(float); i++)
    {
        sum += this->masses[i];
    }

    return this->getMass(State::OCCUPIED) / sum;
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

BeliefMassFunction &BeliefMassFunction::operator+=(const BeliefMassFunction &other)
{
    return *this = *this + other;
}