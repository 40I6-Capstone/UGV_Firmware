#include <stddef.h>

template <typename T, int size>
class MemFilt
{
private:

protected:
    T m_taps[size];        // Previous inputs to use, first element is most recent sample
    T m_output;                // Current Output
    void shift(T val);         // Shift new value into tap array
    virtual T calculate() = 0; // Perform the product sum operation

public:
    MemFilt();
    void update(T newval);
    T getOutput(); // Getter for output
};

/**
 * @brief constructor for filter, allocates memory needed to store taps and sets them to 0
 */
template <typename T, int size>
MemFilt<T, size>::MemFilt()
{
    // Initialize taps to 0
    for (size_t i = 0; i < size; i++)
    {
        this->m_taps[i] = 0;
    }
}

// Shift in new sample
template <typename T, int size>
void MemFilt<T, size>::shift(T val)
{
    for (size_t i = size - 1; i > 0; i--)
    {
        this->m_taps[i] = this->m_taps[i - 1];
    }
    this->m_taps[0] = val;
}

/**
 * @brief Update the filter with a new sampled value
 * @param newval new sample to pass in
 */
template <typename T, int size>
void MemFilt<T, size>::update(T newval)
{
    this->shift(newval);
    this->calculate();
}

/**
 * @brief Return the current output of the filter
 */
template <typename T, int size>
T MemFilt<T, size>::getOutput()
{
    return this->m_output;
}