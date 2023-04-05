/**
 * FIR Filter C++ class
 *
 *
 * @author Shaqeeb Momen
 * @date Oct.27,2022
 */

#include "../MemFilt.hpp"


template <typename T, int size>
class FIRFilt : public MemFilt<T,size>
{
private:
    T *m_coeffs;                  // Coefficients to use
    T calculate();                // Perform the product sum operation
    void updateCoeffs(T *coeffs); // Update the coefficients

public:
    FIRFilt(T *);
};

/**
 * @brief Construct a FIR filter object, passing in a pointer to an array of coefficients
 * @tparam T Numerical type for calculations
 * @tparam size Length of the filter
 * @param coeffs pointer to the array where the coefficients are store,
 * first element applied to the most recent sample
 */
template <typename T, int size>
FIRFilt<T, size>::FIRFilt(T *coeffs)
{
    this->m_coeffs = coeffs;
}

// Perform product-sum operation through samples and taps
template <typename T, int size>
T FIRFilt<T, size>::calculate()
{
    this->m_output = 0;
    for (size_t i = 0; i < size; i++)
    {
        this->m_output += this->m_taps[i] * this->m_coeffs[i];
    }
    return this->m_output;
}
