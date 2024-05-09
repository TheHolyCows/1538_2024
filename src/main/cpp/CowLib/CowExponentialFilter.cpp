#include "CowExponentialFilter.h"

namespace CowLib
{

    CowExponentialFilter::CowExponentialFilter(double exponent)
    {
        m_Exponent = exponent;
    }

    double CowExponentialFilter::Filter(double input)
    {
        if (input == 0.0)
        {
            return 0;
        }

        input = pow(abs(input), m_Exponent) * input / abs(input);
        return input;
    }

    void CowExponentialFilter::Reset(double newExponent)
    {
        m_Exponent = newExponent;
    }

    double ExponentialFilter(double input, double exponent)
    {
        if (input == 0.0)
        {
            return 0.0;
        }

        return pow(abs(input), exponent) * input / abs(input);
    }

} /* namespace CowLib */
