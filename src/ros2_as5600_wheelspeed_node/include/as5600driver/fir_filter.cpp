/**
 * @file fir_filter.cpp
 * @brief Implementation of an FIR filter using a ring buffer.
 * The method assumes the pointer to the array of coefficients is
 * computed offline. This can be done using various programs in
 * MATLAB and Python's Scipy. The array of coefficients should be
 * ordered as [b0,b1,b2,...] where b0 is the coefficient
 * corresponding to the oldest input (largest delay).
*/

/**
 * @brief FIR Filter implementation with a ring buffer.
 * This requires the number of taps and the coefficients for each tap.
*/
class FIRFilter {
public:
    // constructor
    FIRFilter(int numTaps, const double* pCoeffs) :
        m_numTaps(numTaps), m_pCoeffs(pCoeffs), m_pBuffer(new double[numTaps]), m_index(0) {
        // initialize the buffer to zero
        for (int i = 0; i < numTaps; i++) {
            m_pBuffer[i] = 0.0;
        }
    }

    // destructor
    ~FIRFilter() {
        delete[] m_pBuffer;
    }

    // filter a single sample
    double filter(double input) {
        // store the input in the buffer
        m_pBuffer[m_index] = input;

        // update the index
        m_index++;
        // Reset the index on the buffer if greater
        // than the buffer length.
        if (m_index >= m_numTaps) {
            m_index = 0;
        }

        // Calculate the output.
        double output = 0.0;
        int tapIndex = m_index;
        for (int i = 0; i < m_numTaps; i++) {
            output += m_pCoeffs[i] * m_pBuffer[tapIndex];
            tapIndex++;
            // When at the end of the buffer
            if (tapIndex >= m_numTaps) {
                tapIndex = 0;
            }
        }

        return output;
    }

private:
    int m_numTaps;
    const double* m_pCoeffs;
    double* m_pBuffer;
    int m_index;
};